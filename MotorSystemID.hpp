#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>

namespace SysId {

/** Motor calibration and closed-loop J/b identification capture.
 * Default: 30s ZERO-torque capture of a marked manual shaft rotation.
 * Optional: repeated one-speed +/- holds, after explicit unit confirmation.
 * PRBS_PD captures total applied torque for an offline integral fit.
 */
class MotorSystem {
 public:
  enum class TestMode : uint32_t {
    CALIBRATION = 0,
    LONG_SPEED = 1,
    PRBS_PD = 2
  };

  struct Param {
    TestMode mode = TestMode::CALIBRATION;
    float calibration_time = 30.0f;
    float calibration_log_period = 0.02f;
    float test_speed = 0.15f;
    float start_time = 3.0f;
    float hold_time = 10.0f;
    float rest_time = 4.0f;
    float speed_kp = 3.0f;
    float speed_ki = 2.0f;
    float acceleration_limit = 0.2f;
    float torque_limit = 2.0f;
    float speed_limit = 0.6f;
    float long_log_period = 0.05f;
  };

  enum class Status : uint32_t {
    WAITING,
    RUNNING,
    DONE,
    BAD_INPUT,
    FEEDBACK_LOST,
    OVERSPEED,
    POSITION_JUMP,
    BUFFER_FULL,
    UNITS_UNCONFIRMED,
    STOP_NOT_SETTLED,
    TRAVEL_LIMIT,
    OVER_TEMPERATURE,
    INVALID_COMMAND,
    IMU_OFFLINE,
    PITCH_MOVED
  };
  enum Flags : uint32_t { NONE = 0, SATURATED = 1 };

  // 32 bytes, exactly 48 KiB for the whole buffer. NOT the old 24-byte format.
  // tau means are over (previous record time, this record time]. The command
  // accumulated for a dt is the command returned at the PREVIOUS control step.
  struct Sample {
    float time;
    float tau_cmd_mean;
    float theta;
    float omega_raw;  // PRBS: signed IMU endpoint speed; legacy: interval mean
    float omega_ref;  // IMU PRBS: motor endpoint speed for comparison
    float tau_feedback_mean;
    uint32_t stage;
    uint32_t flags;
  };
  static constexpr uint32_t MAX_SAMPLES = 1536;
  static_assert(sizeof(Sample) == 32, "Motor sysid sample layout changed");

  MotorSystem() : MotorSystem(Param()) {}
  explicit MotorSystem(const Param& param)
      : test_mode_(param.mode), param_(param) {}

  // angle is the decoded motor output-shaft position, not CycleValue/IMU yaw.
  // fresh means enabled + a recently decoded motor feedback frame.
  float Step(float angle, float omega_raw, float tau_feedback, float dt,
             bool fresh) {
    if (done_) {
      return 0.0f;
    }
    if (!std::isfinite(dt) || dt <= 0.0f || dt > 0.02f ||
        !std::isfinite(angle) || !std::isfinite(omega_raw) ||
        !std::isfinite(tau_feedback)) {
      return Stop(Status::BAD_INPUT);
    }
    if (!started_) {
      latched_mode_ = test_mode_;
      active_mode_ = latched_mode_;
      if (latched_mode_ == TestMode::PRBS_PD) {
        amplitude_latched_ = amplitude_;
        kp_latched_ = kp_;
        kd_latched_ = kd_;
        travel_latched_ = max_travel_;
        limit_latched_ = torque_limit_;
        duration_latched_ = duration_s_;
        hold_latched_ = hold_s_;
        seed_latched_ = seed_;
        if (!std::isfinite(amplitude_latched_) || !std::isfinite(kp_latched_) ||
            !std::isfinite(kd_latched_) || !std::isfinite(travel_latched_) ||
            !std::isfinite(limit_latched_) ||
            !std::isfinite(duration_latched_) ||
            !std::isfinite(hold_latched_) || amplitude_latched_ <= 0 ||
            kp_latched_ <= 0 || kd_latched_ < 0 || travel_latched_ <= 0 ||
            travel_latched_ > 0.5f || limit_latched_ <= 0 ||
            limit_latched_ > 2.0f || amplitude_latched_ > limit_latched_ ||
            duration_latched_ <= 2 || duration_latched_ > 14 ||
            hold_latched_ < .02f || hold_latched_ > 1 || seed_latched_ == 0 ||
            seed_latched_ > 65535)
          return Stop(Status::BAD_INPUT);
        lfsr_ = static_cast<uint16_t>(seed_latched_);
      }
      if (!ValidConfig()) {
        return Stop(Status::BAD_INPUT);
      }
    }
    if (latched_mode_ != TestMode::CALIBRATION && !units_confirmed_) {
      return Stop(Status::UNITS_UNCONFIRMED);
    }
    if (!fresh) {
      wait_time_ += dt;
      if (started_ || wait_time_ > 5.0f) {
        return Stop(Status::FEEDBACK_LOST);
      }
      return 0.0f;
    }
    if (!started_) {
      started_ = true;
      status_ = Status::RUNNING;
      last_angle_ = angle;
      stage_ = latched_mode_ == TestMode::CALIBRATION ? 100
               : latched_mode_ == TestMode::PRBS_PD   ? 400
                                                      : 200;
      PushHistory(0.0f, 0.0f);
      Record(omega_raw, tau_feedback);  // t=0 baseline, zero input interval
      return 0.0f;
    }

    const float DELTA = std::remainder(angle - last_angle_, TWO_PI);
    const float LIMIT = latched_mode_ == TestMode::CALIBRATION
                            ? CALIBRATION_SPEED_LIMIT
                            : param_.speed_limit;
    // Reject ambiguous wrap jumps; do not silently invent a correction scale.
    if (std::fabs(DELTA) > LIMIT * dt + 0.02f) {
      return Stop(Status::POSITION_JUMP);
    }
    last_angle_ = angle;
    theta_ = theta_ + DELTA;
    if (latched_mode_ == TestMode::PRBS_PD &&
        std::fabs(static_cast<float>(theta_)) >= travel_latched_)
      return Stop(Status::TRAVEL_LIMIT);
    time_ += dt;
    PushHistory(time_, theta_);
    EstimateSpeed();
    if (std::fabs(static_cast<float>(omega_)) > LIMIT) {
      return Stop(Status::OVERSPEED);
    }

    // Integrate the input actually held during this elapsed control interval.
    tau_integral_ += previous_tau_ * dt;
    feedback_integral_ += tau_feedback * dt;
    omega_integral_ += omega_raw * dt;
    log_dt_ += dt;
    const float INPUT_LIMIT = latched_mode_ == TestMode::PRBS_PD
                                  ? limit_latched_
                                  : param_.torque_limit;
    if (std::fabs(previous_tau_) >= INPUT_LIMIT * 0.999f) {
      log_flags_ |= SATURATED;
    }

    if (time_ >= Duration()) {
      if (log_dt_ > 0.0f && !Record(omega_raw, tau_feedback)) {
        return 0.0f;
      }
      if (latched_mode_ == TestMode::LONG_SPEED &&
          std::fabs(static_cast<float>(omega_)) > 0.03f) {
        return Stop(Status::STOP_NOT_SETTLED);
      }
      return Stop(Status::DONE);
    }

    float tau = 0.0f;
    if (latched_mode_ == TestMode::PRBS_PD) {
      float excitation = 0.0f;
      stage_ = time_ < 1.0f ? 400 : time_ >= duration_latched_ - 1 ? 403 : 401;
      if (stage_ == 401) {
        if (time_ >= next_bit_s_) {
          const uint16_t BIT = static_cast<uint16_t>(
              (lfsr_ ^ (lfsr_ >> 2) ^ (lfsr_ >> 3) ^ (lfsr_ >> 5)) & 1U);
          lfsr_ = static_cast<uint16_t>((lfsr_ >> 1) | (BIT << 15));
          excitation_ = amplitude_latched_ *
                        (0.5f * static_cast<float>(lfsr_ & 3U) - .75f);
          next_bit_s_ = time_ + hold_latched_;
        }
        excitation = excitation_;
      }
      excitation_now_ = excitation;
      const float RAW = excitation - kp_latched_ * static_cast<float>(theta_) -
                        kd_latched_ * omega_raw;
      tau = std::clamp(RAW, -limit_latched_, limit_latched_);
      if (tau != RAW) log_flags_ |= SATURATED;
    } else if (latched_mode_ == TestMode::CALIBRATION) {
      stage_ = 100;
      omega_ref_ = 0.0f;
    } else {
      const float TARGET = TargetSpeed();
      const float DELTA_REF = param_.acceleration_limit * dt;
      reference_ += std::clamp(TARGET - reference_, -DELTA_REF, DELTA_REF);
      omega_ref_ = reference_;
      const float ERROR = reference_ - static_cast<float>(omega_);
      const float CANDIDATE = integral_ + param_.speed_ki * ERROR * dt;
      const float RAW = param_.speed_kp * ERROR + CANDIDATE;
      tau = std::clamp(RAW, -param_.torque_limit, param_.torque_limit);
      if (RAW == tau || RAW * ERROR <= 0.0f) {
        integral_ = CANDIDATE;
      }
    }
    if (log_dt_ >= LogPeriod()) {
      if (!Record(omega_raw, tau_feedback)) {
        return 0.0f;
      }
    }
    previous_tau_ = tau;
    tau_cmd_ = tau;
    return tau;
  }

  void Reset() {
    // Preserve the explicit mode/unit choice; latch mode only on next entry.
    // Editing test_mode_ mid-capture cannot enable powered motion.
    write_idx_ = 0;
    count_ = 0;
    tau_cmd_ = 0.0f;
    theta_ = 0.0f;
    omega_ = 0.0f;
    omega_ref_ = 0.0f;
    stage_ = 0;
    done_ = false;
    status_ = Status::WAITING;
    started_ = false;
    active_mode_ = test_mode_;
    time_ = 0.0f;
    wait_time_ = 0.0f;
    last_angle_ = 0.0f;
    reference_ = 0.0f;
    integral_ = 0.0f;
    previous_tau_ = 0.0f;
    tau_integral_ = 0.0f;
    feedback_integral_ = 0.0f;
    omega_integral_ = 0.0f;
    log_dt_ = 0.0f;
    log_flags_ = NONE;
    history_count_ = 0;
    history_index_ = 0;
    next_bit_s_ = 0;
    excitation_ = 0;
    excitation_now_ = 0;
  }

  Sample buf_[MAX_SAMPLES];  // first member: dump exactly sizeof(buf_)
  volatile uint32_t write_idx_ = 0;
  volatile uint32_t count_ = 0;
  volatile float tau_cmd_ = 0.0f;
  volatile float theta_ = 0.0f;
  volatile float omega_ =
      0.0f;  // finite-window encoder slope, used by speed PI
  volatile float omega_ref_ = 0.0f;
  volatile float omega_motor_ = 0.0f;
  volatile uint32_t stage_ = 0;
  volatile bool done_ = false;
  volatile Status status_ = Status::WAITING;
  volatile TestMode test_mode_;
  volatile bool units_confirmed_ = false;
  volatile TestMode active_mode_ = TestMode::CALIBRATION;

  volatile float amplitude_ = .10f;  // four levels: +/- .75A, +/- .25A
  volatile float kp_ = 2.0f;
  volatile float kd_ = .12f;
  volatile float max_travel_ = .06f;
  volatile float torque_limit_ = .20f;
  volatile float duration_s_ = 14.0f;
  volatile float hold_s_ = .10f;
  volatile uint32_t seed_ = 0xACE1;
  volatile float excitation_now_ = 0.0f;
  void Cancel(Status reason) { Stop(reason); }

 private:
  static constexpr float TWO_PI = 6.2831853071795864769f;
  static constexpr float CALIBRATION_SPEED_LIMIT = 2.0f;
  static constexpr uint32_t HISTORY_SIZE = 32;
  static constexpr float SPEED_WINDOW = 0.04f;

  struct History {
    float time;
    float angle;
  };
  Param param_;
  History history_[HISTORY_SIZE]{};
  uint32_t history_count_ = 0;
  uint32_t history_index_ = 0;
  bool started_ = false;
  TestMode latched_mode_ = TestMode::CALIBRATION;
  float time_ = 0.0f;
  float wait_time_ = 0.0f;
  float last_angle_ = 0.0f;
  float reference_ = 0.0f;
  float integral_ = 0.0f;
  float previous_tau_ = 0.0f;
  float tau_integral_ = 0.0f;
  float feedback_integral_ = 0.0f;
  float omega_integral_ = 0.0f;
  float log_dt_ = 0.0f;
  uint32_t log_flags_ = NONE;
  float amplitude_latched_ = 0, kp_latched_ = 0, kd_latched_ = 0;
  float travel_latched_ = 0, limit_latched_ = 0, duration_latched_ = 0;
  float hold_latched_ = 0, next_bit_s_ = 0, excitation_ = 0;
  uint32_t seed_latched_ = 0;
  uint16_t lfsr_ = 0;

  float Duration() const {
    if (latched_mode_ == TestMode::PRBS_PD) return duration_latched_;
    return latched_mode_ == TestMode::CALIBRATION
               ? param_.calibration_time
               : param_.start_time +
                     4.0f * (param_.hold_time + param_.rest_time);
  }

  float LogPeriod() const {
    if (latched_mode_ == TestMode::PRBS_PD) return .01f;
    return latched_mode_ == TestMode::CALIBRATION
               ? param_.calibration_log_period
               : param_.long_log_period;
  }

  bool ValidConfig() const {
    if (latched_mode_ != TestMode::CALIBRATION &&
        latched_mode_ != TestMode::LONG_SPEED &&
        latched_mode_ != TestMode::PRBS_PD) {
      return false;
    }
    const float VALUES[] = {
        param_.calibration_time,   param_.calibration_log_period,
        param_.test_speed,         param_.start_time,
        param_.hold_time,          param_.rest_time,
        param_.speed_kp,           param_.speed_ki,
        param_.acceleration_limit, param_.torque_limit,
        param_.speed_limit,        param_.long_log_period};
    for (float value : VALUES) {
      if (!std::isfinite(value) || value <= 0.0f) {
        return false;
      }
    }
    return param_.test_speed < param_.speed_limit &&
           Duration() / LogPeriod() + 2.0f <= MAX_SAMPLES;
  }

  float TargetSpeed() {
    if (time_ < param_.start_time) {
      stage_ = 200;
      return 0.0f;
    }
    const float CYCLE_TIME = param_.hold_time + param_.rest_time;
    const float ELAPSED = time_ - param_.start_time;
    const uint32_t CYCLE = static_cast<uint32_t>(ELAPSED / CYCLE_TIME);
    const float PHASE = ELAPSED - static_cast<float>(CYCLE) * CYCLE_TIME;
    const bool HOLD = PHASE < param_.hold_time;
    stage_ = 201 + 2 * CYCLE + (HOLD ? 0 : 1);
    return HOLD ? (CYCLE % 2 == 0 ? param_.test_speed : -param_.test_speed)
                : 0.0f;
  }

  void PushHistory(float time, float angle) {
    history_[history_index_] = {time, angle};
    history_index_ = (history_index_ + 1) % HISTORY_SIZE;
    history_count_ = std::min(history_count_ + 1, HISTORY_SIZE);
  }

  void EstimateSpeed() {
    // Position slope over at least ~40ms, avoiding noisy two-sample
    // derivatives. Select the oldest sample still needed for that window.
    const float TIME = time_;
    for (uint32_t age = 1; age < history_count_; ++age) {
      const uint32_t INDEX =
          (history_index_ + HISTORY_SIZE - 1 - age) % HISTORY_SIZE;
      const float SPAN = TIME - history_[INDEX].time;
      if (SPAN >= SPEED_WINDOW || age + 1 == history_count_) {
        if (SPAN > 0.0f) {
          omega_ = (static_cast<float>(theta_) - history_[INDEX].angle) / SPAN;
        }
        break;
      }
    }
  }

  bool Record(float omega_raw, float tau_feedback) {
    if (count_ >= MAX_SAMPLES) {
      Stop(Status::BUFFER_FULL);
      return false;
    }
    const float TAU = log_dt_ > 0.0f ? tau_integral_ / log_dt_ : 0.0f;
    const float FEEDBACK =
        log_dt_ > 0.0f ? feedback_integral_ / log_dt_ : tau_feedback;
    const float SPEED = latched_mode_ == TestMode::PRBS_PD ? omega_raw
                        : log_dt_ > 0.0f ? omega_integral_ / log_dt_
                                         : omega_raw;
    const float AUX_SPEED =
        latched_mode_ == TestMode::PRBS_PD ? omega_motor_ : omega_ref_;
    buf_[count_] = {time_,     TAU,      theta_, SPEED,
                    AUX_SPEED, FEEDBACK, stage_, log_flags_};
    count_ = count_ + 1;
    write_idx_ = count_;  // linear, non-wrapping capture; equal to count
    tau_integral_ = 0.0f;
    feedback_integral_ = 0.0f;
    omega_integral_ = 0.0f;
    log_dt_ = 0.0f;
    log_flags_ = NONE;
    return true;
  }

  float Stop(Status status) {
    tau_cmd_ = 0.0f;
    previous_tau_ = 0.0f;
    status_ = status;
    done_ = true;
    return 0.0f;
  }
};

}  // namespace SysId
