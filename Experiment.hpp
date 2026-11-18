#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>

// Restored Double Yaw excitation and interval-recording core. No hardware
// access. Generic: no motor type, machine zero/limits or known inertia is
// embedded. Input torque, feedback torque and both rates use position-positive
// signs.
namespace SysId {
class Experiment {
public:
  enum Status : uint32_t {
    IDLE = 0,
    WAITING = 0,
    RUNNING = 1,
    DONE = 2,
    INVALID_INPUT = 3,
    BAD_INPUT = 3,
    MOTOR_OFFLINE = 4,
    FEEDBACK_LOST = 4,
    OVERSPEED = 5,
    POSITION_LIMIT = 6,
    POSITION_JUMP = 6,
    BUFFER_FULL = 7,
    UNITS_UNCONFIRMED = 8,
    STOP_NOT_SETTLED = 9,
    TRAVEL_LIMIT = 10,
    OVER_TEMPERATURE = 11,
    INVALID_COMMAND = 12,
    IMU_OFFLINE = 13,
    PITCH_MOVED = 14,
    TX_QUEUE_FAILED = 15,
    MANUAL_STOP = 16,
    BAD_DT = 17,
    TX_NOT_OBSERVED = 18
  };
  enum Flag : uint32_t { SATURATED = 1, MOTOR_REUSED = 2, GYRO_REUSED = 4 };
  static constexpr uint32_t CAPACITY = 1536;
  // Supplied by the CAN torque encoding, never a fixed motor model.
  volatile float torque_constant_ = 0.0f;
  volatile float command_quantum_ = 0.0f;
  struct Header {
    uint32_t magic, version, sample_bytes, capacity;
    uint32_t count, status, seed, reserved_flags;
    float center, min_position, max_position, amplitude;
    float kp, kd, torque_limit, max_travel;
    float settle_s, excite_s, return_s, log_period_s;
    float hold_fast_s, hold_mid_s, hold_slow_s;
    float torque_constant, command_quantum;
    uint32_t saturation_intervals, feedback_stale_intervals;
    float elapsed_s;
    float reserved[4];
  };
  struct Sample {
    float time_s, position, omega_gyro, omega_motor;
    // Interval ending at time_s. Commands are AFTER PD, clipping and GM6020
    // quantization; feedback is current-derived torque, not a torque sensor.
    float torque_cmd_mean, torque_feedback_mean, excitation_mean, pd_mean;
    float pitch, gyro_y;
    uint32_t stage, flags;
  };
  struct Capture {
    Header header;
    Sample samples[CAPACITY];
  };
  struct Input {
    float dt, position, omega_gyro, omega_motor, torque_feedback;
    float pitch, gyro_y, temperature;
    bool motor_online, imu_online, motor_new, gyro_new;
  };

  // Ozone: edit parameters while disabled, then set enable_=true. Parameters
  // are latched for the entire capture. Starting again overwrites the old run.
  volatile bool enable_ = false;
  volatile bool done_ = false;
  volatile Status status_ = IDLE;
  volatile uint32_t count_ = 0;
  volatile uint32_t stage_ = 0;
  volatile float time_s_ = 0.0f;
  volatile float position_ = 0.0f;
  volatile float omega_ = 0.0f;
  volatile float tau_cmd_ = 0.0f;
  volatile float prbs_ = 0.0f;
  volatile float pd_ = 0.0f;
  volatile float amplitude_ = 0.07f;
  volatile float kp_ = 2.0f;
  volatile float kd_ = 0.12f;
  volatile float torque_limit_ = 0.20f;
  volatile float max_travel_ = 0.06f;
  // Absolute limits belong to the application YAML.
  volatile float min_position_ = -1000000.0f;
  volatile float max_position_ = 1000000.0f;
  volatile uint32_t seed_ = 0xACE1;
  Capture capture_{};

  bool Active() const { return active_; }
  void Cancel(Status reason) { Stop(reason); }

  float Step(const Input &input) {
    if (!enable_) {
      return active_ ? Stop(MANUAL_STOP, &input) : 0.0f;
    }
    const bool STARTING = !active_;
    if (STARTING && !Start(input))
      return 0.0f;
    if (!Finite(input))
      return Stop(INVALID_INPUT);
    if (!input.motor_online)
      return Stop(MOTOR_OFFLINE, &input);
    if (!input.imu_online)
      return Stop(IMU_OFFLINE, &input);
    if (input.dt <= 0.0f || input.dt > 0.02f)
      return Stop(BAD_DT);
    if (input.temperature >= 75.0f)
      return Stop(OVER_TEMPERATURE, &input);
    if (STARTING)
      return 0.0f;
    // Record the input held over the elapsed interval, BEFORE computing a new
    // command. This avoids attributing the next PRBS bit to the last response.
    time_ += input.dt;
    interval_ += input.dt;
    cmd_integral_ += previous_tau_ * input.dt;
    feedback_integral_ += input.torque_feedback * input.dt;
    excitation_integral_ += previous_prbs_ * input.dt;
    pd_integral_ += previous_pd_ * input.dt;
    if (previous_saturated_) {
      flags_ |= SATURATED;
      capture_.header.saturation_intervals++;
    }
    if (!input.motor_new) {
      flags_ |= MOTOR_REUSED;
      capture_.header.feedback_stale_intervals++;
    }
    if (!input.gyro_new)
      flags_ |= GYRO_REUSED;
    position_ = input.position;
    omega_ = input.omega_gyro;
    time_s_ = time_;
    capture_.header.elapsed_s = time_;
    const auto &H = capture_.header;
    if (input.position <= H.min_position + 0.015f ||
        input.position >= H.max_position - 0.015f)
      return Stop(POSITION_LIMIT, &input);
    if (std::fabs(input.position - H.center) >= H.max_travel)
      return Stop(TRAVEL_LIMIT, &input);
    if (std::fabs(input.omega_gyro) > 3.0f ||
        std::fabs(input.omega_motor) > 3.0f)
      return Stop(OVERSPEED, &input);
    if (std::fabs(input.pitch - initial_pitch_) > 0.05f)
      return Stop(PITCH_MOVED, &input);
    if (interval_ >= H.log_period_s && !Record(input))
      return Stop(BUFFER_FULL);
    if (time_ >= H.settle_s + H.excite_s + H.return_s)
      return Stop(DONE, &input);

    const float EXCITE_TIME = time_ - H.settle_s;
    float excitation = 0.0f;
    uint32_t stage = 0;
    if (EXCITE_TIME >= 0.0f && EXCITE_TIME < H.excite_s) {
      stage = 1U + static_cast<uint32_t>(EXCITE_TIME / (H.excite_s / 3.0f));
      const float HOLD = stage == 1   ? H.hold_fast_s
                         : stage == 2 ? H.hold_mid_s
                                      : H.hold_slow_s;
      if (stage != previous_stage_ || time_ >= next_bit_time_) {
        const uint16_t BIT = static_cast<uint16_t>(
            (lfsr_ ^ (lfsr_ >> 2) ^ (lfsr_ >> 3) ^ (lfsr_ >> 5)) & 1U);
        lfsr_ = static_cast<uint16_t>((lfsr_ >> 1) | (BIT << 15));
        sign_ = (lfsr_ & 1U) ? 1.0f : -1.0f;
        next_bit_time_ = time_ + HOLD;
      }
      excitation = sign_ * H.amplitude * (stage == 2 ? 0.65f : 1.0f);
    } else if (EXCITE_TIME >= H.excite_s) {
      stage = 4;
    }
    const float PD =
        -H.kp * (input.position - H.center) - H.kd * input.omega_gyro;
    const float RAW = excitation + PD;
    const float CLIPPED = std::clamp(RAW, -H.torque_limit, H.torque_limit);
    // Prediction only: CommandQueued replaces this with the actual packed
    // value.
    const float QUANTUM = command_quantum_;
    previous_tau_ =
        QUANTUM > 0 ? std::trunc(CLIPPED / QUANTUM) * QUANTUM : CLIPPED;
    previous_prbs_ = excitation;
    previous_pd_ = PD;
    previous_saturated_ = RAW != CLIPPED;
    previous_stage_ = stage;
    stage_ = stage;
    tau_cmd_ = previous_tau_;
    prbs_ = excitation;
    pd_ = PD;
    return CLIPPED;
  }

  void TxFailed() {
    if (active_)
      Stop(TX_QUEUE_FAILED);
  }
  // Actual packed torque observed at CAN AddMessage, in position-positive
  // units.
  void CommandQueued(float torque) {
    if (active_) {
      previous_tau_ = torque;
      tau_cmd_ = torque;
    }
  }

private:
  static bool Finite(const Input &input) {
    return std::isfinite(input.position) && std::isfinite(input.omega_gyro) &&
           std::isfinite(input.omega_motor) &&
           std::isfinite(input.torque_feedback) && std::isfinite(input.pitch) &&
           std::isfinite(input.gyro_y) && std::isfinite(input.temperature) &&
           std::isfinite(input.dt);
  }
  bool Start(const Input &input) {
    auto &h = capture_.header;
    h = {};
    h.magic = 0x59534944;
    h.version = 2;
    h.sample_bytes = sizeof(Sample);
    h.capacity = CAPACITY;
    h.seed = seed_;
    h.center = input.position;
    h.min_position = min_position_;
    h.max_position = max_position_;
    h.amplitude = amplitude_;
    h.kp = kp_;
    h.kd = kd_;
    h.torque_limit = torque_limit_;
    h.max_travel = max_travel_;
    h.settle_s = 2.0f;
    h.excite_s = 12.0f;
    h.return_s = 1.0f;
    h.log_period_s = 0.01f;
    h.hold_fast_s = 0.04f;
    h.hold_mid_s = 0.10f;
    h.hold_slow_s = 0.25f;
    h.torque_constant = torque_constant_;
    h.command_quantum = command_quantum_;
    count_ = 0;
    done_ = false;
    time_ = 0.0f;
    time_s_ = 0.0f;
    interval_ = 0.0f;
    cmd_integral_ = feedback_integral_ = 0.0f;
    excitation_integral_ = pd_integral_ = 0.0f;
    flags_ = 0;
    previous_tau_ = previous_prbs_ = previous_pd_ = 0.0f;
    previous_saturated_ = false;
    previous_stage_ = 0;
    lfsr_ = static_cast<uint16_t>(h.seed);
    next_bit_time_ = 0.0f;
    initial_pitch_ = input.pitch;
    const float VALUES[] = {h.center,       h.min_position, h.max_position,
                            h.amplitude,    h.kp,           h.kd,
                            h.torque_limit, h.max_travel};
    for (const float VALUE : VALUES) {
      if (!std::isfinite(VALUE)) {
        Stop(INVALID_INPUT);
        return false;
      }
    }
    if (!Finite(input) || h.amplitude <= 0.0f || h.amplitude > h.torque_limit ||
        h.kp <= 0.0f || h.kd < 0.0f || h.torque_limit > 0.5f ||
        h.max_travel <= 0.005f || h.max_travel > 0.2f || h.seed == 0 ||
        h.seed > 65535 || h.min_position + 0.015f >= h.center ||
        h.max_position - 0.015f <= h.center) {
      Stop(INVALID_INPUT);
      return false;
    }
    active_ = true;
    status_ = RUNNING;
    h.status = RUNNING;
    stage_ = 0;
    position_ = input.position;
    omega_ = input.omega_gyro;
    // Initial sample is the left endpoint, with no elapsed input interval.
    capture_.samples[0] = {0.0f,
                           input.position,
                           input.omega_gyro,
                           input.omega_motor,
                           0.0f,
                           input.torque_feedback,
                           0.0f,
                           0.0f,
                           input.pitch,
                           input.gyro_y,
                           0,
                           0};
    h.count = 1;
    count_ = 1;
    return true;
  }
  bool Record(const Input &input) {
    auto &h = capture_.header;
    if (h.count >= CAPACITY || interval_ <= 0.0f)
      return false;
    capture_.samples[h.count] = {time_,
                                 input.position,
                                 input.omega_gyro,
                                 input.omega_motor,
                                 cmd_integral_ / interval_,
                                 feedback_integral_ / interval_,
                                 excitation_integral_ / interval_,
                                 pd_integral_ / interval_,
                                 input.pitch,
                                 input.gyro_y,
                                 previous_stage_,
                                 flags_};
    h.count++;
    count_ = h.count;
    interval_ = 0.0f;
    cmd_integral_ = feedback_integral_ = 0.0f;
    excitation_integral_ = pd_integral_ = 0.0f;
    flags_ = 0;
    return true;
  }
  float Stop(Status status, const Input *input = nullptr) {
    if (active_ && input && interval_ > 0.0f && Finite(*input))
      Record(*input);
    active_ = false;
    enable_ = false;
    done_ = true;
    status_ = status;
    capture_.header.status = status;
    tau_cmd_ = 0.0f;
    prbs_ = 0.0f;
    pd_ = 0.0f;
    return 0.0f;
  }
  bool active_ = false, previous_saturated_ = false;
  uint16_t lfsr_ = 0;
  uint32_t previous_stage_ = 0, flags_ = 0;
  float time_ = 0.0f, interval_ = 0.0f, initial_pitch_ = 0.0f;
  float next_bit_time_ = 0.0f, sign_ = 1.0f;
  float previous_tau_ = 0.0f, previous_prbs_ = 0.0f, previous_pd_ = 0.0f;
  float cmd_integral_ = 0.0f, feedback_integral_ = 0.0f;
  float excitation_integral_ = 0.0f, pd_integral_ = 0.0f;
};
static_assert(sizeof(Experiment::Header) == 128);
static_assert(sizeof(Experiment::Sample) == 48);
static_assert(sizeof(Experiment::Capture) == 0x12080);
static_assert(offsetof(Experiment::Capture, samples) == 128);

} // namespace SysId
