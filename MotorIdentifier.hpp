#pragma once

// clang-format off
/* === MODULE MANIFEST V2 ===
module_description: 电机闭环系统辨识采集 / Motor closed-loop identification capture
standalone: false
depends:
- id: QDU-Robomaster/Motor
  ref: same-or-dev
- id: QDU-Robomaster/RMMotor
  ref: same-or-dev
- id: QDU-Robomaster/DMMotor
  ref: same-or-dev
- id: xrobot-org/BMI088
  ref: same-or-dev
- id: xrobot-org/MadgwickAHRS
  ref: same-or-dev
=== END MANIFEST === */
// clang-format on

#include "DMMotor.hpp"
#include "ImuFeedback.hpp"
#include "Motor.hpp"
#include "MotorSystemID.hpp"
#include "RMMotor.hpp"
#include "can.hpp"
#include "timebase.hpp"

namespace SysId {

// Requires BMI088 gyro + AHRS pitch topics, no Gimbal/CMD dependency or thread.
// Gyro is used for PRBS PD damping and J/b fitting; encoder position is used
// for centering/limits. Keep base/big yaw fixed and pitch close to calibrated
// zero: a gyro body z component is NOT Euler yaw rate at arbitrary pitch.
// Call Poll()
// before the normal motor controller; true means this object owns that cycle.
class MotorIdentifier : public MotorSystem {
 public:
  MotorIdentifier() : MotorSystem(Defaults()) {}
  MotorIdentifier(Motor& motor, LibXR::CAN& bus, uint32_t feedback_id,
                  float feedback_torque_sign = 1.0f)
      : MotorIdentifier() {
    Bind(motor, bus, feedback_id, feedback_torque_sign);
  }
  // Bind once. CAN is only observed for freshness; the driver decodes feedback.
  bool Bind(Motor& motor, LibXR::CAN& bus, uint32_t feedback_id,
            float feedback_torque_sign = 1.0f) {
    if (motor_ || feedback_id > 0x7FF ||
        (feedback_torque_sign != 1 && feedback_torque_sign != -1))
      return false;
    motor_ = &motor;
    feedback_torque_sign_ = feedback_torque_sign;
    auto cb = LibXR::CAN::Callback::Create(
        [](bool in_isr, MotorIdentifier* self,
           const LibXR::CAN::ClassicPack& pack) {
          (void)in_isr;
          if (pack.dlc != 8) return;
          self->last_rx_us_ =
              static_cast<uint32_t>(LibXR::Timebase::GetMicroseconds());
        },
        this);
    bus.Register(cb, LibXR::CAN::Type::STANDARD,
                 LibXR::CAN::FilterMode::ID_RANGE, feedback_id, feedback_id);
    return true;
  }
  // 0 stop/idle, 1 PRBS+PD, 2 zero-torque manual calibration.
  volatile uint32_t command_ = 0;
  volatile float min_position_ = -1000000.0f;
  volatile float max_position_ = 1000000.0f;
  bool Busy() const { return active_ || command_ != 0; }
  // Axis/sign refer to the BMI088 topic AFTER its configured rotation.
  // Select the sign so that positive encoder motion gives positive gyro speed.
  // pitch_zero is the AHRS pitch reading when the yaw and gyro axes align.
  bool BindImu(const char* gyro_topic, const char* euler_topic,
               uint32_t axis = 2, float sign = 1.0f, float pitch_zero = 0.0f) {
    return imu_.Bind(gyro_topic, euler_topic, axis, sign, pitch_zero);
  }
  volatile float omega_imu_ = 0, pitch_imu_ = 0;

  bool Poll() {
    const uint64_t NOW = LibXR::Timebase::GetMicroseconds();
    float dt = last_poll_us_ == 0
                   ? .002f
                   : static_cast<float>(NOW - last_poll_us_) * 1e-6f;
    last_poll_us_ = NOW;
    imu_.Update(NOW);
    omega_imu_ = imu_.omega_;
    pitch_imu_ = imu_.pitch_;
    const uint32_t COMMAND = command_;
    if (!active_ && COMMAND == 0) return false;
    if (!motor_) {
      Cancel(Status::BAD_INPUT);
      command_ = 0;
      return true;
    }
    if (!active_) {
      Reset();
      dt = .002f;  // An idle debugger pause is not a sampling interval.
      if (COMMAND != 1 && COMMAND != 2) return Finish(Status::INVALID_COMMAND);
      test_mode_ = COMMAND == 1 ? TestMode::PRBS_PD : TestMode::CALIBRATION;
      mode_command_ = COMMAND;
      active_ = true;
      pitch_started_ = false;
      start_us_ = NOW;
      last_enable_us_ = 0;
      min_latched_ = min_position_;
      max_latched_ = max_position_;
      if (!std::isfinite(min_latched_) || !std::isfinite(max_latched_) ||
          min_latched_ >= max_latched_)
        return Finish(Status::BAD_INPUT);
    }
    if (COMMAND == 0) return Finish(Status::WAITING);
    if (COMMAND != mode_command_) return Finish(Status::INVALID_COMMAND);
    motor_->Update();
    const auto& FB = motor_->GetFeedback();
    omega_motor_ = FB.omega;
    if (COMMAND == 1) {
      // IMU is mandatory: do not silently fall back to motor speed.
      if (!imu_.Fresh(NOW)) return Finish(Status::IMU_OFFLINE);
      if (!std::isfinite(imu_.omega_) || !std::isfinite(imu_.pitch_))
        return Finish(Status::BAD_INPUT);
      if (!pitch_started_) {
        initial_pitch_ = imu_.pitch_;
        pitch_started_ = true;
      }
      if (std::fabs(imu_.pitch_) > .05f ||
          std::fabs(imu_.pitch_ - initial_pitch_) > .05f)
        return Finish(Status::PITCH_MOVED);
      if (std::fabs(imu_.omega_) > 3.0f || !std::isfinite(FB.omega) ||
          std::fabs(FB.omega) > 3.0f)
        return Finish(Status::OVERSPEED);
    }
    if (!std::isfinite(FB.position) || FB.position <= min_latched_ ||
        FB.position >= max_latched_)
      return Finish(Status::POSITION_JUMP);
    if (!std::isfinite(FB.temp) || FB.temp >= 75)
      return Finish(Status::OVER_TEMPERATURE);
    units_confirmed_ =
        true;  // Driver mapping is configured by the application.
    const uint32_t STAMP = last_rx_us_;
    const bool FRESH =
        STAMP != 0 &&
        static_cast<uint32_t>(static_cast<uint32_t>(NOW) - STAMP) <= 50000U;
    if (status_ == Status::RUNNING &&
        (!FRESH || (COMMAND == 1 && FB.state != 1)))
      return Finish(Status::FEEDBACK_LOST);
    if (COMMAND == 1 && FB.state == 0) {
      if (FRESH && (last_enable_us_ == 0 || NOW - last_enable_us_ >= 200000)) {
        motor_->Enable();
        last_enable_us_ = NOW;
      }
      if (NOW - start_us_ > 5000000) return Finish(Status::FEEDBACK_LOST);
      return true;
    }
    const float OUTPUT =
        Step(FB.position, COMMAND == 1 ? imu_.omega_ : FB.omega,
             FB.torque * feedback_torque_sign_, dt,
             FRESH && (COMMAND == 2 || FB.state == 1));
    if (done_) {
      active_ = false;
      command_ = 0;
      motor_->Relax();
      return true;
    }
    motor_->Control(
        Motor::MotorCmd{.mode = Motor::MODE_TORQUE, .torque = OUTPUT});
    return true;
  }

 private:
  static Param Defaults() {
    Param p;
    p.torque_limit = .2f;
    p.speed_limit = 3.0f;
    return p;
  }
  bool Finish(Status reason) {
    Cancel(reason);
    command_ = 0;
    active_ = false;
    if (motor_) motor_->Relax();
    return true;
  }
  Motor* motor_ = nullptr;
  ImuFeedback imu_;
  bool pitch_started_ = false;
  float initial_pitch_ = 0;
  volatile uint32_t last_rx_us_ = 0;
  float feedback_torque_sign_ = 1.0f;
  bool active_ = false;
  uint32_t mode_command_ = 0;
  uint64_t last_poll_us_ = 0, start_us_ = 0, last_enable_us_ = 0;
  float min_latched_ = 0, max_latched_ = 0;
};
}  // namespace SysId
