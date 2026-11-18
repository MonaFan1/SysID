#pragma once

#include <cmath>

#include "message.hpp"
#include "timebase.hpp"
#include "transform.hpp"

namespace SysId {
// BMI088 publishes angular velocity in rad/s after its configured rotation.
// MadgwickAHRS publishes Euler angles in rad. Topics must already exist at
// Bind. Subscribe independently of Gimbal: this module never depends on Gimbal
// state.
class ImuFeedback {
 public:
  bool Bind(const char* gyro_topic, const char* euler_topic, uint32_t axis = 2,
            float sign = 1.0f, float pitch_zero = 0.0f) {
    if (gyro_ || !gyro_topic || !euler_topic || axis > 2 ||
        (sign != 1 && sign != -1) || !std::isfinite(pitch_zero))
      return false;
    axis_ = axis;
    sign_ = sign;
    pitch_zero_ = pitch_zero;
    gyro_ = new GyroSubscriber(gyro_topic);
    euler_ = new EulerSubscriber(euler_topic);
    gyro_->StartWaiting();
    euler_->StartWaiting();
    return true;
  }
  void Update(uint64_t now) {
    if (!gyro_) return;
    if (gyro_->Available()) {
      omega_ = sign_ * gyro_->GetData()(axis_);
      gyro_stamp_ = now;
      gyro_->StartWaiting();
    }
    if (euler_->Available()) {
      pitch_ = euler_->GetData().Pitch() - pitch_zero_;
      euler_stamp_ = now;
      euler_->StartWaiting();
    }
  }
  bool Fresh(uint64_t now) const {
    // Receipt times at Poll(), not a hardware sensor timestamp. Never refresh
    // the stamp merely because the last value is reused by the control loop.
    return gyro_stamp_ && euler_stamp_ && now - gyro_stamp_ <= 20000 &&
           now - euler_stamp_ <= 50000;
  }
  float omega_ = 0, pitch_ = 0;

 private:
  using GyroSubscriber =
      LibXR::Topic::ASyncSubscriber<Eigen::Matrix<float, 3, 1>>;
  using EulerSubscriber =
      LibXR::Topic::ASyncSubscriber<LibXR::EulerAngle<float>>;
  GyroSubscriber* gyro_ = nullptr;
  EulerSubscriber* euler_ = nullptr;
  uint32_t axis_ = 2;
  float sign_ = 1, pitch_zero_ = 0;
  uint64_t gyro_stamp_ = 0, euler_stamp_ = 0;
};
}  // namespace SysId
