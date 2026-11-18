#pragma once

#include <cmath>

#include "message.hpp"
#include "queue/spsc_queue.hpp"
#include "timebase.hpp"
#include "transform.hpp"

namespace SysId {
// BMI088 publishes angular velocity in rad/s after its configured rotation.
// MadgwickAHRS publishes Euler angles in rad. Topics must already exist at
// Bind. Subscribe independently of Gimbal: this module never depends on Gimbal
// state.
class ImuFeedback {
public:
  bool Bind(const char *gyro_topic, const char *euler_topic, uint32_t axis = 2,
            float sign = 1.0f, float pitch_zero = 0.0f) {
    if (gyro_ || !gyro_topic || !euler_topic || axis > 2 ||
        (sign != 1 && sign != -1) || !std::isfinite(pitch_zero))
      return false;
    axis_ = axis;
    sign_ = sign;
    pitch_zero_ = pitch_zero;
    gyro_ = new LibXR::Topic::QueuedSubscriber(gyro_topic, gyro_queue_);
    euler_ = new EulerSubscriber(euler_topic);
    euler_->StartWaiting();
    return true;
  }
  void Update(uint64_t now) {
    gyro_new_ = false;
    if (!gyro_)
      return;
    GyroMessage sample;
    float block_area = 0, block_span = 0;
    for (uint32_t n = 0;
         n < 32 && gyro_queue_.Pop(sample) == LibXR::ErrorCode::OK; ++n) {
      const uint64_t TS = sample.timestamp;
      const float RAW = sign_ * sample.data(axis_);
      gyro_y_ = sample.data(1);
      if (gyro_stamp_ && TS > gyro_stamp_) {
        const float DT = static_cast<float>(TS - gyro_stamp_) * 1e-6f;
        const float AREA = .5f * (RAW + last_raw_) * DT;
        gyro_area_ += AREA;
        block_area += AREA;
        block_span += DT;
      }
      last_raw_ = RAW;
      omega_raw_ = RAW;
      gyro_stamp_ = TS;
      gyro_new_ = true;
    }
    if (gyro_new_)
      omega_ = block_span > 0 ? block_area / block_span : last_raw_;
    if (euler_->Available()) {
      pitch_ = euler_->GetData().Pitch() - pitch_zero_;
      euler_stamp_ = now;
      euler_->StartWaiting();
    }
  }
  bool Fresh(uint64_t now) const {
    // Gyro freshness uses the publisher hardware timestamp; Euler uses receipt
    // time. Reusing a value never refreshes the timestamp.
    return gyro_stamp_ && euler_stamp_ && now - gyro_stamp_ <= 20000 &&
           now - euler_stamp_ <= 50000;
  }
  float omega_ = 0, pitch_ = 0, gyro_y_ = 0;
  float omega_raw_ = 0, gyro_area_ = 0;
  uint64_t GyroTimestamp() const { return gyro_stamp_; }
  bool gyro_new_ = false;

private:
  using GyroMessage = LibXR::Topic::Message<Eigen::Matrix<float, 3, 1>>;
  using EulerSubscriber =
      LibXR::Topic::ASyncSubscriber<LibXR::EulerAngle<float>>;
  LibXR::SPSCQueue<GyroMessage> gyro_queue_{32};
  LibXR::Topic::QueuedSubscriber *gyro_ = nullptr;
  float last_raw_ = 0;
  EulerSubscriber *euler_ = nullptr;
  uint32_t axis_ = 2;
  float sign_ = 1, pitch_zero_ = 0;
  uint64_t gyro_stamp_ = 0, euler_stamp_ = 0;
};
} // namespace SysId
