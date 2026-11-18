#pragma once
#include <cstdint>
namespace LibXR {
enum class ErrorCode { OK };
}
class Motor {
 public:
  enum ControlMode { MODE_TORQUE };
  struct MotorCmd {
    ControlMode mode;
    float torque = 0;
  };
  struct Feedback {
    uint8_t state = 0;
    float position = 0, omega = 0, torque = 0, temp = 20;
  };
  virtual ~Motor() = default;
  virtual LibXR::ErrorCode Update() = 0;
  virtual const Feedback& GetFeedback() = 0;
  virtual void Control(const MotorCmd&) = 0;
  virtual void Enable() = 0;
  virtual void Relax() = 0;
};
