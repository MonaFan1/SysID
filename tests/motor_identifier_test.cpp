#include <cassert>
#include <iostream>

#include "MotorIdentifier.hpp"
struct FakeMotor : Motor {
  Feedback fb;
  float output = 0;
  int sends = 0, relaxes = 0, enables = 0;
  LibXR::ErrorCode Update() override { return LibXR::ErrorCode::OK; }
  const Feedback& GetFeedback() override { return fb; }
  void Control(const MotorCmd& cmd) override {
    output = cmd.torque;
    ++sends;
  }
  void Enable() override {
    fb.state = 1;
    ++enables;
  }
  void Relax() override {
    output = 0;
    ++relaxes;
  }
};
using Status = SysId::MotorSystem::Status;
using Gyro = LibXR::Topic::ASyncSubscriber<Eigen::Matrix<float, 3, 1>>;
using Euler = LibXR::Topic::ASyncSubscriber<LibXR::EulerAngle<float>>;
void Imu(float z = 0, float pitch = 0) {
  Eigen::Matrix<float, 3, 1> gyro{};
  gyro(2) = z;
  LibXR::EulerAngle<float> angle{};
  angle.pitch = pitch;
  Gyro::Publish(gyro);
  Euler::Publish(angle);
}
int main() {
  FakeMotor motor;
  LibXR::CAN can;
  SysId::MotorIdentifier id(motor, can, 0x15);
  assert(id.BindImu("gyro", "euler", 2, -1));
  assert(!id.Poll() && motor.sends == 0);
  LibXR::Timebase::now += 1000000;  // pause while idle must not abort startup
  id.command_ = 1;
  Imu();
  can.Receive();
  assert(id.Poll());
  assert(motor.enables == 1 && motor.sends == 0);
  LibXR::Timebase::now += 2000;
  Imu();
  can.Receive();
  id.Poll();
  assert(id.status_ == Status::RUNNING);
  for (int i = 0; i < 550; ++i) {
    LibXR::Timebase::now += 2000;
    Imu();
    can.Receive();
    id.Poll();
  }
  assert(motor.sends > 500 && motor.output != 0);
  id.command_ = 0;
  id.Poll();
  assert(motor.output == 0 && motor.relaxes == 1 && !id.Busy());
  const auto count = id.count_;
  assert(!id.Poll() && id.count_ == count);
  id.command_ = 1;
  LibXR::Timebase::now += 2000;
  Imu();
  can.Receive();
  id.Poll();
  LibXR::Timebase::now += 51000;
  Imu();
  can.Receive(2);
  id.Poll();
  assert(id.status_ == Status::FEEDBACK_LOST && motor.output == 0);
  id.command_ = 1;
  LibXR::Timebase::now += 2000;
  Imu();
  can.Receive();
  id.Poll();
  motor.fb.position = .07;
  LibXR::Timebase::now += 2000;
  can.Receive();
  id.Poll();
  assert(id.done_ && motor.output == 0);
  motor.fb.position = 0;
  motor.fb.state = 0;
  id.command_ = 2;
  for (int i = 0; i < 20; ++i) {
    LibXR::Timebase::now += 2000;
    can.Receive();
    id.Poll();
    assert(motor.output == 0 && !id.done_);
  }
  id.command_ = 0;
  id.Poll();
  id.command_ = 9;
  id.Poll();
  assert(id.status_ == Status::INVALID_COMMAND && !id.Busy());
  motor.fb.state = 1;
  Imu(-.4f);
  can.Receive();
  id.command_ = 1;
  id.Poll();
  LibXR::Timebase::now += 2000;
  Imu(-.4f);
  can.Receive();
  id.Poll();
  assert(id.omega_imu_ == .4f && motor.fb.omega == 0);
  assert(motor.output <
         -.04f);  // PD uses signed gyro, not the zero motor speed.
  LibXR::Timebase::now += 21000;
  can.Receive();
  id.Poll();
  assert(id.status_ == Status::IMU_OFFLINE && motor.output == 0);
  Imu(0, .1f);
  can.Receive();
  id.command_ = 1;
  id.Poll();
  assert(id.status_ == Status::PITCH_MOVED && motor.output == 0);
  std::cout << "PASS: idle, one-command enable/output, stop, freeze, "
               "disconnect, travel, passive calibration, invalid command.\n";
}
