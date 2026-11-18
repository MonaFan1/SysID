#include <cassert>
#include <cmath>
#include <fstream>
#include <iostream>
#include <limits>
#include <string>

#include "MotorSystemID.hpp"

static constexpr float TWO_PI = 6.2831853071795864769f;
static constexpr double PI = 3.14159265358979323846;

float QuantizePosition(float angle) {
  const float WRAPPED = std::remainder(angle, TWO_PI);
  const int CODE = static_cast<int>((WRAPPED + TWO_PI) / (2 * TWO_PI) * 65535);
  return CODE * (2 * TWO_PI) / 65535 - TWO_PI;
}
void Dump(const SysId::MotorSystem& s, const std::string& path) {
  std::ofstream out(path, std::ios::binary);
  out.write(reinterpret_cast<const char*>(s.buf_), sizeof(s.buf_));
  const uint32_t COUNT = s.count_, INDEX = s.write_idx_;
  out.write(reinterpret_cast<const char*>(&INDEX), 4);
  out.write(reinterpret_cast<const char*>(&COUNT), 4);
  assert(out.good());
}
void CheckFreeze(SysId::MotorSystem& s) {
  const uint32_t COUNT = s.count_;
  const float TIME = s.buf_[COUNT - 1].time;
  for (int i = 0; i < 1000; ++i) assert(s.Step(0, 0, 0, 0.002f, false) == 0);
  assert(s.count_ == COUNT && s.buf_[COUNT - 1].time == TIME);
  assert(COUNT < 1536 && s.write_idx_ == COUNT);
}
void Calibration(float position_scale, const std::string& path,
                 float velocity_scale = 1.0f) {
  SysId::MotorSystem s;
  double t = 0;
  for (unsigned step = 0; !s.done_ && step < 20000; ++step) {
    const float DT = step % 3 == 0 ? 0.0027f : 0.0019f;
    t += DT;
    const double PHASE = std::clamp((t - 3.0) / 20.0, 0.0, 1.0);
    const double ANGLE = PI * (1 - std::cos(PI * PHASE));
    const double SPEED =
        (t > 3 && t < 23) ? PI * PI / 20 * std::sin(PI * PHASE) : 0;
    const float TORQUE = s.Step(QuantizePosition(3.1f + position_scale * ANGLE),
                                velocity_scale * SPEED + 0.007f, 0, DT, true);
    assert(TORQUE == 0);  // Includes moving encoder and both wrap crossings.
    if (step == 4000) {
      s.test_mode_ = SysId::MotorSystem::TestMode::LONG_SPEED;
      s.units_confirmed_ = true;
    }
  }
  assert(s.status_ == SysId::MotorSystem::Status::DONE);
  assert(s.active_mode_ == SysId::MotorSystem::TestMode::CALIBRATION);
  assert(s.count_ > 1300);
  for (uint32_t i = 0; i < s.count_; ++i) {
    assert(s.buf_[i].stage == 100 && s.buf_[i].tau_cmd_mean == 0);
    assert(s.buf_[i].omega_ref == 0);
  }
  assert(std::fabs(s.theta_ - TWO_PI * position_scale) < 0.001);
  Dump(s, path);
  CheckFreeze(s);
}
void LongSpeed(const std::string& path) {
  SysId::MotorSystem::Param param;
  param.mode = SysId::MotorSystem::TestMode::LONG_SPEED;
  SysId::MotorSystem s(param);
  assert(s.Step(0, 0, 0, 0.002f, true) == 0);
  assert(s.status_ == SysId::MotorSystem::Status::UNITS_UNCONFIRMED);
  assert(s.count_ == 0);
  s.units_confirmed_ = true;
  s.Reset();
  assert(s.test_mode_ == SysId::MotorSystem::TestMode::LONG_SPEED);
  double angle = 3.1, speed = 0;
  float torque = 0;
  bool stages[9]{};
  for (unsigned step = 0; !s.done_ && step < 35000; ++step) {
    const float DT = step % 3 == 0 ? 0.0027f : 0.0019f;
    for (int j = 0; j < 20; ++j) {
      const double H = DT / 20;
      const double DRIVE = torque - 0.03 - 1.2 * speed;
      const double FRICTION = std::fabs(speed) < 0.0005
                                  ? std::clamp(DRIVE, -0.2, 0.2)
                                  : (speed > 0 ? 0.2 : -0.2);
      const double NEXT = speed + (DRIVE - FRICTION) / 0.12 * H;
      speed = speed * NEXT < 0 && std::fabs(DRIVE) < 0.2 ? 0 : NEXT;
      angle += speed * H;
    }
    torque = s.Step(QuantizePosition(angle), speed + 0.007f, torque, DT, true);
    assert(std::fabs(torque) <= 2);
    if (s.stage_ >= 200 && s.stage_ <= 208) stages[s.stage_ - 200] = true;
    if (s.stage_ == 201 && s.buf_[s.count_ - 1].time > 5) assert(torque > 0);
    if (s.stage_ == 203 && s.buf_[s.count_ - 1].time > 20) assert(torque < 0);
  }
  assert(s.status_ == SysId::MotorSystem::Status::DONE);
  assert(s.buf_[s.count_ - 1].time >= 59);
  for (bool present : stages) assert(present);
  assert(std::fabs(speed) < 0.03);
  Dump(s, path);
  CheckFreeze(s);
}
void Prbs(const std::string& path) {
  SysId::MotorSystem::Param p;
  p.mode = SysId::MotorSystem::TestMode::PRBS_PD;
  p.speed_limit = 5;
  SysId::MotorSystem s(p);
  s.units_confirmed_ = true;
  s.amplitude_ = .3f;
  s.kp_ = 4;
  s.kd_ = .12f;
  s.max_travel_ = .3f;
  s.torque_limit_ = .5f;
  double q = 0, w = 0;
  float u = 0;
  bool exciting = false, settling = false;
  for (int n = 0; n < 7500 && !s.done_; ++n) {
    for (int j = 0; j < 20; ++j) {
      w += (u - .09 * w) / .012 * .0001;
      q += w * .0001;
    }
    s.omega_motor_ = w;
    u = s.Step(q, w, u, .002f, true);
    assert(std::fabs(u) <= .5f);
    exciting |= s.stage_ == 401;
    settling |= s.stage_ == 403;
    if (n == 1000) {
      s.amplitude_ = 10;
      s.duration_s_ = 3;
      s.kp_ = 100;
    }
  }
  assert(s.status_ == SysId::MotorSystem::Status::DONE);
  assert(exciting && settling && s.count_ > 1100);
  assert(s.buf_[s.count_ - 1].time >= 14);
  Dump(s, path);
  CheckFreeze(s);
  s.Reset();
  s.amplitude_ = .1f;
  s.duration_s_ = 14;
  s.kp_ = 2;
  s.max_travel_ = .01f;
  s.Step(0, 0, 0, .002f, true);
  s.Step(.015f, 0, 0, .002f, true);
  assert(s.status_ == SysId::MotorSystem::Status::TRAVEL_LIMIT);
  s.Reset();
  s.seed_ = 0;
  s.Step(0, 0, 0, .002f, true);
  assert(s.status_ == SysId::MotorSystem::Status::BAD_INPUT);
}
int main(int argc, char** argv) {
  const std::string DIR = argc > 1 ? argv[1] : ".";
  Calibration(1.0f, DIR + "/calibration_ok.bin");
  Calibration(0.5f, DIR + "/calibration_half_angle.bin");
  Calibration(0.5f, DIR + "/calibration_both_half.bin", 0.5f);
  LongSpeed(DIR + "/long_speed.bin");
  Prbs(DIR + "/prbs.bin");
  SysId::MotorSystem s;
  s.Step(0, 0, 0, 0.002f, true);
  s.Step(0, 0, 0, 0.002f, false);
  assert(s.status_ == SysId::MotorSystem::Status::FEEDBACK_LOST);
  s.Reset();
  s.Step(0, 0, 0, 0.002f, true);
  s.Step(1, 0, 0, 0.002f, true);
  assert(s.status_ == SysId::MotorSystem::Status::POSITION_JUMP);
  s.Reset();
  s.Step(0, 0, 0, std::numeric_limits<float>::quiet_NaN(), true);
  assert(s.status_ == SysId::MotorSystem::Status::BAD_INPUT);
  s.Reset();
  s.Step(0, 0, 0, 0.002f, true);
  for (int i = 1; i < 20 && !s.done_; ++i)
    s.Step(i * 0.006f, 0, 0, 0.002f, true);
  assert(s.status_ == SysId::MotorSystem::Status::OVERSPEED);
  SysId::MotorSystem::Param invalid;
  invalid.calibration_log_period = 0.001f;
  SysId::MotorSystem full(invalid);
  full.Step(0, 0, 0, 0.002f, true);
  assert(full.status_ == SysId::MotorSystem::Status::BAD_INPUT);
  std::cout
      << "PASS: passive default, marked rotation, wrong-scale fixture, "
         "unit gate, latched mode, long +/- repeats, freeze, fault checks.\n";
}
