#include "Experiment.hpp"
#include "reference/SmallYawSysId.hpp"
#include <cassert>
#include <cmath>
#include <cstring>
#include <fstream>
#include <iostream>
static SmallYawSysId original;
static SysId::Experiment restored;
int main(int argc, char **argv) {
  restored.min_position_ = -2.18f;
  restored.max_position_ = -1.19f;
  restored.command_quantum_ = SmallYawSysId::COMMAND_QUANTUM;
  restored.torque_constant_ = SmallYawSysId::TORQUE_CONSTANT;
  original.enable_ = true;
  restored.enable_ = true;
  float t = 0, previous = 0;
  for (int i = 0; i < 9000 && !original.done_; ++i) {
    const float DT = .002f + static_cast<float>((i % 3) - 1) * .0001f;
    t += DT;
    const float Q = -2.08f + .01f * std::sin(5 * t) + .002f * std::sin(20 * t);
    const float W = .05f * std::cos(5 * t) + .04f * std::cos(20 * t);
    const SmallYawSysId::Input A{DT, Q,  W,    W,    previous,   0,
                                 0,  30, true, true, i % 7 != 0, i % 3 != 0};
    const SysId::Experiment::Input B{
        DT, Q, W, W, previous, 0, 0, 30, true, true, i % 7 != 0, i % 3 != 0};
    const float U = original.Step(A), V = restored.Step(B);
    assert(U == V && original.count_ == restored.count_ &&
           original.stage_ == restored.stage_);
    const int16_t RAW = static_cast<int16_t>(-U / .741f / 3.0f * 16384.0f);
    previous = static_cast<float>(RAW) * -1.0f * .741f * 3.0f / 16384.0f;
    original.CommandQueued(previous);
    restored.CommandQueued(previous);
  }
  assert(original.status_ == SmallYawSysId::DONE &&
         restored.status_ == SysId::Experiment::DONE);
  assert(restored.time_s_ >= 15 && restored.count_ > 1200);
  assert(std::memcmp(original.capture_.samples, restored.capture_.samples,
                     original.count_ * sizeof(SmallYawSysId::Sample)) == 0);
  if (argc > 1) {
    std::ofstream file(std::string(argv[1]) + "/restored.bin",
                       std::ios::binary);
    file.write(reinterpret_cast<const char *>(&restored.capture_),
               sizeof(restored.capture_));
    assert(file.good());
  }
  std::cout << "PASS: all requests, stages and all 48-byte samples match "
               "yesterday's core exactly.\n";
}
