#include "ImuFeedback.hpp"
#include <cassert>
#include <cmath>
#include <iostream>
using Gyro = LibXR::Topic::ASyncSubscriber<Eigen::Matrix<float, 3, 1>>;
using Euler = LibXR::Topic::ASyncSubscriber<LibXR::EulerAngle<float>>;
int main() {
  SysId::ImuFeedback imu;
  assert(imu.Bind("gyro", "euler"));
  const uint64_t START = LibXR::Timebase::now;
  for (int n = 0; n <= 40; ++n) {
    LibXR::Timebase::now = START + n * 500;
    Eigen::Matrix<float, 3, 1> g{};
    g(2) = 1.0f + (n % 2 ? -10.0f : 10.0f);
    Gyro::Publish(g);
    Euler::Publish({});
    if (n % 4 == 0)
      imu.Update(LibXR::Timebase::now);
  }
  assert(std::fabs(imu.gyro_area_ - .020f) < 1e-6f);
  assert(std::fabs(imu.omega_ - 1.0f) < 1e-5f);
  assert(imu.omega_raw_ == 11.0f);
  assert(imu.Fresh(LibXR::Timebase::now));
  std::cout << "PASS: all 2 kHz samples retained; alternating noise cancels "
               "before decimation.\n";
}
