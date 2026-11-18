#pragma once
#include <cstdint>
namespace LibXR {
using MicrosecondTimestamp = uint64_t;
struct Timebase {
  static inline uint64_t now = 1000;
  static uint64_t GetMicroseconds() { return now; }
};
} // namespace LibXR
