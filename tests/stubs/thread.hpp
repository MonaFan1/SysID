#pragma once
#include <cstdint>
namespace LibXR {
struct Thread {
  enum class Priority { HIGH };
  static inline unsigned created = 0;
  static inline uint32_t stack = 0;
  template <class T>
  void Create(T *, void (*)(T *), const char *, uint32_t size, Priority) {
    ++created;
    stack = size;
  }
  static void Sleep(unsigned) {}
};
} // namespace LibXR
