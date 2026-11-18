#pragma once
#include <cstdint>
namespace LibXR {
struct Topic {
  template <class T>
  struct ASyncSubscriber {
    static inline T value{};
    static inline uint64_t sequence = 0;
    uint64_t seen = 0;
    explicit ASyncSubscriber(const char*) {}
    void StartWaiting() { seen = sequence; }
    bool Available() { return sequence != seen; }
    T GetData() { return value; }
    static void Publish(T data) {
      value = data;
      ++sequence;
    }
  };
};
}  // namespace LibXR
