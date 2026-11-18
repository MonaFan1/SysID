#pragma once
#include <cstdint>
#include <functional>
namespace LibXR {
struct CAN {
  enum class Type { STANDARD };
  enum class FilterMode { ID_RANGE };
  struct ClassicPack {
    uint8_t dlc = 8;
  };
  struct Callback {
    std::function<void(const ClassicPack&)> fn;
    template <class F, class T>
    static Callback Create(F f, T* self) {
      return Callback{[=](const ClassicPack& p) { f(false, self, p); }};
    }
  };
  Callback callback;
  void Register(Callback cb, Type, FilterMode, uint32_t, uint32_t) {
    callback = cb;
  }
  void Receive(uint8_t dlc = 8) { callback.fn(ClassicPack{dlc}); }
};
}  // namespace LibXR
