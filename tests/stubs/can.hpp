#pragma once
#include "Motor.hpp"
#include <cstdint>
#include <functional>
#include <vector>
namespace LibXR {
struct CAN {
  enum class Type : uint8_t {
    STANDARD,
    EXTENDED,
    REMOTE_STANDARD,
    REMOTE_EXTENDED,
    ERROR,
    TYPE_NUM
  };
  enum class FilterMode { ID_RANGE, ID_MASK };
  struct ClassicPack {
    uint32_t id = 0x205;
    Type type = Type::STANDARD;
    uint8_t dlc = 8;
    uint8_t data[8]{};
  };
  struct Configuration {};
  struct ErrorState {};
  struct Callback {
    std::function<void(bool, const ClassicPack &)> fn;
    template <class F, class T> static Callback Create(F f, T *p) {
      return Callback{[=](bool isr, const ClassicPack &v) { f(isr, p, v); }};
    }
  };
  struct Filter {
    Callback cb;
    Type type;
    FilterMode mode;
    uint32_t a, b;
  };
  std::vector<Filter> filters;
  ErrorCode queue_result = ErrorCode::OK;
  ClassicPack last_tx{};
  uint32_t tx_count = 0;
  virtual ~CAN() = default;
  virtual ErrorCode SetConfig(const Configuration &) { return ErrorCode::OK; }
  virtual uint32_t GetClockFreq() const { return 42000000; }
  virtual ErrorCode GetErrorState(ErrorState &) const { return ErrorCode::OK; }
  virtual ErrorCode AddMessage(const ClassicPack &p) {
    last_tx = p;
    ++tx_count;
    return queue_result;
  }
  void Register(Callback cb, Type t, FilterMode mode, uint32_t a, uint32_t b) {
    filters.push_back({cb, t, mode, a, b});
  }
  void Receive(uint8_t dlc = 8) {
    ClassicPack p;
    p.dlc = dlc;
    OnMessage(p, true);
  }
  void Receive(const ClassicPack &p) { OnMessage(p, true); }

protected:
  void OnMessage(const ClassicPack &p, bool isr) {
    for (auto &f : filters)
      if (f.type == p.type &&
          (f.mode == FilterMode::ID_RANGE ? p.id >= f.a && p.id <= f.b
                                          : (p.id & f.a) == f.b))
        f.cb.fn(isr, p);
  }
};
} // namespace LibXR
