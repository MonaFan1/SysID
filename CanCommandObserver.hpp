#pragma once
#include "can.hpp"
#include <atomic>
#include <cmath>

namespace SysId {
// Transparent transport wrapper. Pass Bus(can1) to the unchanged motor driver
// in YAML. Captures its packed frame and actual AddMessage queue result.
// The tested motor must exclusively own its CAN control group during the run.
class CanCommandObserver : public LibXR::CAN {
public:
  enum class Encoding : uint32_t { SIGNED16 = 0, UNSIGNED12 = 1 };
  struct Param {
    uint32_t tx_id = 0;
    uint32_t byte_offset = 0;
    Encoding encoding = Encoding::SIGNED16;
    float torque_scale = 1;
    float torque_bias = 0;
  };
  static CanCommandObserver &Bus(LibXR::CAN &physical) {
    for (auto *p = head_; p; p = p->next_)
      if (&p->physical_ == &physical)
        return *p;
    auto *p = new CanCommandObserver(physical);
    p->next_ = head_;
    head_ = p;
    return *p;
  }
  bool Configure(const Param &p) {
    if (configured_ || p.tx_id > 0x7ff || p.byte_offset > 6 ||
        (p.encoding != Encoding::SIGNED16 &&
         p.encoding != Encoding::UNSIGNED12) ||
        !std::isfinite(p.torque_scale) || p.torque_scale == 0 ||
        !std::isfinite(p.torque_bias))
      return false;
    param_ = p;
    configured_ = true;
    return true;
  }
  LibXR::ErrorCode SetConfig(const Configuration &p) override {
    return physical_.SetConfig(p);
  }
  uint32_t GetClockFreq() const override { return physical_.GetClockFreq(); }
  LibXR::ErrorCode GetErrorState(ErrorState &p) const override {
    return physical_.GetErrorState(p);
  }
  LibXR::ErrorCode AddMessage(const ClassicPack &pack) override {
    const auto RESULT = physical_.AddMessage(pack);
    if (configured_ && pack.type == Type::STANDARD && pack.id == param_.tx_id &&
        pack.dlc == 8) {
      const uint32_t N = param_.byte_offset;
      const uint16_t RAW =
          (static_cast<uint16_t>(pack.data[N]) << 8) | pack.data[N + 1];
      const float CODE = param_.encoding == Encoding::SIGNED16
                             ? static_cast<float>(static_cast<int16_t>(RAW))
                             : static_cast<float>(RAW & 0x0fff);
      torque_.store(CODE * param_.torque_scale + param_.torque_bias);
      queued_.store(RESULT == LibXR::ErrorCode::OK);
      count_.fetch_add(1);
    }
    return RESULT;
  }
  uint32_t Count() const { return count_.load(); }
  float Torque() const { return torque_.load(); }
  bool Queued() const { return queued_.load(); }

private:
  explicit CanCommandObserver(LibXR::CAN &physical) : physical_(physical) {
    auto cb =
        Callback::Create([](bool isr, CanCommandObserver *self,
                            const ClassicPack &p) { self->OnMessage(p, isr); },
                         this);
    for (uint32_t i = 0; i < static_cast<uint32_t>(Type::TYPE_NUM); ++i)
      physical.Register(cb, static_cast<Type>(i), FilterMode::ID_MASK, 0, 0);
  }
  LibXR::CAN &physical_;
  Param param_;
  bool configured_ = false;
  std::atomic<uint32_t> count_{0};
  std::atomic<float> torque_{0};
  std::atomic<bool> queued_{false};
  CanCommandObserver *next_ = nullptr;
  static inline CanCommandObserver *head_ = nullptr;
};
} // namespace SysId
