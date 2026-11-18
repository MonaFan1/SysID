#pragma once
#include "Motor.hpp"
#include <deque>
namespace LibXR {
template <class T> class SPSCQueue {
public:
  explicit SPSCQueue(unsigned n) : capacity_(n) {}
  ErrorCode Push(const T &value) {
    if (data_.size() >= capacity_)
      return ErrorCode::FAILED;
    data_.push_back(value);
    return ErrorCode::OK;
  }
  ErrorCode Pop(T &value) {
    if (data_.empty())
      return ErrorCode::FAILED;
    value = data_.front();
    data_.pop_front();
    return ErrorCode::OK;
  }

private:
  std::deque<T> data_;
  unsigned capacity_;
};
} // namespace LibXR
