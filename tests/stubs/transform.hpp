#pragma once
namespace Eigen {
template <class T, int R, int C>
struct Matrix {
  T data[R * C]{};
  T& operator()(unsigned i) { return data[i]; }
  T operator()(unsigned i) const { return data[i]; }
};
}  // namespace Eigen
namespace LibXR {
template <class T>
struct EulerAngle {
  T pitch = 0;
  T Pitch() const { return pitch; }
};
}  // namespace LibXR
