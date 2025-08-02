#pragma once

#include "agilib/math/types.hpp"

namespace agi {

struct Point {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Point() = default;
  Point(const Point&) = default;
  ~Point() = default;

  static constexpr int SIZE = 7;

  bool valid() const {
    return std::isfinite(t) && position.allFinite();
  }
  Scalar t{NAN};
  Vector<3> position{NAN, NAN, NAN};
};


}  // namespace agi
