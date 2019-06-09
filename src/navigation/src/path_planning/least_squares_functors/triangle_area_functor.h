#ifndef TRIANGLE_AREA_FUNCTOR_H
#define TRIANGLE_AREA_FUNCTOR_H

#include "base_functor.h"

class TriangleAreaFunctor : public BaseFunctor {
 public:
  TriangleAreaFunctor(const Gate& previous_gate, const Gate& gate, const Gate& next_gate, double weight = 1)
      : BaseFunctor(gate, weight), previous_gate_(previous_gate), next_gate_(next_gate) {}

  template <typename T>
  bool operator()(const T previous_gate_parameter[1],
                  const T gate_parameter[1],
                  const T next_gate_parameter[1],
                  T residual[1]) const {
    return (*this)(
        previous_gate_parameter[0], gate_parameter[0], next_gate_parameter[0], residual[0]);
  }

  template <typename T>
  bool operator()(const T& previous_gate_parameter,
                  const T& gate_parameter,
                  const T& next_gate_parameter,
                  T& residual) const {
    const auto a = getValueAtParameter(previous_gate_, previous_gate_parameter);
    const auto b = getValueAtParameter(gate_, gate_parameter);
    const auto c = getValueAtParameter(next_gate_, next_gate_parameter);

    //    b--c
    //   /
    //  a
    const auto a_to_b = b - a;
    const auto b_to_c = c - b;

    // Use the area between each three points as cost function.
    // This is the Menger curvature
    // (https://en.wikipedia.org/wiki/Menger_curvature) without the division
    // with sqrt(a_to_b.squaredNorm() * b_to_c.squaredNorm() *
    // a_to_c.squaredNorm())

    const auto cross2 = a_to_b.x() * b_to_c.y() - a_to_b.y() * b_to_c.x();
    const auto curvature = cross2;

    const double scale = 1000.0;  // scaling to have the parameter near to 1.0
    residual = weight_ * scale * curvature;
    return true;
  }

 protected:
  const Gate& previous_gate_;
  const Gate& next_gate_;
};

#endif  // TRIANGLE_AREA_FUNCTOR_H
