#ifndef CURVATURE_FUNCTOR_H
#define CURVATURE_FUNCTOR_H

#include "base_functor.h"

class CurvatureFunctor : public BaseFunctor {
 public:
  CurvatureFunctor(const Gate &previous_gate, const Gate &gate, const Gate &next_gate, double weight = 1)
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
  bool operator()(const T &previous_gate_parameter,
                  const T &gate_parameter,
                  const T &next_gate_parameter,
                  T &residual) const {
    const auto a = getValueAtParameter(previous_gate_, previous_gate_parameter);
    const auto b = getValueAtParameter(gate_, gate_parameter);
    const auto c = getValueAtParameter(next_gate_, next_gate_parameter);

    //    b--c
    //   /
    //  a
    const auto a_to_b = b - a;
    const auto b_to_c = c - b;
    const auto a_to_c = a - c;

    const auto cross2 = a_to_b.x() * b_to_c.y() - a_to_b.y() * b_to_c.x();
    const auto curvature =
        2.0 * cross2 /
        sqrt(a_to_b.squaredNorm() * b_to_c.squaredNorm() * a_to_c.squaredNorm());
    residual = weight_ * curvature;
    return true;
  }

 protected:
  const Gate &previous_gate_;
  const Gate &next_gate_;
};

#endif  // CURVATURE_FUNCTOR_H
