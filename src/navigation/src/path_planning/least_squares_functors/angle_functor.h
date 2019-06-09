#ifndef ANGLE_FUNCTOR_H
#define ANGLE_FUNCTOR_H

#include "base_functor.h"

class AngleFunctor : public BaseFunctor {

 public:
  AngleFunctor(const Gate &previous_gate, const Gate &gate, const Gate &next_gate, double weight = 1)
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

    const auto previous_point = getValueAtParameter(previous_gate_, previous_gate_parameter);
    const auto point = getValueAtParameter(gate_, gate_parameter);
    const auto next_point = getValueAtParameter(next_gate_, next_gate_parameter);

    const auto to_previous = previous_point - point;
    const auto to_next = next_point - point;

    const auto dot_product = (to_previous.normalized()).dot(to_next.normalized());
    residual = weight_ * (dot_product + 1.0);

    return true;
  }

 protected:
  const Gate &previous_gate_;
  const Gate &next_gate_;
};

#endif  // ANGLE_FUNCTOR_H
