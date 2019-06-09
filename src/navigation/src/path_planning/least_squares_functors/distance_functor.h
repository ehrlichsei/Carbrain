#ifndef DISTANCE_FUNCTOR_H
#define DISTANCE_FUNCTOR_H

#include "base_functor.h"

class DistanceFunctor : public BaseFunctor {

 public:
  DistanceFunctor(const Gate &gate, const Gate &next_gate, double weight = 1)
      : BaseFunctor(gate, weight), next_gate_(next_gate) {}

  template <typename T>
    bool operator()(const T gate_parameter[1], const T next_gate_parameter[1], T residual[2]) const {
      return (*this)(gate_parameter[0], next_gate_parameter[0], residual);
    }

    template <typename T>
      bool operator()(const T &gate_parameter, const T &next_gate_parameter, T residual[2]) const {

    const auto point = getValueAtParameter(gate_, gate_parameter);
    const auto next_point = getValueAtParameter(next_gate_, next_gate_parameter);
    residual[0] = weight_ * (next_point - point).x();
    residual[1] = weight_ * (next_point - point).y();
    return true;
  }

 protected:
  const Gate &next_gate_;
};

#endif  // DISTANCE_FUNCTOR_H
