#ifndef PREFERED_POINT_FUNCTOR_H
#define PREFERED_POINT_FUNCTOR_H

#include "base_functor.h"

class PreferedPointFunctor : public BaseFunctor {

 public:
  explicit PreferedPointFunctor(const Gate &gate, double weight = 1)
      : BaseFunctor(gate, weight) {}

  template <typename T>
  bool operator()(const T gate_parameter[1], T residual[1]) const {
    return (*this)(gate_parameter[0], residual[0]);
  }

  template <typename T>
  bool operator()(const T &gate_parameter, T &residual) const {
    const auto prefered_point_parameter = gate_.getPreferedPathPointParameter();
    const auto point_weight = gate_.getPreferedPathWeight();
    residual = weight_ * point_weight * (prefered_point_parameter - gate_parameter);
    return true;
  }
};

#endif  // PREFERED_POINT_FUNCTOR_H
