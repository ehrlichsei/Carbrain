#ifndef ENDPOINT_ANGLE_FUNCTOR_H
#define ENDPOINT_ANGLE_FUNCTOR_H

#include "base_functor.h"

class EndpointAngleFunctor : public BaseFunctor {

 public:
  EndpointAngleFunctor(const Gate &previous_gate, const Gate &gate, double weight = 1)
      : BaseFunctor(gate, weight), previous_gate_(previous_gate) {}

  template <typename T>
  bool operator()(const T previous_gate_parameter[1],
                  const T gate_parameter[1],
                  T residual[1]) const {
    return (*this)(previous_gate_parameter[0], gate_parameter[0], residual[0]);
  }

  template <typename T>
  bool operator()(const T &previous_gate_parameter, const T &gate_parameter, T &residual) const {
    const auto previous_point = getValueAtParameter(previous_gate_, previous_gate_parameter);
    const auto point = getValueAtParameter(gate_, gate_parameter);
    const auto driving_direction = (point - previous_point).normalized();
    const Eigen::Vector2d target_driving_direction =
        (gate_.getTransformationFromGateFrame().rotation() *
         Eigen::Vector3d::UnitY()).topRows<2>();
    const auto cross2 = driving_direction.x() * target_driving_direction.y() -
                        driving_direction.y() * target_driving_direction.x();
    residual = weight_ * cross2;
    return true;
  }

 protected:
  const Gate &previous_gate_;
};

#endif  // ENDPOINT_ANGLE_FUNCTOR_H
