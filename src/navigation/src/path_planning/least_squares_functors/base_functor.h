#ifndef BASE_FUNCTOR_H
#define BASE_FUNCTOR_H

template <typename T>
static Eigen::Matrix<T, 2, 1> getValueAtParameter(const Gate &gate, const T gate_parameter) {
  const auto gate_left = gate.getLeftLaneBoundary().topRows<2>().cast<T>();
  const auto gate_right = gate.getRightLaneBoundary().topRows<2>().cast<T>();
  return gate_left + gate_parameter * (gate_right - gate_left);
}

class BaseFunctor {
 public:
  BaseFunctor(const Gate &gate, double weight) : gate_(gate), weight_(weight) {}

 protected:
  const Gate &gate_;
  const double weight_;
};

class CenterFunctor : public BaseFunctor {

 public:
  CenterFunctor(const Gate &gate,
                double center_value,
                double center_weight,
                double residual_at_border,
                double border_start_distance)
      : BaseFunctor(gate, center_weight),
        center_(gate_.getParam<Gate::LEFT>() +
                center_value *
                    (gate_.getParam<Gate::RIGHT>() - gate_.getParam<Gate::LEFT>())),
        left_(gate.getParam<Gate::LEFT>()),
        right_(gate.getParam<Gate::RIGHT>()),
        residual_at_border_(residual_at_border),
        border_start_distance_(border_start_distance) {}

  template <typename T>
  bool operator()(const T gate_parameter[1], T residual[2]) const {
    return (*this)(gate_parameter[0], residual[0], residual[1]);
  }

  template <typename T>
  bool operator()(const T &gate_parameter, T &residual_center, T &residual_border) const {
    residual_center = weight_ * (gate_parameter - center_);
    residual_border = static_cast<T>(0.0);
    if (gate_parameter - left_ < border_start_distance_) {
      const auto linear_residual_left =
          (border_start_distance_ - (gate_parameter - left_)) / border_start_distance_;
      residual_border += residual_at_border_ * linear_residual_left * linear_residual_left;
    }
    if (right_ - gate_parameter < border_start_distance_) {
      const auto linear_residual_right =
          (border_start_distance_ - (right_ - gate_parameter)) / border_start_distance_;
      residual_border += residual_at_border_ * linear_residual_right * linear_residual_right;
    }
    return true;
  }

 protected:
  const double center_;
  const double left_;
  const double right_;
  const double residual_at_border_;
  const double border_start_distance_;
};

#endif  // BASE_FUNCTOR_H
