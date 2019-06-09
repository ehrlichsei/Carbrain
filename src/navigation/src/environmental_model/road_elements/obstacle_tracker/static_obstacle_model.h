#ifndef STATIC_OBSTACLE_MODEL_H
#define STATIC_OBSTACLE_MODEL_H
#include <common/macros.h>

THIRD_PARTY_HEADERS_BEGIN
#include <tf2_eigen/tf2_eigen.h>
THIRD_PARTY_HEADERS_END

#include "obstacle_model.h"

class StaticObstacleModel : public ObstacleModel {
 public:
  StaticObstacleModel(const double initial_speed_in_m_per_s);
  void predict(const double time_diff_sec, const Eigen::Vector2d& lane_direction) override;
  void correct(const Measurement& measurement,
               const MeasurementVariance& r /* Measurement variance */) override;
  virtual ObstacleModel* clone() override;
  virtual std::string name() const override;
  virtual bool isStaticModel() const override;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

 private:
  CovarianceMatrix a_;
  CovarianceMatrix q_;
};

#endif
