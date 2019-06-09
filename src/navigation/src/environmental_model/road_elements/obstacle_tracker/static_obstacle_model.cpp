#include "static_obstacle_model.h"

// TODO: Pass process noise parameter via parameter interface.
StaticObstacleModel::StaticObstacleModel(const double initial_speed_in_m_per_s)
    : ObstacleModel(initial_speed_in_m_per_s), a_(CovarianceMatrix::Identity()) {
  const double variance = 0.005;
  q_ = variance * CovarianceMatrix::Identity();
  q_.bottomRightCorner<1, 1>() << 0;
}

void StaticObstacleModel::predict(const double /*time_diff_sec*/,
                                  const Eigen::Vector2d &lane_direction) {
  current_lane_direction = lane_direction;
  kf_predict(a_, q_);
}

void StaticObstacleModel::correct(const Measurement &measurement,
                                  const MeasurementVariance &r) {
  kf_correct(measurement, r);
}

ObstacleModel *StaticObstacleModel::clone() {
  return new StaticObstacleModel(*this);
}

std::string StaticObstacleModel::name() const { return "Static"; }

bool StaticObstacleModel::isStaticModel() const { return true; }
