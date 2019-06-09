#include "lane_obstacle_model.h"

LaneObstacleModel::LaneObstacleModel(const double initial_speed_in_m_per_s)
    : ObstacleModel(initial_speed_in_m_per_s) {
  const double variance = 0.005;
  q_ = variance * CovarianceMatrix::Identity();
  q_.bottomRightCorner<1, 1>() << 0;
}

void LaneObstacleModel::predict(const double time_diff_sec,
                                const Eigen::Vector2d& lane_direction) {
  current_lane_direction = lane_direction;
  // Linearized Kalman Filter prediction.
  CovarianceMatrix a_k =
      CovarianceMatrix::Identity();  // Predicts next state based on current.
  for (size_t i = 0; i < 8; ++i) {
    if (i % 2 == 0) {
      a_k(i, 8) = (lane_direction[0] * time_diff_sec) / 100;
    } else {
      a_k(i, 8) = (lane_direction[1] * time_diff_sec) / 100;
    }
  }

  // Eigen::Vector3d g_k; // Maps the acceleration variance to additive noise.
  // g_k << /*s=0.5*a*t^2*/ (0.5*lane_direction[0] *
  // time_diff_sec*time_diff_sec) / 100,// /100 because of cm to m (a is in
  // cm/s^2)
  //        /*s=0.5*a*t^2*/ (0.5*lane_direction[1] *
  //        time_diff_sec*time_diff_sec) / 100,
  //        /*v=a*t*/       time_diff_sec;
  StateVector g_k;  // Maps the acceleration variance to additive noise.
  for (size_t i = 0; i < 8; ++i) {
    if (i % 2 == 0) {
      g_k[i] = (0.5 * lane_direction[0] * time_diff_sec * time_diff_sec) / 100;
    } else {
      g_k[i] = (0.5 * lane_direction[1] * time_diff_sec * time_diff_sec) / 100;
    }
  }
  g_k[8] = time_diff_sec;
  // clang-format on
  // const auto g_k_t = g_k.transpose();
  // const double acceleration_variance = 0;  // 1e3;
  // std::cout << (g_k * acceleration_variance * g_k_t) << std::endl;
  const auto q_k = /*g_k * acceleration_variance * g_k_t +*/ q_;
  // std::cout <<"lane";
  kf_predict(a_k, q_k);
}

void LaneObstacleModel::correct(const Measurement& measurement,
                                const MeasurementVariance& r) {
  kf_correct(measurement, r);
  const double min_speed = 30;  // cm/s
  x[8] = std::max(x[8], min_speed);
}

ObstacleModel* LaneObstacleModel::clone() {
  return new LaneObstacleModel(*this);
}

std::string LaneObstacleModel::name() const { return "Lane"; }
bool LaneObstacleModel::isStaticModel() const { return false; }
