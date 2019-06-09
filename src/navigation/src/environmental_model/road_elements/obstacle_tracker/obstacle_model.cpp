#include "obstacle_model.h"
#include <boost/algorithm/clamp.hpp>

// TODO: Get parameters from param iface.
ObstacleModel::ObstacleModel(const double initial_speed_in_m_per_s)
    : x(StateVector::Zero()), h_(MeasurementMatrix::Identity()) {
  x[8] = initial_speed_in_m_per_s * 100.0;

  p = 50 * CovarianceMatrix::Identity();
  p.bottomRightCorner<1, 1>() << 1000;
  h_t_ = h_.transpose();
}

void ObstacleModel::initializeState(const Measurement& obst_position,
                                    const MeasurementVariance& measurement_variance) {
  x.topRows<8>() = obst_position;
  p.topLeftCorner<8, 8>() = measurement_variance;
}

double ObstacleModel::likelihood() { return likelihood_; }

void ObstacleModel::kf_predict(const CovarianceMatrix& a, const CovarianceMatrix& q) {
  x = x_predicted_ = a * x;
  p = a * p * a.transpose() + q;
}

void ObstacleModel::kf_correct(const Measurement& measurement, const MeasurementVariance& r) {
  z_ = measurement;
  r_ = r;
  // Kalman Filter correction
  s_ = h_ * p * h_t_ + r;
  const Eigen::Matrix<double, 9, 8> k = p * h_t_ * s_.inverse();
  const Eigen::Matrix<double, 8, 9> k_t = k.transpose();
  const Eigen::Matrix<double, 8, 1> residual = (z_ - h_ * x);
  ROS_DEBUG_STREAM("" << name() << ": Absolute prediction error: " << residual.norm() << ".");

  x = x + k * residual;
  // Joseph form covariance correction (for better numerical stability)
  const CovarianceMatrix a = CovarianceMatrix::Identity() - k * h_;
  const CovarianceMatrix a_t = a.transpose();
  p = a * p * a_t + k * r * k_t;
  updateLikelihood();
}

/*
static double pdf(const Eigen::Vector2d& x, const Eigen::Vector2d& mean, const
Eigen::Matrix2d& cov) {
  constexpr double inv_sqrt_2pi = 0.3989422804014327;
  double quadform  = (x - mean).transpose() * cov.inverse() * (x - mean);
  double norm_constant = std::pow(inv_sqrt_2pi, cov.rows()) *
std::pow(cov.determinant(), -0.5);
  return norm_constant * std::exp(-.5 * quadform);
}
*/

// From
// https://stackoverflow.com/questions/41538095/evaluate-multivariate-normal-gaussian-density-in-c
/*static double pdf(const ObstacleModel::Measurement& x,
                  const ObstacleModel::Measurement& mean,
                  const ObstacleModel::MeasurementVariance& cov) {
  const double logSqrt2Pi = 0.5 * std::log(2 * M_PI);
  typedef Eigen::LLT<Eigen::MatrixXd> Chol;
  Chol chol(cov);
  // Handle non positive definite covariance somehow:
  if (chol.info() != Eigen::Success)
    return 1e-100;
  const Chol::Traits::MatrixL& L = chol.matrixL();
  double quadform = (L.solve(x - mean)).squaredNorm();
  return std::exp(-x.rows() * logSqrt2Pi - 0.5 * quadform) / L.determinant();
}*/

void ObstacleModel::updateLikelihood() {
  const Measurement predicted = h_ * x_predicted_;
  ObstacleTrackingInfo info_predicted(predicted);
  ObstacleTrackingInfo info_measured(z_);
  const auto compareFkt = [this](const Eigen::Vector2d& a, const Eigen::Vector2d& b) {
    return (b - a).dot(this->current_lane_direction) > 0;
  };
  const Eigen::Vector2d nearest_predicted =
      *std::min_element(
          info_predicted.vertices.begin(), info_predicted.vertices.end(), compareFkt);
  const Eigen::Vector2d nearest_detected =
      *std::min_element(info_measured.vertices.begin(), info_measured.vertices.end(), compareFkt);
  const double diff_in_lane_direction =
      (nearest_predicted - nearest_detected).dot(current_lane_direction);
  const double min_max_distance = 0.05;
  const double clamped_distance = boost::algorithm::clamp(
      diff_in_lane_direction, -min_max_distance, min_max_distance);
  const double scaling = 2;
  // max likelihood = scaling*min_max_distance+0.5
  // min likelihood = -scaling*min_max_distance+0.5
  if (isStaticModel()) {
    likelihood_ = scaling * clamped_distance + 0.5;
  } else {
    likelihood_ = -scaling * clamped_distance + 0.5;
  }
  // likelihood_ = std::max(0.00001, pdf(z_, z_predicted, s_)); // this would be
  // the correct likelihood but the above seems to be better
}
