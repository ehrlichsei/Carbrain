#ifndef OBSTACLE_MODEL_H
#define OBSTACLE_MODEL_H
#include <common/macros.h>
#include <common/types.h>
#include "common/basic_statistics.h"
#include "common/basic_statistics_eigen.h"

THIRD_PARTY_HEADERS_BEGIN
#include <ros/console.h>
#include <eigen3/Eigen/Dense>
THIRD_PARTY_HEADERS_END

class ObstacleModel {
 public:
  using Measurement = Eigen::Matrix<double, 8, 1>;
  using MeasurementVariance = Eigen::Matrix<double, 8, 8>;
  using StateVector = Eigen::Matrix<double, 9, 1>;
  using CovarianceMatrix = Eigen::Matrix<double, 9, 9>;
  using MeasurementMatrix = Eigen::Matrix<double, 8, 9>;
  using TransposedMeasurementMatrix = Eigen::Matrix<double, 9, 8>;
  struct ObstacleTrackingInfo {
    common::Vector2dVector vertices;
    ObstacleTrackingInfo(const StateVector& state_vector) : vertices(4) {
      for (int i = 0; i < 4; ++i) {
        vertices[i] = state_vector.segment<2>(2 * i);
      }
    }
    ObstacleTrackingInfo(const Measurement& state_vector) : vertices(4) {
      for (int i = 0; i < 4; ++i) {
        vertices[i] = state_vector.segment<2>(2 * i);
      }
    }
    Measurement toMeasurement() {
      Measurement measurment_vector(Measurement::Zero());
      measurment_vector << vertices[0], vertices[1], vertices[2], vertices[3];
      return measurment_vector;
    }
    Eigen::Vector2d getCenter() {
      return 0.25 * common::basic_statistics::sum(vertices);
    }
  };
  // NOTE: For better numeric stability, centimeters are used for velocity.
  StateVector x;      /**< State expectation: (position_x [meters], position_y
                             [meters], velocity_along_lane [centimeters / second]) */
  CovarianceMatrix p; /**< State covariance matrix */

  ObstacleModel(const double initial_speed_in_m_per_s);
  virtual ~ObstacleModel() = default;
  ObstacleModel() = default;
  ObstacleModel(const ObstacleModel&) = default;
  ObstacleModel(ObstacleModel&&) = default;
  ObstacleModel& operator=(ObstacleModel&& other) = default;
  void initializeState(const Measurement& obst_position,
                       const MeasurementVariance& measurement_variance);
  virtual void predict(const double time_diff_sec, const Eigen::Vector2d& lane_direction) = 0;
  virtual void correct(const Measurement& measurement,
                       const MeasurementVariance& r /* Measurement variance */) = 0;
  virtual ObstacleModel* clone() = 0;  // Should use copy ctor.
  double likelihood();
  virtual std::string name() const = 0;
  virtual bool isStaticModel() const = 0;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

 protected:
  void kf_predict(const CovarianceMatrix& a, const CovarianceMatrix& q);
  void kf_correct(const Measurement& measurement,
                  const MeasurementVariance& r /* Measurement variance */);
  Eigen::Vector2d current_lane_direction{1, 0};

 private:
  StateVector x_predicted_;         /**< Predicted state */
  MeasurementMatrix h_;             /**< Measurement matrix */
  TransposedMeasurementMatrix h_t_; /**< Transposed measurement matrix */
  // Eigen::Matrix2d r_;               /**< Measurement variance */
  MeasurementVariance r_; /**< Latest measurement_variance */
  Measurement z_;         /**< Latest measurement */
  MeasurementVariance s_; /**< Innovation covariance matrix */
  double likelihood_ = 0.0;

  void updateLikelihood();
};

#endif
