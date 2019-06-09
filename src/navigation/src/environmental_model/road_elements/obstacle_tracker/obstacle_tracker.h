#ifndef OBSTACLE_TRACKER_H
#define OBSTACLE_TRACKER_H
#include <common/macros.h>

THIRD_PARTY_HEADERS_BEGIN
#include <limits>
#include <vector>
#include <memory>
#include <cassert>
#include <ros/console.h>
#include <tf2_eigen/tf2_eigen.h>
#include <Eigen/StdVector>
#include "perception_msgs/Obstacle.h"
#include "navigation_msgs/DrivingCorridor.h"
#include <sensor_msgs/Range.h>
THIRD_PARTY_HEADERS_END

#include "navigation/driving_corridor.h"
#include "obstacle_model.h"
struct TOFMeasurement {
  Eigen::Affine3d tof_pose;
  Eigen::Vector3d reflection_point;
  double dist_to_right_border;
  bool rayHitsObstacle() const { return dist_to_right_border > 0.05; }
};

struct TOFMeasurementAhead {
  Eigen::Affine3d tof_pose;
  Eigen::Vector3d reflection_point;
};

class ObstacleTracker {
 public:
  ObstacleTracker();
  ObstacleTracker(const ObstacleTracker& other) = delete;
  ObstacleTracker& operator=(const ObstacleTracker& other) = delete;
  ObstacleTracker(ObstacleTracker&& other) = default;
  ObstacleTracker& operator=(ObstacleTracker&& other) = default;
  void initialize(std::vector<std::unique_ptr<ObstacleModel>>&& models,
                  const Eigen::VectorXd& model_probabilities,
                  const Eigen::MatrixXd& model_transition_matrix,
                  const perception_msgs::Obstacle& obstacle);
  void predict(const double time_diff_sec,
               const DrivingCorridor& driving_corridor,
               const Eigen::Affine3d& vehicle_pose);
  void correct(const perception_msgs::Obstacle& obstacle);
  void correctTOFMeasurement(const TOFMeasurement& measurement);
  void correctTOFMeasurementAhead(const TOFMeasurementAhead& measurement);

  double getEstimatedSpeed();
  bool isInitialized();
  double getProbDynamic() const;
  common::Vector2dVector getVertices();
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  bool stopPrediction() const;
  unsigned int getNumObservations() const;


  static Eigen::Vector2d verticesToCenter(const perception_msgs::Obstacle& obstacle);

  long getNumberUpdatesWithoutNewMeasurement() const;

 private:
  struct IRMeasurement1D {
    boost::optional<double> start_obstacle;
    boost::optional<double> end_obstacle;
    double variance;
  };
  IRMeasurement1D correctTOFMeasurementIn1D(const bool ray_hits_obstacle,
                                            const double ir_position,
                                            const double reflection_point,
                                            const double obstacle_start,
                                            const double obstacle_end);
  void setSpeedOfStaticModelsToAverage();

  ObstacleModel::MeasurementVariance createCovarianceMatrix(
      const std::vector<std::pair<Eigen::Vector2d, int>>& vertices, const int best_shift);
  std::pair<ObstacleModel::Measurement, ObstacleModel::MeasurementVariance> obstacleMsgToMeasurementAndCovariance(
      const perception_msgs::Obstacle& obstacle);
  unsigned int n_observations_;
  bool initialized_;
  std::vector<std::unique_ptr<ObstacleModel>> models_;
  Eigen::VectorXd mu_; /**< Model probabilities */
  Eigen::MatrixXd pi_; /**< Transition matrix between model states */
  ObstacleModel::StateVector x_;
  ObstacleModel::CovarianceMatrix p_;
  Eigen::MatrixXd omega_; /**< Mixing probabilities */
  Eigen::VectorXd cbar_;  /**< Normalization constant */

  Eigen::Vector2d getNearestCorridorDirection(const DrivingCorridor& driving_corridor,
                                              const int start,
                                              const int end);
  void computeMixingProbabilities();
  void computeStateEstimate();
  long number_updates_without_new_measurement = 0;
  const long max_number_updates_without_new_measurement = 60;
  double measurement_to_right_gate_param = 1.0;
  DrivingCorridor current_corridor;
};

#endif
