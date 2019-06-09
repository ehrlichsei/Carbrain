#include "obstacle_tracker.h"
#include "common/best_score.h"
#include "common/console_colors.h"
#include "common/basic_statistics.h"
#include "common/basic_statistics_eigen.h"
#include "common/eigen_utils.h"
#include <boost/range/numeric.hpp>
#include <boost/range/algorithm/max_element.hpp>
#include "navigation/line_segment.h"
#include <boost/algorithm/clamp.hpp>
using namespace common::eigen_utils;

Eigen::Vector2d ObstacleTracker::verticesToCenter(const perception_msgs::Obstacle &obstacle) {
  if (obstacle.vertices.empty()) {
    ROS_ERROR_STREAM("obstacle.vertices empty in verticesToCenter()");
    return Eigen::Vector2d::Zero();
  }
  Eigen::Vector2d sum_pos(0, 0);
  for (const auto &corner : obstacle.vertices) {
    sum_pos += Eigen::Vector2d(corner.x, corner.y);
  }
  return sum_pos / obstacle.vertices.size();
}
void sortCounterClockwise(std::vector<std::pair<Eigen::Vector2d, int>> &vertices) {
  const Eigen::Vector2d center =
      0.25 * boost::accumulate(vertices,
                               Eigen::Vector2d(0, 0),
                               [](const Eigen::Vector2d &acc,
                                  const std::pair<Eigen::Vector2d, int> &new_element) {
                                 return acc + new_element.first;
                               });
  std::sort(vertices.begin(),
            vertices.end(),
            [&center](const auto &a, const auto &b) {
              const Eigen::Vector2d center_to_a = (a.first - center).normalized();
              const double angle_a = std::atan2(center_to_a.y(), center_to_a.x());
              const Eigen::Vector2d center_to_b = (b.first - center).normalized();
              const double angle_b = std::atan2(center_to_b.y(), center_to_b.x());
              return angle_a < angle_b;
            });
}
bool checkSorted(common::Vector2dVector &vertices) {
  const Eigen::Vector2d center = 0.25 * common::basic_statistics::sum(vertices);
  int sign_changes = 0;
  for (int i = 0; i < 4; ++i) {
    const Eigen::Vector2d center_to_a = (vertices[i] - center).normalized();
    const double angle_a = std::atan2(center_to_a.y(), center_to_a.x());
    const Eigen::Vector2d center_to_b = (vertices[(i + 1) % 4] - center).normalized();
    const double angle_b = std::atan2(center_to_b.y(), center_to_b.x());
    if (angle_a < angle_b) {
      continue;
    }
    sign_changes++;
  }
  return sign_changes == 1;
}

std::pair<ObstacleModel::Measurement, ObstacleModel::MeasurementVariance> ObstacleTracker::obstacleMsgToMeasurementAndCovariance(
    const perception_msgs::Obstacle &obstacle) {
  ObstacleModel::ObstacleTrackingInfo obstacle_info(x_);
  if (!checkSorted(obstacle_info.vertices)) {
    ROS_ERROR_STREAM("not sorted vertices in obstacle tracker");
  }
  std::vector<std::pair<Eigen::Vector2d, int>> msg_vertices(4);
  boost::transform(obstacle.vertices,
                   obstacle.vertices_detected,
                   msg_vertices.begin(),
                   [](const auto &point, const int detected) {
                     return std::make_pair(Eigen::Vector2d(point.x, point.y), detected);
                   });

  sortCounterClockwise(msg_vertices);
  const auto scoreFkt = [&](const int shift) {
    double sum_difference = 0;
    for (int i = 0; i < 4; ++i) {
      sum_difference += (obstacle_info.vertices[i] -
                         msg_vertices[(i + shift) % 4].first).squaredNorm();
    }
    return sum_difference;
  };

  int best_shift = 0;
  double min_score = scoreFkt(0);
  for (int i = 0; i < 4; ++i) {
    const double score = scoreFkt(i);
    if (score < min_score) {
      min_score = score;
      best_shift = i;
    }
  }

  ObstacleModel::Measurement measurement;
  for (int i = 0; i < 4; ++i) {
    measurement.segment<2>(2 * i) = msg_vertices[(i + best_shift) % 4].first;
  }

  return std::make_pair(measurement, createCovarianceMatrix(msg_vertices, best_shift));
}

ObstacleTracker::ObstacleTracker() : n_observations_(0), initialized_(false) {}


double ObstacleTracker::getEstimatedSpeed() { return x_[8] / 100.0; }

bool ObstacleTracker::isInitialized() { return initialized_; }

double ObstacleTracker::getProbDynamic() const {
  double prob_dynamic = 0;
  for (size_t i = 0; i < models_.size(); ++i) {
    if (!models_[i]->isStaticModel()) {
      prob_dynamic += mu_[i];
    }
  }
  return prob_dynamic;
}

common::types::Vector2dVector ObstacleTracker::getVertices() {
  ObstacleModel::ObstacleTrackingInfo obstacle_info(x_);
  return obstacle_info.vertices;
}

ObstacleModel::MeasurementVariance ObstacleTracker::createCovarianceMatrix(
    const std::vector<std::pair<Eigen::Vector2d, int>> &vertices, const int best_shift) {
  const double variance_detected = 0.003;
  const double variance_estimated = 0.08;
  const double variance_insecure = 0.4;
  ObstacleModel::MeasurementVariance measurement_variance =
      ObstacleModel::MeasurementVariance::Zero();

  for (int i = 0; i < 4; ++i) {
    double variance = 100;
    const int current_index = (i + best_shift) % 4;
    switch (vertices[current_index].second) {
      case perception_msgs::Obstacle::VERTEX_DETECTED:
        variance = variance_detected;
        break;
      case perception_msgs::Obstacle::VERTEX_ESTIMATED:
        variance = variance_estimated;
        break;
      case perception_msgs::Obstacle::VERTEX_INSECURE:
        variance = variance_insecure;
        break;
      default:
        ROS_ERROR("invalid type in vector vertices_detected");
    }
    measurement_variance(2 * i, 2 * i) = variance;
    measurement_variance(2 * i + 1, 2 * i + 1) = variance;
  }
  return measurement_variance;
}

bool ObstacleTracker::stopPrediction() const {
  double factor = 1.0;
  if (getProbDynamic() > 0.9 && measurement_to_right_gate_param < 0.5) {
    factor = 4 * measurement_to_right_gate_param * measurement_to_right_gate_param;  // <= 1
  }
  return number_updates_without_new_measurement > factor * max_number_updates_without_new_measurement;
}

unsigned int ObstacleTracker::getNumObservations() const {
  return n_observations_;
}

void ObstacleTracker::initialize(std::vector<std::unique_ptr<ObstacleModel>> &&models,
                                 const Eigen::VectorXd &model_probabilities,
                                 const Eigen::MatrixXd &model_transition_matrix,
                                 const perception_msgs::Obstacle &obstacle) {
  assert(model_transition_matrix.rows() == model_transition_matrix.cols());
  assert(model_transition_matrix.rows() == static_cast<int>(models.size()));
  assert(model_probabilities.rows() == static_cast<int>(models.size()));
  // TODO: Store models_.size() as attribute.
  models_ = std::move(models);
  mu_ = model_probabilities;
  mu_ /= mu_.sum();  // Normalize probabilities.
  pi_ = model_transition_matrix;
  // Initialize state.
  if (obstacle.vertices.size() != 4) {
    ROS_ERROR_STREAM("obstacle.vertices.size not 4 in initialize()");
    return;
  }
  std::vector<std::pair<Eigen::Vector2d, int>> msg_vertices(4);
  boost::transform(obstacle.vertices,
                   obstacle.vertices_detected,
                   msg_vertices.begin(),
                   [](const auto &point, const int detected) {
                     return std::make_pair(Eigen::Vector2d(point.x, point.y), detected);
                   });
  sortCounterClockwise(msg_vertices);  // TODO set also the initial variance
  ObstacleModel::Measurement measurement;
  for (int i = 0; i < 4; ++i) {
    measurement.segment<2>(2 * i) = msg_vertices[i].first;
  }
  x_ << measurement,  // Position
      0;              // Velocity
  ObstacleModel::MeasurementVariance measurement_variance =
      createCovarianceMatrix(msg_vertices, 0);
  p_.topLeftCorner<8, 8>() = measurement_variance;
  for (const auto &p_m : models_) {
    p_m->initializeState(measurement, measurement_variance);
  }
  omega_.resize(models_.size(), models_.size());
  cbar_.resize(models_.size());
  computeMixingProbabilities();
  computeStateEstimate();
  initialized_ = true;
  n_observations_++;
}

void ObstacleTracker::predict(const double time_diff_sec,
                              const DrivingCorridor &driving_corridor,
                              const Eigen::Affine3d &vehicle_pose) {
  current_corridor = driving_corridor;
  Eigen::Vector3d current_center(0, 0, 0);
  ObstacleModel::ObstacleTrackingInfo info(x_);
  current_center << info.getCenter(), 0;
  const double dist_behind_car = 0.2;
  if ((vehicle_pose.inverse() * current_center).x() > -dist_behind_car) {
    number_updates_without_new_measurement++;
  }
  ROS_DEBUG_STREAM("number_updates_without_new_measurement "
                   << COLOR_YELLOW << number_updates_without_new_measurement);
  if (stopPrediction()) {
    ROS_WARN_STREAM_THROTTLE(
        0.5,
        "Received " << number_updates_without_new_measurement
                    << " times no matching obstacle measurement. Stop "
                       "predicting position.");
    return;
  }

  if (initialized_) {
    if (driving_corridor.empty()) {
      return;
    }
    const Eigen::Vector2d lane_dir =
        getNearestCorridorDirection(driving_corridor, -2, 10);

    // IMM prediction
    // Mix states.
    std::vector<ObstacleModel::StateVector, Eigen::aligned_allocator<ObstacleModel::StateVector>> xs_mixed(
        models_.size());
    std::vector<ObstacleModel::CovarianceMatrix, Eigen::aligned_allocator<ObstacleModel::CovarianceMatrix>> ps_mixed(
        models_.size());
    for (size_t i = 0; i < models_.size(); ++i) {
      // Mix state.
      ObstacleModel::StateVector x_mixed_i = ObstacleModel::StateVector::Zero();
      for (size_t j = 0; j < models_.size(); ++j) {
        x_mixed_i += models_[j]->x * omega_(j, i);
      }
      xs_mixed[i] = x_mixed_i;
      // Mix covariance.
      ObstacleModel::CovarianceMatrix p_mixed_i =
          ObstacleModel::CovarianceMatrix::Zero();
      for (size_t j = 0; j < models_.size(); ++j) {
        const auto y = models_[j]->x - x_mixed_i;
        const auto y_t = y.transpose();
        p_mixed_i += (models_[j]->p + y * y_t) * omega_(j, i);
      }
      ps_mixed[i] = p_mixed_i;
    }
    // Inject mixed states for following estimations.
    for (size_t i = 0; i < models_.size(); ++i) {
      auto &p_m = models_[i];
      p_m->x = xs_mixed[i];
      p_m->p = ps_mixed[i];
      p_m->predict(time_diff_sec, lane_dir);
    }
    computeStateEstimate();
  }
}

void ObstacleTracker::correct(const perception_msgs::Obstacle &obstacle) {
  if (obstacle.vertices.size() != 4) {
    ROS_ERROR_STREAM(
        "obstacle.vertices.size not 4 in obstacleMsgToMeasurement(). "
        "obstacle.vertices.size is "
        << obstacle.vertices.size());
  }

  number_updates_without_new_measurement = 0;
  if (!initialized_) {
    return;
  }
  ObstacleModel::Measurement z;
  ObstacleModel::MeasurementVariance z_variance;
  std::tie(z, z_variance) = obstacleMsgToMeasurementAndCovariance(obstacle);
  const double clamped_certainty = boost::algorithm::clamp(obstacle.certainty, 0.3, 0.7);
  const double factor = 0.7 - clamped_certainty;
  z_variance += std::pow(factor, 2) * ObstacleModel::MeasurementVariance::Identity();

  // min(std::pow(factor, 2)) = min(std::pow(0  , 2)) = 0
  // max(std::pow(factor, 2)) = min(std::pow(0.4, 2)) = 0.16

  ObstacleModel::ObstacleTrackingInfo info(z);
  const Eigen::Vector2d measurement_center = info.getCenter();
  if (current_corridor.size() > 1) {
    const Gate nearest_gate =
        *common::min_score(
            current_corridor,
            [&](const auto &gate) {
              const Eigen::Vector3d gate_center = gate.getLaneCenter();
              return (gate_center.topRows<2>() - measurement_center).norm();
            });
    std::vector<double> dist_to_right(4);
    boost::transform(info.vertices,
                     dist_to_right.begin(),
                     [&](const Eigen::Vector2d &v) {
                       return (1.0 - nearest_gate.toParam(to3D(v)));
                     });
    measurement_to_right_gate_param = *boost::max_element(dist_to_right);
  }

  // IMM correction
  Eigen::VectorXd likelihoods(models_.size());
  for (int i = 0; i < static_cast<int>(models_.size()); ++i) {
    const auto &p_m = models_[i];
    p_m->correct(z, z_variance);
    likelihoods[i] = p_m->likelihood();
    ROS_DEBUG_STREAM("Likelihood for " << p_m->name() << ": " << likelihoods[i] << ".");
  }

  mu_ = cbar_.cwiseProduct(likelihoods);
  mu_ /= mu_.sum();
  computeMixingProbabilities();
  computeStateEstimate();

  setSpeedOfStaticModelsToAverage();

  n_observations_++;
  ROS_DEBUG_STREAM("Observations: " << n_observations_);
  ROS_DEBUG_STREAM("p(static): " << mu_[0] << ", p(following): " << mu_[1]);
}

void ObstacleTracker::setSpeedOfStaticModelsToAverage() {
  // calculate estimated speed of all dynamic models
  double sum_speed_dynamic = 0;
  unsigned int num_dyn_models = 0;
  for (const auto &model : models_) {
    if (model->isStaticModel()) {
      continue;
    }
    sum_speed_dynamic = model->x[8];
    num_dyn_models++;
  }
  const double avg_speed = num_dyn_models == 0 ? 0 : sum_speed_dynamic / num_dyn_models;
  // set speed of all static models to the average speed
  for (const auto &model : models_) {
    if (model->isStaticModel()) {
      model->x[8] = avg_speed;
    }
  }
}

ObstacleTracker::IRMeasurement1D ObstacleTracker::correctTOFMeasurementIn1D(
    const bool ray_hits_obstacle,
    const double tof_position,
    const double reflection_point,
    const double obstacle_start,
    const double obstacle_end) {


  ROS_DEBUG_STREAM("ray_hits_obstacle " << (ray_hits_obstacle ? COLOR_BLUE : COLOR_RED)
                                        << std::boolalpha << ray_hits_obstacle);
  const auto isBetween = [](const double a, const double start, const double end) {
    return start < a && a < end;
  };


  if (!ray_hits_obstacle) {
    const bool hits_predicted_tof = isBetween(tof_position, obstacle_start, obstacle_end);
    const double variance_no_detection = 0.1;

    if (!hits_predicted_tof) {
      return IRMeasurement1D{boost::none, boost::none, variance_no_detection};
    }

    const double dist_to_end = obstacle_end - tof_position;
    const double dist_to_start = tof_position - obstacle_start;

    // hits_predicted:
    const double shift = 0.03;
    if (dist_to_start < dist_to_end) {
      return IRMeasurement1D{tof_position + shift, boost::none, variance_no_detection};
    } else {
      return IRMeasurement1D{boost::none, tof_position - shift, variance_no_detection};
    }
  }

  assert(ray_hits_obstacle);  // now the reflection_point is valid => use it now
                              // instead of the tof_position
  const double delta = 0.04;
  const bool hits_predicted_reflection_point =
      isBetween(reflection_point, obstacle_start - delta, obstacle_end + delta);
  const double variance_detection_predicted = 0.4;
  if (hits_predicted_reflection_point) {
    return IRMeasurement1D{obstacle_start, boost::none, variance_detection_predicted};
  }

  // no hit predicted:
  const double variance_detection_not_predicted = 0.01;
  const double shift = 0.03;
  if (reflection_point < obstacle_start) {  // tof before estimated obstacle
    return IRMeasurement1D{reflection_point - shift, boost::none, variance_detection_not_predicted};
  } else {  // tof behind estimated obstacle
    return IRMeasurement1D{boost::none, reflection_point + shift, variance_detection_not_predicted};
  }
}


void ObstacleTracker::correctTOFMeasurement(const TOFMeasurement &measurement) {
  if (measurement.rayHitsObstacle()) {
    number_updates_without_new_measurement = 0;
  }
  if (!initialized_ || current_corridor.empty()) {
    return;
  }
  const Eigen::Vector2d direction = getNearestCorridorDirection(
      current_corridor, -5, 5);  // TODO is only correct if corridor is straight

  const Eigen::Vector2d zeroPoint = measurement.tof_pose.translation().topRows<2>();
  const auto transformation2DTo1D = [&zeroPoint, &direction](const Eigen::Vector2d &point) {
    return (point - zeroPoint).dot(direction);
  };
  const auto transformation3DTo1D = [&transformation2DTo1D](const Eigen::Vector3d &point) {
    return transformation2DTo1D(point.topRows<2>());
  };
  const ObstacleModel::ObstacleTrackingInfo obstacle(x_);
  const auto min_max_vertices =
      common::minmax_score(obstacle.vertices, transformation2DTo1D);
  const Eigen::Vector2d min_vertex = *min_max_vertices.first;
  const Eigen::Vector2d max_vertex = *min_max_vertices.second;
  const double start_obstacle = transformation2DTo1D(min_vertex);
  const double end_obstacle = transformation2DTo1D(max_vertex);

  const IRMeasurement1D measurement_1d = correctTOFMeasurementIn1D(
      measurement.rayHitsObstacle(),
      transformation3DTo1D(measurement.tof_pose.translation()),
      transformation3DTo1D(measurement.reflection_point),
      start_obstacle,
      end_obstacle);
  if (measurement_1d.start_obstacle == boost::none &&
      measurement_1d.end_obstacle == boost::none) {
    return;
  }
  if (measurement_1d.start_obstacle != boost::none &&
      measurement_1d.end_obstacle != boost::none) {
    ROS_ERROR_STREAM(
        "measurement_1d has start_obstacle and end_obstacle. One of them is "
        "ignored.");
    assert(false);
  }

  ObstacleModel::ObstacleTrackingInfo obstacle_measurement = obstacle;
  assert((measurement_1d.end_obstacle != boost::none &&
          measurement_1d.start_obstacle == boost::none) ||
         (measurement_1d.end_obstacle == boost::none &&
          measurement_1d.start_obstacle != boost::none));
  const Eigen::Vector2d translation =
      measurement_1d.start_obstacle != boost::none
          ? (*measurement_1d.start_obstacle - start_obstacle) * direction.topRows<2>()  // start is set
          : (*measurement_1d.end_obstacle - end_obstacle) * direction.topRows<2>();  // end is set

  // create measured obstacle through shifting of the estimated obstacle along
  // the corridor direction
  for (auto &v : obstacle_measurement.vertices) {
    v += translation;
  }

  ObstacleModel::MeasurementVariance measurement_variance =
      measurement_1d.variance * ObstacleModel::MeasurementVariance::Identity();
  for (const auto &model : models_) {
    model->correct(obstacle_measurement.toMeasurement(), measurement_variance);
  }
  computeStateEstimate();
  setSpeedOfStaticModelsToAverage();
}

void ObstacleTracker::correctTOFMeasurementAhead(const TOFMeasurementAhead &measurement) {
  if (number_updates_without_new_measurement < 4) {
    // if obstacle is detected in the image ignore the tof measurment
    ROS_DEBUG("Ignore the tof measurment");
    return;
  }
  ROS_DEBUG("correctIRMeasurementAhead");
  number_updates_without_new_measurement = 0;
  ObstacleModel::ObstacleTrackingInfo obstacle(x_);
  const Eigen::Vector2d nearest_obstacle_point =
      *common::min_score(obstacle.vertices,
                         [&measurement](const Eigen::Vector2d &vertex) {
                           return (measurement.tof_pose.inverse() * to3D(vertex)).x();
                         });
  const Eigen::Vector2d ray_direction =
      (to2D(measurement.reflection_point - measurement.tof_pose.translation())).normalized();
  const Eigen::Vector2d translation =
      (to2D(measurement.reflection_point) - nearest_obstacle_point).dot(ray_direction) * ray_direction;

  ObstacleModel::ObstacleTrackingInfo obstacle_measurement = obstacle;

  // create measured obstacle through shifting of the estimated obstacle along
  // the ray direction
  for (auto &v : obstacle_measurement.vertices) {
    v += translation;
  }

  ObstacleModel::MeasurementVariance measurement_variance =
      0.5 * ObstacleModel::MeasurementVariance::Identity();
  // IMM correction
  Eigen::VectorXd likelihoods(models_.size());
  for (int i = 0; i < static_cast<int>(models_.size()); ++i) {
    const auto &p_m = models_[i];
    p_m->correct(obstacle_measurement.toMeasurement(), measurement_variance);
    likelihoods[i] = p_m->likelihood();
    ROS_DEBUG_STREAM("Likelihood for " << p_m->name() << ": " << likelihoods[i] << ".");
  }

  mu_ = cbar_.cwiseProduct(likelihoods);
  mu_ /= mu_.sum();
  computeMixingProbabilities();
  computeStateEstimate();

  setSpeedOfStaticModelsToAverage();

  // TODO test if the intersection point with the line is better than the
  // nearest vertex
  /*std::array<LineSegment, 4> line_segments;
  std::transform(info.vertices.begin(),
                 info.vertices.end() - 1,
                 info.vertices.begin() + 1,
                 line_segments.begin(),
                 [](const Eigen::Vector2d &a, const Eigen::Vector2d &b) {
                   return LineSegment(a, b);
                 });
  line_segments[3] = LineSegment(info.vertices[3], info.vertices[0]);
  const LineSegment line =
      *common::min_score(
          line_segments,
          [&measurement](const LineSegment &line_segment) {
            if
  (line_segment.isPointOnLeftSide(measurement.tof_pose.translation())) {
              return std::numeric_limits<double>::max();
            }
            return line.getDistanceToPoint(measurement.reflection_point);
          });*/
}

Eigen::Vector2d ObstacleTracker::getNearestCorridorDirection(const DrivingCorridor &driving_corridor,
                                                             const int start,
                                                             const int end) {
  const Eigen::Vector2d obst_pos = x_.topRows<2>();
  auto nearest_gate =
      common::min_score(driving_corridor,
                        [&](const auto &gate) {
                          const Eigen::Vector3d gate_pos = gate.getCenter();
                          return (gate_pos.topRows<2>() - obst_pos).norm();
                        });
  const int index_nearest_gate =
      static_cast<int>(std::distance(driving_corridor.begin(), nearest_gate));
  Eigen::Vector3d direction_sum(0, 0, 0);
  for (int i = std::max(0, index_nearest_gate + start);
       i < std::min(static_cast<int>(driving_corridor.size()), index_nearest_gate + end);
       ++i) {
    direction_sum += driving_corridor.at(i).getVectorLeftToRight().normalized();
  }
  return direction_sum.unitOrthogonal().topRows<2>();
}

void ObstacleTracker::computeMixingProbabilities() {
  for (int row = 0; row < mu_.rows(); ++row) {
    cbar_[row] = mu_.dot(pi_.row(row).transpose());
  }
  for (int i = 0; i < static_cast<int>(models_.size()); ++i) {
    for (int j = 0; j < static_cast<int>(models_.size()); ++j) {
      omega_(i, j) = (pi_(i, j) * mu_[i]) / cbar_[j];
    }
  }
}

void ObstacleTracker::computeStateEstimate() {
  x_ = ObstacleModel::StateVector::Zero();
  for (size_t i = 0; i < models_.size(); ++i) {
    x_ += models_[i]->x * mu_[i];
  }

  p_ = ObstacleModel::CovarianceMatrix::Zero();
  for (size_t i = 0; i < models_.size(); ++i) {
    auto &p_m = models_[i];
    const auto y = p_m->x - x_;
    const auto y_t = y.transpose();
    p_ += (models_[i]->p + y * y_t) * mu_[i];
  }
}

long ObstacleTracker::getNumberUpdatesWithoutNewMeasurement() const {
  return number_updates_without_new_measurement;
}
