#include "obstacle.h"
#include "../road_element_visitors/road_element_visitor.h"
#include "common/best_score.h"
#include "common/eigen_utils.h"
#include <tf2_eigen/tf2_eigen.h>

namespace environmental_model {
using namespace common::eigen_utils;
Obstacle::Obstacle(const perception_msgs::Obstacle &obstacle) : msg(obstacle) {
  initializeTracker(obstacle);
}

void Obstacle::predict(const double time_diff_sec,
                       const DrivingCorridor &driving_corridor,
                       const Eigen::Affine3d &vehicle_pose) {
  if (tracker.isInitialized()) {
    tracker.predict(time_diff_sec, driving_corridor, vehicle_pose);
  }
  updateMessage();
}

void Obstacle::accept(RoadElementVisitor &visitor, TrackingElement &tracking_element) {
  visitor.visit(*this, tracking_element);
}

void Obstacle::newMsg(const MsgCollection &msg, const Eigen::Affine3d & /*vehicle_pose*/) {
  if (msg.obstacles.sub_messages.empty()) {
    return;
  }
  const auto certaintyAsScore =
      [](const auto &message) { return message.certainty; };
  const auto highest_prob_msg =
      *common::max_score(msg.obstacles.sub_messages, certaintyAsScore);
  if (highest_prob_msg.certainty > 0.2 || this->msg.certainty < highest_prob_msg.certainty) {
    const auto &new_msg = highest_prob_msg;
    this->msg = new_msg;
    if (!tracker.isInitialized()) {
      initializeTracker(new_msg);
    } else {
      tracker.correct(new_msg);
    }
    updateMessage();
    ROS_DEBUG_STREAM("Estimated obstacle velocity: " << tracker.getEstimatedSpeed());
  }
}

void Obstacle::addMsgOfConflictingObstacle(const perception_msgs::Obstacle &confl_msg) {
  if (!tracker.isInitialized()) {
    return;
  }
  tracker.correct(confl_msg);
  updateMessage();
}

void Obstacle::collectMsgs(MsgCollection &collection) {
  collection.obstacles.sub_messages.push_back(msg);
}

void Obstacle::setProbabilityFromLastMsg(RoadElement::ProbabilityVector &probabilities) {
  if (msg.certainty > probabilities[RoadElementType::Obstacle]) {
    probabilities[RoadElementType::Obstacle] = msg.certainty;
  }
}

void Obstacle::updateMessage() {
  common::Vector2dVector vertices = tracker.getVertices();

  msg.vertices.clear();
  std::transform(vertices.begin(),
                 vertices.end(),
                 std::back_inserter(msg.vertices),
                 [](const Eigen::Vector2d &v) {
                   Eigen::Vector3d vertex(v.x(), v.y(), 0);
                   return tf2::toMsg(vertex);
                 });
  msg.base_hull_polygon = msg.vertices;
  setBaseAreaIfNotEmpty(msg.base_hull_polygon);
}

void Obstacle::initializeTracker(const perception_msgs::Obstacle &obstacle) {
  // Build models for IMM tracker.
  // TODO: Get parameters and stuff from param iface.
  const double initial_speed_in_m_per_s = 0.6;
  std::vector<std::unique_ptr<ObstacleModel>> models;
  models.push_back(std::make_unique<StaticObstacleModel>(initial_speed_in_m_per_s));
  models.push_back(std::make_unique<LaneObstacleModel>(initial_speed_in_m_per_s));
  Eigen::VectorXd state_probabilities(2);
  state_probabilities << 0.8, 0.2;
  Eigen::MatrixXd state_transition_probabilities(2, 2);
  const double change_prob = 1e-12;
  // clang-format off
  state_transition_probabilities << 1 - change_prob, change_prob,
                                    change_prob, 1 - change_prob;
  // clang-format on
  tracker.initialize(
      std::move(models), state_probabilities, state_transition_probabilities, obstacle);
}
perception_msgs::Obstacle Obstacle::getMsg() const { return msg; }

Obstacle::DynamicState Obstacle::getDynamicState() {
  return {tracker.getProbDynamic(), tracker.getEstimatedSpeed()};
}

bool Obstacle::isNew() {
  if (!tracker.isInitialized()) {
    return true;
  }
  return tracker.getNumObservations() < 2;
}

bool Obstacle::canBeConnectedToTOFMeasurement(const TOFMeasurement &measurement) {
  if (!tracker.isInitialized()) {
    ROS_ERROR("obstacle tracker not initialized but received ir measurement");
    return false;
  }
  const common::Vector2dVector vertices = tracker.getVertices();
  std::vector<double> vertices_in_ir_front_x(vertices.size());
  boost::transform(vertices,
                   vertices_in_ir_front_x.begin(),
                   [&measurement](const Eigen::Vector2d &v) {
                     return (measurement.tof_pose.inverse() * to3D(v)).x();
                   });
  assert(vertices_in_ir_front_x.size() == 4);
  const double min_dist_ortho_to_corridor =
      *std::min_element(vertices_in_ir_front_x.begin(), vertices_in_ir_front_x.end());
  if (min_dist_ortho_to_corridor < 0.0) {
    return false;
  }
  Eigen::Vector3d obstacle_position = to3D(getEstimatedPosition());
  const Eigen::Vector3d obstacle_position_in_ir_front =
      measurement.tof_pose.inverse() * obstacle_position;

  return -1.25 < obstacle_position_in_ir_front.y() &&
         obstacle_position_in_ir_front.y() < 0.55;
}

bool Obstacle::canBeConnectedToTOFMeasurementAhead(const TOFMeasurementAhead &measurement) {
  if (!tracker.isInitialized()) {
    ROS_ERROR("obstacle tracker not initialized but received ir measurement");
    return false;
  }
  const bool obstacle_is_in_front_of_tof_sensor =
      (measurement.tof_pose.inverse() * to3D(getEstimatedPosition())).x() > 0;
  return obstacle_is_in_front_of_tof_sensor;
}

void Obstacle::newTOFMeasurement(const TOFMeasurement &measurement) {
  tracker.correctTOFMeasurement(measurement);
}

void Obstacle::newTOFMeasurementAhead(const TOFMeasurementAhead &measurement) {
  tracker.correctTOFMeasurementAhead(measurement);
}

bool Obstacle::stoppedPrediction() const { return tracker.stopPrediction(); }

long Obstacle::getNumberUpdatesWithoutNewMeasurement() const {
  return tracker.getNumberUpdatesWithoutNewMeasurement();
}

double Obstacle::getCertainty() const { return msg.certainty; }

Eigen::Vector2d Obstacle::getEstimatedPosition() const {
  return ObstacleTracker::verticesToCenter(msg);
}
}  // namespace environmental_model
