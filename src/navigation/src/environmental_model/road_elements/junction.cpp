#include "junction.h"
#include "../road_element_visitors/road_element_visitor.h"
#include "common/best_score.h"
namespace environmental_model {
Junction::Junction(const perception_msgs::Junction &junction) : msg(junction) {}

void Junction::accept(RoadElementVisitor &visitor, TrackingElement &tracking_element) {
  visitor.visit(*this, tracking_element);
}

void Junction::newMsg(const MsgCollection &msg, const Eigen::Affine3d &vehicle_pose) {
  if (msg.junctions.sub_messages.empty()) {
    return;
  }
  const double dist_to_vehicle =
      (vehicle_pose.translation() - getPose().translation()).norm();
  if (dist_to_vehicle < 0.6 && this->msg.certainty > 0.2) {
    return;
  }
  const auto certaintyAsScore =
      [](const auto &message) { return message.certainty; };
  const auto highest_prob_msg =
      *common::max_score(msg.junctions.sub_messages, certaintyAsScore);
  if (highest_prob_msg.certainty > 0.2 || this->msg.certainty < highest_prob_msg.certainty) {
    this->msg = highest_prob_msg;
    setBaseAreaIfNotEmpty(highest_prob_msg.base_hull_polygon);
    Eigen::Vector3d msg_position(0, 0, 0);
    tf2::fromMsg(highest_prob_msg.pose.position, msg_position);
    updateStoppingPoint(msg_position, vehicle_pose);
  }
}
void Junction::collectMsgs(MsgCollection &collection) {
  collection.junctions.sub_messages.push_back(msg);
}

void Junction::setProbabilityFromLastMsg(RoadElement::ProbabilityVector &probabilities) {
  if (msg.certainty > probabilities[RoadElementType::Junction]) {
    probabilities[RoadElementType::Junction] = msg.certainty;
  }
}

Eigen::Affine3d Junction::getPose() const {
  Eigen::Affine3d pose;
  tf2::fromMsg(msg.pose, pose);
  return pose;
}

void Junction::merge(const Junction &junction, const Eigen::Affine3d &vehicle_pose) {
  updateStoppingPoint(junction.stopping_point, vehicle_pose);
}

perception_msgs::Junction Junction::getMsg() const { return msg; }

bool Junction::typeRightSide() const {
  return msg.junction_type == perception_msgs::Junction::TYPE_STOPLINE_RIGHT ||
         msg.junction_type == perception_msgs::Junction::TYPE_GIVEWAY_RIGHT;
}

bool Junction::getObstacleWaiting() const {
  return obstacle_waiting_score_ > 0.2;
}


double Junction::getObstacleWaitingScore() const {
  return obstacle_waiting_score_;
}

void Junction::setObstacleWaitingScore(double obstacle_waiting_score) {
  obstacle_waiting_score_ = obstacle_waiting_score;
}

double Junction::getCertainty() const { return msg.certainty; }

void Junction::updateStoppingPoint(const Eigen::Vector3d &msg_position,
                                   const Eigen::Affine3d &vehicle_pose) {
  if (stopping_point == Eigen::Vector3d::Zero() ||
      (vehicle_pose.translation() - msg_position).norm() <
          (vehicle_pose.translation() - stopping_point).norm()) {
    stopping_point = msg_position;
  }
}

Eigen::Vector3d Junction::getStoppingPoint() const { return stopping_point; }
}  // namespace environmental_model
