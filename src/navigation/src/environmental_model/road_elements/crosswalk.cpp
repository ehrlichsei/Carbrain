#include "crosswalk.h"
#include "../road_element_visitors/road_element_visitor.h"
#include "common/best_score.h"
namespace environmental_model {
Crosswalk::Crosswalk(const perception_msgs::Crosswalk& crosswalk)
    : msg(crosswalk) {}

void Crosswalk::accept(RoadElementVisitor& visitor, TrackingElement& tracking_element) {
  visitor.visit(*this, tracking_element);
}

void Crosswalk::newMsg(const MsgCollection& msg, const Eigen::Affine3d& vehicle_pose) {
  if (msg.crosswalks.sub_messages.empty()) {
    return;
  }
  const double dist_to_vehicle =
      (vehicle_pose.translation() - getPose().translation()).norm();
  if (dist_to_vehicle < 0.6 && this->msg.certainty > 0.2) {
    return;
  }
  const auto certaintyAsScore =
      [](const auto& message) { return message.certainty; };
  const auto highest_prob_msg =
      *common::max_score(msg.crosswalks.sub_messages, certaintyAsScore);
  if (highest_prob_msg.certainty > 0.2 || this->msg.certainty < highest_prob_msg.certainty) {
    this->msg = highest_prob_msg;
    setBaseAreaIfNotEmpty(highest_prob_msg.base_hull_polygon);
  }
}

void Crosswalk::collectMsgs(MsgCollection& collection) {
  collection.crosswalks.sub_messages.push_back(msg);
}

void Crosswalk::setProbabilityFromLastMsg(RoadElement::ProbabilityVector& probabilities) {
  if (msg.certainty > probabilities[RoadElementType::Crosswalk]) {
    probabilities[RoadElementType::Crosswalk] = msg.certainty;
  }
}

Eigen::Affine3d Crosswalk::getPose() {
  Eigen::Affine3d pose;
  tf2::fromMsg(msg.pose, pose);
  return pose;
}

perception_msgs::Crosswalk Crosswalk::getMsg() const { return msg; }

bool Crosswalk::hasPedestrianWaiting() const {
  return pedestrian_waiting_score_ > 0.4;
}

double Crosswalk::getPedestrianWaitingScore() const {
  return pedestrian_waiting_score_;
}

void Crosswalk::setPedestrianWaitingScore(double pedestrian_waiting_score) {
  pedestrian_waiting_score_ = pedestrian_waiting_score;
}

double Crosswalk::getCertainty() const { return msg.certainty; }
}  // namespace environmental_model
