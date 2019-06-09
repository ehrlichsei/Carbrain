#include "start_line.h"
#include "../road_element_visitors/road_element_visitor.h"
#include "common/best_score.h"

namespace environmental_model {
StartLine::StartLine(const perception_msgs::StartLine &start_line)
    : msg(start_line) {}

void StartLine::accept(RoadElementVisitor &visitor, TrackingElement &tracking_element) {
  visitor.visit(*this, tracking_element);
}

void StartLine::newMsg(const MsgCollection &msg, const Eigen::Affine3d & /*vehicle_pose*/) {
  if (msg.start_lines.sub_messages.empty()) {
    return;
  }
  const auto certaintyAsScore =
      [](const auto &message) { return message.certainty; };
  const auto highest_prob_msg =
      *common::max_score(msg.start_lines.sub_messages, certaintyAsScore);
  if (highest_prob_msg.certainty > 0.2 || this->msg.certainty < highest_prob_msg.certainty) {
    this->msg = highest_prob_msg;
    setBaseAreaIfNotEmpty(highest_prob_msg.base_hull_polygon);
  }
}

void StartLine::collectMsgs(MsgCollection &collection) {
  collection.start_lines.sub_messages.push_back(msg);
}

void StartLine::setProbabilityFromLastMsg(RoadElement::ProbabilityVector &probabilities) {
  if (msg.certainty > probabilities[RoadElementType::StartLine]) {
    probabilities[RoadElementType::StartLine] = msg.certainty;
  }
  if (msg.certainty <= 0.06 && msg.certainty > probabilities[RoadElementType::Junction]) {
    probabilities[RoadElementType::Junction] = msg.certainty;
  }
}

perception_msgs::StartLine StartLine::getMsg() const { return msg; }

double StartLine::getCertainty() const { return msg.certainty; }

Eigen::Affine3d StartLine::getPose() const {
  Eigen::Affine3d pose;
  tf2::fromMsg(msg.pose, pose);
  return pose;
}
}  // namespace environmental_model
