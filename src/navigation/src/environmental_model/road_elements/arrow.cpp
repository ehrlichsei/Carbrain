#include "arrow.h"
#include "../road_element_visitors/road_element_visitor.h"
#include "common/best_score.h"
namespace environmental_model {
Arrow::Arrow(const perception_msgs::ArrowMarking& arrow_marking)
    : msg(arrow_marking) {}

void Arrow::accept(RoadElementVisitor& visitor, TrackingElement& tracking_element) {
  visitor.visit(*this, tracking_element);
}


void Arrow::newMsg(const MsgCollection& msg, const Eigen::Affine3d& /*vehicle_pose*/) {
  if (msg.arrow_markings.sub_messages.empty()) {
    return;
  }
  const auto certaintyAsScore =
      [](const auto& message) { return message.certainty; };
  const auto highest_prob_msg =
      *common::max_score(msg.arrow_markings.sub_messages, certaintyAsScore);
  if (highest_prob_msg.certainty > 0.2 || this->msg.certainty < highest_prob_msg.certainty) {
    this->msg = highest_prob_msg;
    setBaseAreaIfNotEmpty(highest_prob_msg.base_hull_polygon);
  }
}

void Arrow::collectMsgs(MsgCollection& collection) {
  collection.arrow_markings.sub_messages.push_back(msg);
}

void Arrow::setProbabilityFromLastMsg(RoadElement::ProbabilityVector& probabilities) {
  if (msg.certainty > probabilities[RoadElementType::ArrowMarking]) {
    probabilities[RoadElementType::ArrowMarking] = msg.certainty;
  }
}

double Arrow::getCertainty() const { return msg.certainty; }

Eigen::Affine3d Arrow::getPose() const {
  Eigen::Affine3d pose;
  tf2::fromMsg(msg.pose, pose);
  return pose;
}

perception_msgs::ArrowMarking Arrow::getMsg() const { return msg; }

}  // namespace environmental_model
