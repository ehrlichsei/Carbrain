#include "unidentified.h"
#include "../road_element_visitors/road_element_visitor.h"
#include "common/best_score.h"
namespace environmental_model {
Unidentified::Unidentified(const perception_msgs::Unidentified &unidentified)
    : msg(unidentified) {}

void Unidentified::accept(RoadElementVisitor &visitor, TrackingElement &tracking_element) {
  visitor.visit(*this, tracking_element);
}

void Unidentified::newMsg(const MsgCollection &msg, const Eigen::Affine3d & /*vehicle_pose*/) {
  if (msg.unidentifieds.sub_messages.empty()) {
    return;
  }
  const auto certaintyAsScore =
      [](const auto &message) { return message.certainty; };
  const auto highest_prob_msg =
      *common::max_score(msg.unidentifieds.sub_messages, certaintyAsScore);
  if (highest_prob_msg.certainty > 0.2 || this->msg.certainty < highest_prob_msg.certainty) {
    this->msg = highest_prob_msg;
    setBaseAreaIfNotEmpty(highest_prob_msg.base_hull_polygon);
  }
}

void Unidentified::collectMsgs(MsgCollection &collection) {
  collection.unidentifieds.sub_messages.push_back(msg);
}

void Unidentified::setProbabilityFromLastMsg(RoadElement::ProbabilityVector &probabilities) {
  if (msg.certainty > probabilities[RoadElementType::Unidentified]) {
    probabilities[RoadElementType::Unidentified] = msg.certainty;
  }
}

perception_msgs::Unidentified Unidentified::getMsg() const { return msg; }

double Unidentified::getCertainty() const { return msg.certainty; }
}  // namespace environmental_model
