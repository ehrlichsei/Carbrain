#include "speed_limit.h"
#include "../road_element_visitors/road_element_visitor.h"
#include "common/best_score.h"
namespace environmental_model {
SpeedLimit::SpeedLimit(const perception_msgs::SpeedLimitMarking &speed_limit)
    : msg(speed_limit) {}

SpeedLimit::SpeedLimit(const std::shared_ptr<RoadElementParameter> &parameter)
    : parameter_(parameter) {}

void SpeedLimit::accept(RoadElementVisitor &visitor, TrackingElement &tracking_element) {
  visitor.visit(*this, tracking_element);
}

void SpeedLimit::newMsg(const MsgCollection &msg, const Eigen::Affine3d & /*vehicle_pose*/) {
  if (msg.speed_limits.sub_messages.empty()) {
    return;
  }
  const auto certaintyAsScore =
      [](const auto &message) { return message.certainty; };
  const auto highest_prob_msg =
      *common::max_score(msg.speed_limits.sub_messages, certaintyAsScore);
  if (highest_prob_msg.certainty > 0.2 || this->msg.certainty < highest_prob_msg.certainty) {
    assert(highest_prob_msg.speed_limit % 10 == 0);
    assert(highest_prob_msg.speed_limit < 100 && highest_prob_msg.speed_limit > 0);

    if (parameter_ == nullptr) {
      ROS_ERROR("speed_limit: parameter_ is not instanciated");
      return;
    }

    const size_t saved_speed_limit_msgs_count = parameter_->saved_speed_limit_msgs_count;
    double alpha = parameter_->alpha;

    Eigen::Matrix<float, 10, 1> current_certainties, previous_certainties, merged_certainties;
    current_certainties = Eigen::Matrix<float, 10, 1>::Zero();
    previous_certainties = Eigen::Matrix<float, 10, 1>::Zero();
    merged_certainties = Eigen::Matrix<float, 10, 1>::Zero();

    for (const auto &sub_message : msg.speed_limits.sub_messages) {
      int sub_message_speed_limit = sub_message.speed_limit;
      if (sub_message.limit_relieved && current_certainties(0, 0) < sub_message.certainty) {
        current_certainties(0, 0) = sub_message.certainty;
      } else if (current_certainties(sub_message_speed_limit / 10, 0) < sub_message.certainty) {
        current_certainties(sub_message_speed_limit / 10, 0) = sub_message.certainty;
      }
    }
    if (!previous_certainties_queu.empty()) {
      for (const auto &vec : previous_certainties_queu) {
        previous_certainties += vec;
      }
    }

    merged_certainties = current_certainties + previous_certainties;

    this->msg = highest_prob_msg;

    std::ptrdiff_t i;
    merged_certainties.maxCoeff(&i);
    int max_index = i;

    switch (max_index) {
      case 0:
        this->msg.limit_relieved = true;
        break;
      default:
        this->msg.speed_limit = max_index * 10;
        this->msg.limit_relieved = false;
        break;
    }

    previous_certainties_queu.push_front(current_certainties);

    for (auto &vec : previous_certainties_queu) {
      vec = alpha * vec;
    }

    if (previous_certainties_queu.size() > saved_speed_limit_msgs_count) {
      previous_certainties_queu.pop_back();
    }

    setBaseAreaIfNotEmpty(highest_prob_msg.base_hull_polygon);
  }
}

void SpeedLimit::collectMsgs(MsgCollection &collection) {
  collection.speed_limits.sub_messages.push_back(msg);
}

void SpeedLimit::setProbabilityFromLastMsg(RoadElement::ProbabilityVector &probabilities) {
  if (msg.certainty > probabilities[RoadElementType::SpeedLimit]) {
    probabilities[RoadElementType::SpeedLimit] = msg.certainty;
  }
}

perception_msgs::SpeedLimitMarking SpeedLimit::getMsg() const { return msg; }

double SpeedLimit::getCertainty() const { return msg.certainty; }
}  // namespace environmental_model
