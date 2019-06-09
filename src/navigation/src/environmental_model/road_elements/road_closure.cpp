#include "road_closure.h"
#include "../road_element_visitors/road_element_visitor.h"
#include <numeric>
#include "common/pca_eigen.h"
#include <tf2_eigen/tf2_eigen.h>
#include "common/best_score.h"
namespace environmental_model {
RoadClosure::RoadClosure(const perception_msgs::RoadClosure &road_closure)
    : msg(road_closure) {}

RoadClosure::RoadClosure(const std::shared_ptr<RoadElementParameter> &parameter)
    : parameter_(parameter) {}

void RoadClosure::accept(RoadElementVisitor &visitor, TrackingElement &tracking_element) {
  visitor.visit(*this, tracking_element);
}

void RoadClosure::newMsg(const MsgCollection &msg, const Eigen::Affine3d &vehicle_pose) {
  if (msg.road_closures.sub_messages.empty()) {
    return;
  }
  const auto certaintyAsScore =
      [](const auto &message) { return message.certainty; };
  const auto highest_prob_msg =
      *common::max_score(msg.road_closures.sub_messages, certaintyAsScore);
  if (highest_prob_msg.certainty > 0.2 || this->msg.certainty < highest_prob_msg.certainty) {
    // Check if this->msg is null or invalid
    if (this->msg.certainty == 0 || this->msg.hull_polygon.size() != 4) {
      this->msg = highest_prob_msg;
      this->msg.base_hull_polygon = this->msg.hull_polygon;
      setBaseAreaIfNotEmpty(this->msg.base_hull_polygon);
      return;
    }

    if (parameter_ == nullptr) {
      ROS_ERROR("road_closure: parameter_ is not instanciated");
      return;
    }
    const double road_closure_start_distance_param = parameter_->road_closure_start_distance;
    const double road_closure_end_distance_param = parameter_->road_closure_end_distance;
    const double min_assumed_road_closure_length_param =
        parameter_->min_assumed_road_closure_length;

    Eigen::Vector3d previous_start;
    tf2::fromMsg(this->msg.hull_polygon[0], previous_start);
    Eigen::Vector3d previous_end;
    tf2::fromMsg(this->msg.hull_polygon[3], previous_end);

    Eigen::Vector3d previous_start_vehicle_coordinates = vehicle_pose.inverse() * previous_start;
    Eigen::Vector3d previous_end_vehicle_coordinates = vehicle_pose.inverse() * previous_end;

    Eigen::Vector3d first_left;
    tf2::fromMsg(this->msg.hull_polygon[1], first_left);
    Eigen::Vector3d second_left;
    tf2::fromMsg(this->msg.hull_polygon[2], second_left);

    double road_closure_length = (second_left - first_left).norm();

    if (previous_start_vehicle_coordinates.x() > road_closure_start_distance_param) {
      this->msg.hull_polygon[0] = highest_prob_msg.hull_polygon[0];
      this->msg.hull_polygon[1] = highest_prob_msg.hull_polygon[1];
    }
    if (previous_end_vehicle_coordinates.x() > road_closure_end_distance_param ||
        road_closure_length < min_assumed_road_closure_length_param) {
      this->msg.hull_polygon[2] = highest_prob_msg.hull_polygon[2];
      this->msg.hull_polygon[3] = highest_prob_msg.hull_polygon[3];
    }
    this->msg.base_hull_polygon = this->msg.hull_polygon;
    setBaseAreaIfNotEmpty(this->msg.base_hull_polygon);
  }
}

void RoadClosure::collectMsgs(MsgCollection &collection) {
  if (msg.hull_polygon.empty()) {
    ROS_ERROR("empty hull polygon in collectMsgs road closure");
    return;
  }
  collection.road_closures.sub_messages.push_back(msg);
}

void RoadClosure::setProbabilityFromLastMsg(RoadElement::ProbabilityVector &probabilities) {
  if (msg.certainty > probabilities[RoadElementType::RoadClosure]) {
    probabilities[RoadElementType::RoadClosure] = msg.certainty;
  }
}

perception_msgs::RoadClosure RoadClosure::getMsg() const { return msg; }

Eigen::Affine3d RoadClosure::getPose() const {
  if (msg.hull_polygon.size() != 4) {
    ROS_ERROR_STREAM_THROTTLE(1,
                              "road closure hull polygon has "
                                  << msg.hull_polygon.size()
                                  << " instead of 4");
    return Eigen::Affine3d::Identity();
  }
  Eigen::Vector3d start = getStart();
  Eigen::Vector3d end = getEnd();
  const Eigen::Affine3d pose =
      Eigen::Translation3d(getStart()) *
      Eigen::AngleAxisd(
          common::getAngle((end - start).topRows<2>(), Eigen::Vector2d::UnitX()),
          Eigen::Vector3d::UnitZ());
  return pose;
}

Eigen::Vector3d RoadClosure::getStart() const {
  if (msg.hull_polygon.size() != 4) {
    ROS_ERROR_STREAM_THROTTLE(1,
                              "road closure hull polygon has "
                                  << msg.hull_polygon.size()
                                  << " instead of 4");
    return Eigen::Vector3d::Zero();
  }
  Eigen::Vector3d start_road_closure = Eigen::Vector3d::Zero();
  tf2::fromMsg(msg.hull_polygon.front(), start_road_closure);
  return start_road_closure;
}

Eigen::Vector3d RoadClosure::getEnd() const {
  if (msg.hull_polygon.size() != 4) {
    ROS_ERROR_STREAM_THROTTLE(1,
                              "road closure hull polygon has "
                                  << msg.hull_polygon.size()
                                  << " instead of 4");
    return Eigen::Vector3d::Zero();
  }
  Eigen::Vector3d end_road_closure = Eigen::Vector3d::Zero();
  tf2::fromMsg(msg.hull_polygon.back(), end_road_closure);
  return end_road_closure;
}

double RoadClosure::getWidth() const {
  if (msg.hull_polygon.size() != 4) {
    ROS_ERROR_STREAM_THROTTLE(1,
                              "road closure hull polygon has "
                                  << msg.hull_polygon.size()
                                  << " instead of 4");
    return 0;
  }
  Eigen::Vector3d start;
  tf2::fromMsg(msg.hull_polygon[0], start);
  Eigen::Vector3d first_left;
  tf2::fromMsg(msg.hull_polygon[1], first_left);
  Eigen::Vector3d end;
  tf2::fromMsg(msg.hull_polygon[3], end);
  Eigen::Vector3d second_left;
  tf2::fromMsg(msg.hull_polygon[2], second_left);
  Eigen::Vector3d direction((end - start).unitOrthogonal());
  const double width_first = std::abs((start - first_left).dot(direction));
  const double width_second = std::abs((end - second_left).dot(direction));
  return (width_first + width_second) / 2.0;
}

bool RoadClosure::getObstacleWaiting() const {
  return obstacle_waiting_score_ > 0.2;
}

double RoadClosure::getObstacleWaitingScore() const {
  return obstacle_waiting_score_;
}

void RoadClosure::setObstacleWaitingScore(double obstacle_waiting_score) {
  obstacle_waiting_score_ = obstacle_waiting_score;
}

double RoadClosure::getCertainty() const { return msg.certainty; }
}  // namespace environmental_model
