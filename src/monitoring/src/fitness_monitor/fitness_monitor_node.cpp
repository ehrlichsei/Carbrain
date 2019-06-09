#include "fitness_monitor_node.h"
#include <common/macros.h>

THIRD_PARTY_HEADERS_BEGIN
#include <tf2_ros/buffer.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <lateral_controller_msgs/DrivingError.h>
#include <monitoring/FitnessMonitorConfig.h>
THIRD_PARTY_HEADERS_END

#include "common/node_creation_makros.h"

FitnessMonitorNode::FitnessMonitorNode(ros::NodeHandle &node_handle)
    : NodeBase(node_handle),
      tf2_listener_(tf2_buffer_),
      fitness_monitor_(&parameter_handler_, &fitness_calculator_),
      round_detection_(&parameter_handler_, &fitness_monitor_) {
  parameter_handler_.addDynamicReconfigureServer<monitoring::FitnessMonitorConfig>(node_handle_);
}

void FitnessMonitorNode::startModule() {
  state_estimation_subscriber_ = node_handle_.subscribe(
      "state_estimation", 100, &FitnessMonitorNode::stateEstimationCallback, this);
  lateral_error_subscriber_ = node_handle_.subscribe(
      "lateral_error", 100, &FitnessMonitorNode::lateralErrorCallback, this);
  start_line_subscriber_ = node_handle_.subscribe(
      "start_line", 100, &FitnessMonitorNode::startlineCallback, this);
}

void FitnessMonitorNode::stopModule() {
  state_estimation_subscriber_.shutdown();
  lateral_error_subscriber_.shutdown();
  start_line_subscriber_.shutdown();
}

void FitnessMonitorNode::stateEstimationCallback(const state_estimation_msgs::State::ConstPtr &state_msg) {
  fitness_calculator_.addMotionMeasurement(state_msg->header.stamp,
                                           state_msg->acceleration,
                                           state_msg->speed_x,
                                           state_msg->speed_y,
                                           state_msg->yaw_rate);
  round_detection_.updateVehiclePosition(getVehicleToWorldTransformation(ros::Time::now()));
}

void FitnessMonitorNode::lateralErrorCallback(const lateral_controller_msgs::DrivingError error) {
  fitness_calculator_.addLateralControlError(error.error);
}

void FitnessMonitorNode::startlineCallback(const perception_msgs::StartLines::ConstPtr &lines_msg) {
  if (lines_msg->sub_messages.empty()) {
    return;
  }

  geometry_msgs::PoseStamped pose;
  pose.pose = lines_msg->sub_messages.front().pose;
  pose.header = lines_msg->sub_messages.front().header;
  tf2::Stamped<Eigen::Affine3d> transform_pose;
  tf2::fromMsg(pose, transform_pose);
  Eigen::Affine3d start_line =
      getVehicleToWorldTransformation(lines_msg->header.stamp) * transform_pose;
  round_detection_.updateStartlinePosition(start_line);
}

Eigen::Affine3d FitnessMonitorNode::getVehicleToWorldTransformation(const ros::Time& transform_time) const {
  geometry_msgs::TransformStamped vehicle_to_world_transform_message;
  try {
    vehicle_to_world_transform_message = tf2_buffer_.lookupTransform(
        "world", "vehicle", transform_time, ros::Duration(0.1));
  } catch (tf2::TransformException &ex) {
    ROS_WARN("Can NOT transform vehicle to world: %s", ex.what());
    return Eigen::Affine3d();
  }

  return tf2::transformToEigen(vehicle_to_world_transform_message);
}

const std::string FitnessMonitorNode::getName() {
  return std::string("fitness_monitor");
}

CREATE_NODE(FitnessMonitorNode)
