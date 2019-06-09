#include "steering_controller_node_debug.h"

#include <common/macros.h>

THIRD_PARTY_HEADERS_BEGIN
#include "control/SteeringControllerConfig.h"
#include <common_msgs/Float32Stamped.h>
THIRD_PARTY_HEADERS_END

#include <common/realtime_channel_ids.h>
#include <common/ros_chrono_conversions.h>

SteeringControllerNodeDebug::SteeringControllerNodeDebug(ros::NodeHandle& node_handle)
    : SteeringControllerNode(node_handle) {
  steering_controller_ = std::make_unique<SteeringController>(&parameter_handler_);
  parameter_handler_.addDynamicReconfigureServer<control::SteeringControllerConfig>(node_handle_);
}

void SteeringControllerNodeDebug::startModule() {
  SteeringControllerNode::startModule();
  state_estimation_subscriber_ =
      node_handle_.subscribe<const state_estimation_msgs::State&, SteeringControllerNodeDebug>(
          "state_estimation", 5, &SteeringControllerNodeDebug::handleState, this);
}
void SteeringControllerNodeDebug::handleState(const state_estimation_msgs::State& msg) {

  steering_controller_->setParams();
  const auto time_since_last_measurement = msg.header.stamp - time_of_last_measurement;
  const SteeringController::SteeringControlCommands steering_control_commands =
      steering_controller_->calcSteeringCommands(
          common::toBoost(time_since_last_measurement),
          controls_rear_axle ? msg.steering_angle_back : msg.steering_angle_front,
          false);

  steering_controller_->servo_set_value_publisher_queue_.push(
      steering_control_commands.steering_output);

  if (steering_control_commands.angle_error != boost::none) {
    steering_controller_->angle_error_publisher_queue_.push(
        *(steering_control_commands.angle_error));
  }

  time_of_last_measurement = msg.header.stamp;
}

void SteeringControllerNodeDebug::stopModule() {
  SteeringControllerNode::stopModule();
  state_estimation_subscriber_.shutdown();
}
