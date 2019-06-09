#include "speed_controller_node_debug.h"
#include <common/macros.h>

THIRD_PARTY_HEADERS_BEGIN
#include "control/SpeedControllerConfig.h"
#include <common_msgs/Float32Stamped.h>
THIRD_PARTY_HEADERS_END

#include "common/node_creation_makros.h"
#include "common/ros_chrono_conversions.h"

SpeedControllerNodeDebug::SpeedControllerNodeDebug(ros::NodeHandle &node_handle)
    : SpeedControllerNode(node_handle) {
  speed_controller_ = std::make_unique<SpeedController>(&parameter_handler_);
  parameter_handler_.addDynamicReconfigureServer<control::SpeedControllerConfig>(node_handle_);
}

void SpeedControllerNodeDebug::startModule() {
  SpeedControllerNode::startModule();
  state_estimation_subscriber_ = node_handle_.subscribe(
      "state_estimation", 5, &SpeedControllerNodeDebug::handleState, this);
}

void SpeedControllerNodeDebug::stopModule() {
  SpeedControllerNode::stopModule();
  state_estimation_subscriber_.shutdown();
}


void SpeedControllerNodeDebug::handleState(const state_estimation_msgs::State &msg) {

  speed_controller_->setParams();
  const auto time_since_last_measurement = msg.header.stamp - time_of_last_measurement;
  const SpeedController::SpeedControllerCommands speed_controller_commands =
      speed_controller_->calcSpeedCommands(
          common::toBoost(time_since_last_measurement), msg.speed_x, false);

  speed_controller_->engine_power_publisher_queue_.push(speed_controller_commands.engine_power);
  speed_controller_->speed_error_publisher_queue_.push(speed_controller_commands.speed_error);

  time_of_last_measurement = msg.header.stamp;
}
