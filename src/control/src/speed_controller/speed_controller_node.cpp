#include "speed_controller_node.h"
#include <common/macros.h>

THIRD_PARTY_HEADERS_BEGIN
#include <common/node_creation_makros.h>
#include <common_msgs/Float32Stamped.h>
#include "control/SpeedControllerConfig.h"
THIRD_PARTY_HEADERS_END

#include "common/node_creation_makros.h"
#include "common/realtime_timings.h"
#include "speed_controller.h"
#include "speed_controller_node_debug.h"

SpeedControllerNode::SpeedControllerNode(ros::NodeHandle &node_handle)
    : NodeBase(node_handle) {}


// In general it is a very good practice to have not-throwing destructors.
// In this special case it does not really matter though, because this
// destructor should only be called during shutdown. If boost::thread throws
// an exception the operating system will take care of the remains and clean
// them up.
// NOLINTNEXTLINE(bugprone-exception-escape)
SpeedControllerNode::~SpeedControllerNode() {
  SpeedControllerNode::stopModule();
}

SpeedControllerNodeRealtime::SpeedControllerNodeRealtime(ros::NodeHandle &node_handle)
    : SpeedControllerNode(node_handle) {
  speed_controller_ = std::make_unique<SpeedControllerRealtime>(&parameter_handler_);
  parameter_handler_.addDynamicReconfigureServer<control::SpeedControllerConfig>(node_handle_);
}

void SpeedControllerNodeRealtime::startModule() {
  SpeedControllerNode::startModule();
}

void SpeedControllerNodeRealtime::stopModule() {
  SpeedControllerNode::stopModule();
}

void SpeedControllerNode::startModule() {
  velocity_command_subscriber_ =
      node_handle_.subscribe("velocity_command",
                             1,
                             &SpeedControllerNode::onVelocityCommandReceived,
                             this,
                             common::noDelayTransport());
  emergency_stop_command_subscriber_ = node_handle_.subscribe(
      "emergency_stop", 1, &SpeedControllerNode::onEmergencyStopReceived, this);

  speed_controller_->setParams();
  speed_controller_->start();

  engine_power_publisher_ = node_handle_.advertise<common_msgs::Float32Stamped>(
      "debug/engine_power", 5, true);
  speed_error_publisher_ = node_handle_.advertise<common_msgs::Float32Stamped>(
      "debug/speed_error", 5, true);

  reset_service_server_ = node_handle_.advertiseService(
      "reset", &SpeedControllerNode::resetCallback, this);

  auto_reset_subscriber_ = node_handle_.subscribe(
      "auto_reset", 1, &SpeedControllerNode::autoResetCallback, this);

  try {
    debug_publisher_thread_ =
        boost::thread(&SpeedControllerNode::debugPublisherLoop, this);
  } catch (const boost::thread_resource_error &e) {
    ROS_ERROR(
        "unable to launch debug publisher threads in speed controller: \n %s", e.what());
  }
}

void SpeedControllerNode::stopModule() {
  debug_publisher_thread_.interrupt();

  if (debug_publisher_thread_.joinable()) {
    debug_publisher_thread_.join();
  }

  reset_service_server_.shutdown();
  auto_reset_subscriber_.shutdown();

  engine_power_publisher_.shutdown();
  speed_error_publisher_.shutdown();

  speed_controller_->stop();
  speed_controller_->reset();
  velocity_command_subscriber_.shutdown();
  emergency_stop_command_subscriber_.shutdown();
}

bool SpeedControllerNode::resetCallback(std_srvs::EmptyRequest &, std_srvs::EmptyResponse &) {
  speed_controller_->reset();
  ROS_INFO("reset speed controller");
  return true;
}

void SpeedControllerNode::autoResetCallback(UNUSED const std_msgs::Empty &auto_reset) {
  speed_controller_->reset();
  ROS_WARN("auto reset");
}

const std::string SpeedControllerNode::getName() {
  return std::string("speed_controller");
}

void SpeedControllerNode::onVelocityCommandReceived(
    const longitudinal_controller_msgs::DrivingSpeedConstPtr &msg) {
  if (RealtimeTimings::LOWLEVEL_CONTROLLER_WAIT_TIME != 0) {
    const int delay = (ros::Time::now() - msg->header.stamp).toNSec() / 1000;
    if (delay > RealtimeTimings::LOWLEVEL_CONTROLLER_WAIT_TIME +
                    RealtimeTimings::STATE_ESTIMATION_CYCLE_TIME) {
      ROS_WARN_THROTTLE(1, "delay (measure->HLC set point) is too big: %dus!", delay);
    }
  }
  speed_controller_->setTargetSpeed(msg->speed, msg->header.stamp);
}

void SpeedControllerNode::onEmergencyStopReceived(std_msgs::Bool msg) {
  speed_controller_->setEmergencyStop(msg.data);
}

void SpeedControllerNode::debugPublisherLoop() {

  while (!boost::this_thread::interruption_requested()) {

    double engine_power = 0.0;
    if (speed_controller_->getNextEnginePower(&engine_power)) {
      common_msgs::Float32Stamped msg;
      msg.header.stamp = ros::Time::now();
      msg.data = engine_power;
      engine_power_publisher_.publish(msg);
    }

    double speed_error = 0.0;
    if (speed_controller_->getNextSpeedError(&speed_error)) {
      common_msgs::Float32Stamped msg;
      msg.header.stamp = ros::Time::now();
      msg.data = speed_error;
      speed_error_publisher_.publish(msg);
    }
  }
}

CREATE_NODE_WITH_FANCY_DEBUG(SpeedControllerNodeRealtime, SpeedControllerNodeDebug)
