#include "steering_controller_node.h"
#include <common/macros.h>

THIRD_PARTY_HEADERS_BEGIN
#include <common_msgs/Float32Stamped.h>
#include "control/SteeringControllerConfig.h"
THIRD_PARTY_HEADERS_END

#include "steering_controller_node_debug.h"

#include "common/node_creation_makros.h"

#include <common/realtime_channel_ids.h>
#include <common/realtime_timings.h>


SteeringControllerNode::SteeringControllerNode(ros::NodeHandle& node_handle)
    : NodeBase(node_handle),
      controls_rear_axle(node_handle.param<bool>("back_steering_controller", false)) {
  if (controls_rear_axle) {
    ROS_INFO("this steering controller controls the rear axle");
  } else {
    ROS_INFO("this steering controller controls the front axle");
  }
}

// In general it is a very good practice to have not-throwing destructors.
// In this special case it does not really matter though, because this
// destructor should only be called during shutdown. If boost::thread throws
// an exception the operating system will take care of the remains and clean
// them up.
// NOLINTNEXTLINE(bugprone-exception-escape)
SteeringControllerNode::~SteeringControllerNode() {
  SteeringControllerNode::stopModule();
}

SteeringControllerNodeRealtime::SteeringControllerNodeRealtime(ros::NodeHandle& node_handle)
    : SteeringControllerNode(node_handle) {
  steering_controller_ = std::make_unique<SteeringControllerRealtime>(
      &parameter_handler_,
      controls_rear_axle ? CHANNEL_ID_STEERING_BACK_SERVO_OUTPUT : CHANNEL_ID_STEERING_SERVO_OUTPUT,
      controls_rear_axle ? CHANNEL_ID_STEERING_ANGLE_MEASURE_BACK
                         : CHANNEL_ID_STEERING_ANGLE_MEASURE_FRONT);
  parameter_handler_.addDynamicReconfigureServer<control::SteeringControllerConfig>(node_handle_);
}

void SteeringControllerNodeRealtime::startModule() {
  SteeringControllerNode::startModule();
}

void SteeringControllerNodeRealtime::stopModule() {
  SteeringControllerNode::stopModule();
}


void SteeringControllerNode::startModule() {
  steering_angle_command_subscriber_ =
      node_handle_.subscribe("steering_angle_command",
                             1,
                             &SteeringControllerNode::onSteeringCommandReceived,
                             this,
                             common::noDelayTransport());
  steering_controller_->setParams();
  steering_controller_->start();

  servo_set_value_publisher_ = node_handle_.advertise<common_msgs::Float32Stamped>(
      "debug/servo_set_value", 5, true);
  angle_error_publisher_ = node_handle_.advertise<common_msgs::Float32Stamped>(
      "debug/angle_error", 5, true);

  auto_reset_subscriber_ = node_handle_.subscribe(
      "auto_reset", 1, &SteeringControllerNode::autoResetCallback, this);

  try {
    servo_set_value_publisher_thread_ =
        boost::thread(&SteeringControllerNode::servoSetValuePublisherLoop, this);
    angle_error_publisher_thread_ =
        boost::thread(&SteeringControllerNode::angleErrorPublisherLoop, this);
  } catch (const boost::thread_resource_error& e) {
    ROS_ERROR(
        "unable to launch debug publisher threads in steering controller: \n "
        "%s",
        e.what());
  }
}

void SteeringControllerNode::stopModule() {
  servo_set_value_publisher_thread_.interrupt();
  angle_error_publisher_thread_.interrupt();

  if (servo_set_value_publisher_thread_.joinable()) {
    servo_set_value_publisher_thread_.join();
  }
  if (angle_error_publisher_thread_.joinable()) {
    angle_error_publisher_thread_.join();
  }

  auto_reset_subscriber_.shutdown();

  servo_set_value_publisher_.shutdown();
  angle_error_publisher_.shutdown();

  steering_controller_->stop();
  steering_controller_->reset();
}

void SteeringControllerNode::autoResetCallback(UNUSED const std_msgs::Empty& auto_reset) {
  steering_controller_->reset();
  ROS_WARN("auto reset");
}

void SteeringControllerNode::onSteeringCommandReceived(
    const lateral_controller_msgs::DrivingSteeringAngleConstPtr& msg) {
  if (RealtimeTimings::LOWLEVEL_CONTROLLER_WAIT_TIME != 0) {
    const int delay = (ros::Time::now() - msg->header.stamp).toNSec() / 1000;
    if (delay > RealtimeTimings::LOWLEVEL_CONTROLLER_WAIT_TIME +
                    RealtimeTimings::STATE_ESTIMATION_CYCLE_TIME) {
      ROS_WARN_THROTTLE(1, "delay (measure-> HLC set point) is too big: %dus", delay);
    }
  }
  steering_controller_->setTargetAngle(msg->steering_angle);
}

void SteeringControllerNode::servoSetValuePublisherLoop() {
  while (!boost::this_thread::interruption_requested()) {
    double servo_set_value = 0.0;
    if (steering_controller_->getNextServoSetValue(&servo_set_value)) {
      common_msgs::Float32Stamped msg;
      msg.header.stamp = ros::Time::now();
      msg.data = servo_set_value;
      servo_set_value_publisher_.publish(msg);
    }
  }
}

void SteeringControllerNode::angleErrorPublisherLoop() {
  while (!boost::this_thread::interruption_requested()) {
    double angle_error = 0.0;
    if (steering_controller_->getNextAngleError(&angle_error)) {
      common_msgs::Float32Stamped msg;
      msg.header.stamp = ros::Time::now();
      msg.data = angle_error;
      angle_error_publisher_.publish(msg);
    }
  }
}

const std::string SteeringControllerNode::getName() {
  return std::string("steering_controller");
}

CREATE_NODE_WITH_FANCY_DEBUG(SteeringControllerNodeRealtime, SteeringControllerNodeDebug)
