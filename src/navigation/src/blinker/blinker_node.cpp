#include "blinker_node.h"

#include <ros/node_handle.h>
#include <Eigen/Core>
#include <functional>
#include "common/node_creation_makros.h"
#include "common/path_conversion.h"
#include "navigation/driving_corridor.h"

BlinkerNode::BlinkerNode(ros::NodeHandle &node_handle)
    : NodeBase(node_handle), tf_listener(tf_buffer), pavlov_blinker_command_enabled(false) {
  pavlov_blinker_command.command = controller_msgs::BlinkerCommand::NONE;
  parameter_handler_.registerParam(Blinker::VEHICLE_WIDTH);
  parameter_handler_.registerParam(Blinker::VEHICLE_WIDTH_SUBTRACTIION);
}

void BlinkerNode::startModule() {
  timer = node_handle_.createTimer(
      ros::Duration(0.75), &BlinkerNode::timerCallback, this, true);
  timer.stop();
  set_pavlov_blinker_command_server = node_handle_.advertiseService(
      "blinker_command_pavlov", &BlinkerNode::setPavlovBlinkerCommand, this);
  safe_corridor_subscriber = node_handle_.subscribe(
      "safe_corridor", 1, &BlinkerNode::safeCorridorCallback, this);
  blinker_publisher =
      node_handle_.advertise<controller_msgs::LightsCommand>("blinker_command", 1);
}

void BlinkerNode::stopModule() {
  set_pavlov_blinker_command_server.shutdown();
  blinker_publisher.shutdown();
}

void BlinkerNode::blinkerCommand(const BlinkerDecision blinker_decision) {
  if (pavlov_blinker_command_enabled) {
    return;
  }
  if (blinker_decision == old_blinker_decision) {
    counter_same_decision_++;
    const size_t changing_threshold = 4;
    if (counter_same_decision_ == changing_threshold) {
      changeBlinker(blinker_decision);
    }
  } else {
    counter_same_decision_ = 0;
    old_blinker_decision = blinker_decision;
  }
}

void BlinkerNode::changeBlinker(const BlinkerDecision blinker_decision) {
  controller_msgs::BlinkerCommand blinker_msg;
  switch (blinker_decision) {
    case BlinkerDecision::BLINK_LEFT:
      ROS_INFO("BLINK_LEFT");
      blinker_msg.command = controller_msgs::BlinkerCommand::LEFT;
      resetTimer(timer_duration);
      break;
    case BlinkerDecision::BLINK_RIGHT:
      ROS_INFO("BLINK_RIGHT");
      blinker_msg.command = controller_msgs::BlinkerCommand::RIGHT;
      resetTimer(timer_duration);
      break;
    case BlinkerDecision::BLINK_RIGHT_AND_LEFT:
      ROS_INFO("BLINK_RIGHT_AND_LEFT");
      blinker_msg.command = controller_msgs::BlinkerCommand::BOTH;
      resetTimer(timer_duration);
      break;
    case BlinkerDecision::ABORT_BLINKING:
      ROS_INFO("ABORT_BLINKING");
      blinker_msg.command = controller_msgs::BlinkerCommand::NONE;
      break;
    case BlinkerDecision::NO_DECISION:
      ROS_INFO("NO_DECISION");
      return;
    default:
      ROS_ERROR_THROTTLE(1, "FIXME: UNIMPLEMENTED BLINKER DECISION");
  }
  controller_msgs::LightsCommand lights_msg;
  lights_msg.blinker = blinker_msg;
  blinker_publisher.publish(lights_msg);
}

void BlinkerNode::safeCorridorCallback(const navigation_msgs::DrivingCorridor::ConstPtr &safe_corridor_msg) {
  DrivingCorridor safe_corridor = DrivingCorridor::fromMessage(safe_corridor_msg);
  Eigen::Affine3d vehicle_pose = Eigen::Affine3d::Identity();
  if (!vehicleToWorld(Eigen::Affine3d::Identity(), vehicle_pose)) {
    return;
  }
  BlinkerDecision decision = blinker.decideBlink(safe_corridor, vehicle_pose);
  blinkerCommand(decision);
}

void BlinkerNode::timerCallback(const ros::TimerEvent &) {
  controller_msgs::BlinkerCommand blinker_msg;
  ROS_INFO("NO_BLINKING");
  blinker_msg.command = controller_msgs::BlinkerCommand::NONE;
  controller_msgs::LightsCommand lights_msg;
  lights_msg.blinker = blinker_msg;
  blinker_publisher.publish(lights_msg);
}

void BlinkerNode::resetTimer(const double duration) {
  timer.stop();
  timer.setPeriod(ros::Duration(duration));
  timer.start();
}

bool BlinkerNode::vehicleToWorld(const Eigen::Affine3d &vehicle_pose,
                                 Eigen::Affine3d &world_pose) {
  try {
    auto vehicle_to_world = tf_buffer.lookupTransform("world", "vehicle", ros::Time(0));
    tf2::doTransform(vehicle_pose, world_pose, vehicle_to_world);
    return true;
  } catch (const tf2::TransformException &ex) {
    ROS_WARN_THROTTLE(1, "Can NOT transform vehicle to world: %s", ex.what());
  }
  return false;
}

const std::string BlinkerNode::getName() { return std::string("blinker"); }


bool BlinkerNode::setPavlovBlinkerCommand(navigation_msgs::PavlovBlinkerCommand::Request &req,
                                          navigation_msgs::PavlovBlinkerCommand::Response &) {
  timer.stop();
  pavlov_blinker_command = req.pavlov_blinker_command;
  pavlov_blinker_command_enabled = req.enable;
  controller_msgs::LightsCommand lights_msg;
  lights_msg.blinker = pavlov_blinker_command;
  blinker_publisher.publish(lights_msg);
  return true;
}

CREATE_NODE(BlinkerNode)
