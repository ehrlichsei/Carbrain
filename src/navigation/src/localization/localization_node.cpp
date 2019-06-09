#include "localization_node.h"
#include <common/macros.h>

THIRD_PARTY_HEADERS_BEGIN
#include <geometry_msgs/TransformStamped.h>
#include "common/tf2_eigen_addon.h"
#include "navigation/LocalizationConfig.h"
THIRD_PARTY_HEADERS_END

#include "common/node_creation_makros.h"

LocalizationNode::LocalizationNode(ros::NodeHandle &node_handle)
    : NodeBase(node_handle), localization_(&parameter_handler_) {
  parameter_handler_.addDynamicReconfigureServer<navigation::LocalizationConfig>(node_handle_);
}

void LocalizationNode::startModule() {

  localization_.reset();

  state_estimation_subscriber_ = node_handle_.subscribe(
      "state_estimation", 5, &LocalizationNode::stateEstimationCallback, this, common::noDelayTransport());
  reset_location_subscriber_ = node_handle_.subscribe(
      "reset_location", 1, &LocalizationNode::resetLocationCallback, this);

  pose_publisher_ =
      node_handle_.advertise<geometry_msgs::PoseStamped>("pose_estimation", 10);
}

void LocalizationNode::stopModule() {
  state_estimation_subscriber_.shutdown();
  reset_location_subscriber_.shutdown();
  pose_publisher_.shutdown();
}

void LocalizationNode::stateEstimationCallback(const state_estimation_msgs::State::ConstPtr &state_msg) {

  geometry_msgs::TransformStamped transform_msg =
      eigenToTransform(localization_.update(state_msg));
  transform_msg.child_frame_id = "vehicle";

  // Publish the new vehicle to world transformation (meaning the new pose)
  transform_broadcaster_.sendTransform(transform_msg);

  // Publish the new pose (actually the KS origin, but with new transformation)
  // This gets currently done to notifiy other node, that the vehicle->world
  // transformation
  // has been updated.
  geometry_msgs::PoseStamped pose;
  pose.header.stamp = state_msg->header.stamp;
  pose.header.frame_id = "vehicle";
  pose.pose.orientation.x = 0.0;
  pose_publisher_.publish(pose);
}

void LocalizationNode::resetLocationCallback(const std_msgs::Empty::ConstPtr &) {
  localization_.reset();
}

const std::string LocalizationNode::getName() {
  return std::string("localization");
}

CREATE_NODE(LocalizationNode)
