#include "parkinglot_aligner_node.h"

#include "common/node_creation_makros.h"
THIRD_PARTY_HEADERS_BEGIN
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
THIRD_PARTY_HEADERS_END

#include <common/tf2_eigen_addon.h>



ParkinglotAlignerNode::ParkinglotAlignerNode(ros::NodeHandle &node_handle)
    : NodeBase(node_handle),
      parkinglot_aligner_(&parameter_handler_),
      transform_listener(transform_buffer) {
  // parameter_handler_.addDynamicReconfigureServer<navigation::ParkinglotAlignerConfig>(
  //   node_handle_);
}

void ParkinglotAlignerNode::startModule() {
  full_corridor_subsriber_ = node_handle_.subscribe(
      "full_corridor", 100, &ParkinglotAlignerNode::fullCorridorCallback, this);

  parkinglot_point_subsriber_ = node_handle_.subscribe(
      "parkinglot_point", 10, &ParkinglotAlignerNode::parkinglotPointCallback, this);

  // Set start point once for the traditional parking discipline.
  // TODO: Remove this when parking during the round trip mode.
  parkinglot_aligner_.setStartPoint(Eigen::Vector3d::Zero());
}

void ParkinglotAlignerNode::stopModule() {
  full_corridor_subsriber_.shutdown();
  parkinglot_point_subsriber_.shutdown();
}

const std::string ParkinglotAlignerNode::getName() {
  return std::string("parkinglot_aligner");
}

void ParkinglotAlignerNode::fullCorridorCallback(const navigation_msgs::DrivingCorridor::ConstPtr& full_corridor_msg) {
  const DrivingCorridor corridor = DrivingCorridor::fromMessage(full_corridor_msg);
  Eigen::Affine3d transformation;
  Eigen::Vector3d carPosition;
  if (!(getCarPosition(carPosition) &&
        parkinglot_aligner_.calculateTransformation(corridor, carPosition, transformation))) {
    return;
  }
  // if the car position could be loaded from the transform buffer and the
  // transformation could be calculated publish the transformation
  publishParkinglotToWorldTransform(transformation, full_corridor_msg->header.stamp);
}

void ParkinglotAlignerNode::publishParkinglotToWorldTransform(const Eigen::Affine3d &parkinglot_to_world_transform,
                                                              const ros::Time &stamp) {

  tf2::Stamped<Eigen::Affine3d> parkinglot_to_world_transform_stamped(
      parkinglot_to_world_transform, stamp, "world");

  geometry_msgs::TransformStamped parkinglot_to_world_transform_msg =
      tf2::toMsg(parkinglot_to_world_transform_stamped, "parkinglot");

  tf2_broadcaster_.sendTransform(parkinglot_to_world_transform_msg);
}

void ParkinglotAlignerNode::parkinglotPointCallback(const geometry_msgs::Point::ConstPtr &parkinglot_point_msg) {
  Eigen::Vector3d v;
  tf2::fromMsg(*parkinglot_point_msg, v);
  parkinglot_aligner_.setStartPoint(v);
}


bool ParkinglotAlignerNode::getCarPosition(Eigen::Vector3d &carPosition) {
  tf2::Stamped<Eigen::Vector3d> vehicle_in_world;
  try {
    vehicle_in_world = transform_buffer.transform(
        tf2::Stamped<Eigen::Vector3d>(
            Eigen::Vector3d::Zero(), ros::Time(0), "vehicle"),
        "world");
  } catch (const tf2::TransformException& e) {
    ROS_WARN_STREAM("Could not transform from vehicle to world: " << e.what());
    return false;
  }
  carPosition = vehicle_in_world; // NOLINT;
  return true;
}
CREATE_NODE(ParkinglotAlignerNode)
