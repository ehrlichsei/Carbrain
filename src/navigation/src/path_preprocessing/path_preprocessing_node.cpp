#include "path_preprocessing_node.h"
#include <common/macros.h>

THIRD_PARTY_HEADERS_BEGIN
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_eigen/tf2_eigen.h>
#include "navigation/PathPreprocessingConfig.h"
#include "perception_msgs/PerpendicularParkingSpot.h"
THIRD_PARTY_HEADERS_END

#include <common/tf2_eigen_addon.h>
#include "common/node_creation_makros.h"
#include "common/debug.h"
#include "common/msg_helper.h"
#include "lane_utils.h"


PathPreprocessingNode::PathPreprocessingNode(ros::NodeHandle &node_handle)
    : NodeBase(node_handle),
      path_preprocessing_(&parameter_handler_),
      tf2_listener_(tf2_buffer_),
      subscriber_synchronizer_(
          left_line_subscriber_, middle_line_subscriber_, right_line_subscriber_, 1) {
  parameter_handler_.registerParam(MAX_ALLOWED_PROCESSING_TIME);

  parameter_handler_.addDynamicReconfigureServer<navigation::PathPreprocessingConfig>(node_handle_);

  subscriber_synchronizer_.registerCallback(boost::bind(
      &PathPreprocessingNode::onNewLanePointsReceived, this, _1, _2, _3));
}

void PathPreprocessingNode::startModule() {
  left_line_subscriber_.subscribe(
      node_handle_, "road_lane_left", 1, common::noDelayTransport());
  middle_line_subscriber_.subscribe(
      node_handle_, "road_lane_middle", 1, common::noDelayTransport());
  right_line_subscriber_.subscribe(
      node_handle_, "road_lane_right", 1, common::noDelayTransport());

  autonomous_mode_state_subscriber_ = node_handle_.subscribe(
      "auto_reset", 1, &PathPreprocessingNode::clearTrackedLane, this);

  lookahead_point_subscriber_ = node_handle_.subscribe(
      "lookahead_point", 1, &PathPreprocessingNode::receiveLookaheadPoint, this);

  preprocessed_path_publisher_ =
      node_handle_.advertise<nav_msgs::Path>("raw_target_path", 10, true);
  full_corridor_publisher_ =
      node_handle_.advertise<navigation_msgs::DrivingCorridor>("full_corridor", 1);
  turning_service_ =
      node_handle_.advertiseService("turn_at", &PathPreprocessingNode::turnAt, this);
  perpendicular_parking_service_ = node_handle_.advertiseService(
      "park_perpendicular_at", &PathPreprocessingNode::parkPerpendicular, this);
  reset_service_ =
      node_handle_.advertiseService("reset", &PathPreprocessingNode::reset, this);
}

void PathPreprocessingNode::stopModule() {
  path_preprocessing_.reset();
  autonomous_mode_state_subscriber_.shutdown();
  left_line_subscriber_.unsubscribe();
  middle_line_subscriber_.unsubscribe();
  right_line_subscriber_.unsubscribe();
  lookahead_point_subscriber_.shutdown();
  preprocessed_path_publisher_.shutdown();
  full_corridor_publisher_.shutdown();
  turning_service_.shutdown();
  perpendicular_parking_service_.shutdown();
  reset_service_.shutdown();
}

bool PathPreprocessingNode::reset(std_srvs::Empty::Request &, std_srvs::Empty::Response &) {
  ROS_INFO("Clearing tracked lane points and gates");
  path_preprocessing_.reset();
  return true;
}

const std::string PathPreprocessingNode::getName() {
  return std::string("path_preprocessing");
}

void PathPreprocessingNode::onNewLanePointsReceived(
    const nav_msgs::Path::ConstPtr &left_lane_path,
    const nav_msgs::Path::ConstPtr &middle_lane_path,
    const nav_msgs::Path::ConstPtr &right_lane_path) {

  const ros::WallTime start = ros::WallTime::now();
  ROS_DEBUG("Received new lane paths");

  Eigen::Affine3d vehicle_to_world_3d;
  try {
    vehicle_to_world_3d = getVehicleToWorldTransformation(middle_lane_path->header.stamp);
  } catch (const tf2::LookupException &ex) {
    ROS_ERROR("Could not lookup transform %s", ex.what());
    return;
  }

  LaneUtils::Lanes lanes{pathToWorldPoints(left_lane_path),
                         pathToWorldPoints(middle_lane_path),
                         pathToWorldPoints(right_lane_path)};
  if (!path_preprocessing_.createDrivingCorridor(vehicle_to_world_3d, lanes) &&
      !path_preprocessing_.pointBehindStartGateParking(vehicle_to_world_3d.translation())) {
    return;
  }

  const ros::Time transform_time = middle_lane_path->header.stamp;
  publishCorridor(
      full_corridor_publisher_, transform_time, path_preprocessing_.getFullCorridor());
  const auto elapsed = (ros::WallTime::now() - start).toNSec() / 1000;
  if (elapsed > parameter_handler_.getParam(MAX_ALLOWED_PROCESSING_TIME)) {
    ROS_INFO("path_preprocessing takes too long : %ld us", elapsed);
  }
  publishPathToWorldTransform(path_preprocessing_.getPathTransform(), transform_time);
}

void PathPreprocessingNode::publishRawPath(const ros::Time &stamp,
                                           const common::EigenAlignedVector<Eigen::Vector3d> &tracked_lane) {
  nav_msgs::Path preprocessed_path;
  preprocessed_path.header.frame_id = "world";
  preprocessed_path.header.stamp = stamp;
  preprocessed_path.poses.reserve(tracked_lane.size());

  for (const Eigen::Vector3d &lane_point : tracked_lane) {
    tf2::Stamped<Eigen::Affine3d> lane_transformation_stamped(
        Eigen::Affine3d(Eigen::Translation3d(lane_point)), stamp, "world");

    preprocessed_path.poses.push_back(tf2::toMsg(lane_transformation_stamped));
  }
  preprocessed_path_publisher_.publish(preprocessed_path);
}

void PathPreprocessingNode::publishCorridor(const ros::Publisher &publisher,
                                            const ros::Time &stamp,
                                            const DrivingCorridor &corridor) {
  navigation_msgs::DrivingCorridor corridor_msg = corridor.toMessage();
  corridor_msg.header.stamp = stamp;
  corridor_msg.header.frame_id = "world";
  publisher.publish(common::toConstPtr(corridor_msg));
}

void PathPreprocessingNode::clearTrackedLane(const std_msgs::Empty::ConstPtr &) {
  ROS_INFO("Clearing tracked lane points and gates");
  path_preprocessing_.reset();
}

void PathPreprocessingNode::receiveLookaheadPoint(const geometry_msgs::PointStamped &msg) {
  Eigen::Vector3d point;
  tf2::fromMsg(msg.point, point);
  path_preprocessing_.setVehicleLookaheadPoint(point);
}

common::EigenAlignedVector<Eigen::Vector3d> PathPreprocessingNode::pathToWorldPoints(
    const nav_msgs::Path::ConstPtr &path) {
  common::EigenAlignedVector<Eigen::Vector3d> world_points;

  Eigen::Affine3d vehicle_to_world_transform =
      getVehicleToWorldTransformation(path->header.stamp);

  world_points.reserve(path->poses.size());
  for (const geometry_msgs::PoseStamped &pose : path->poses) {
    Eigen::Vector3d p;
    tf2::fromMsg(pose.pose.position, p);
    world_points.push_back(vehicle_to_world_transform * p);
  }
  return world_points;
}

void PathPreprocessingNode::publishPathToWorldTransform(const Eigen::Affine3d &path_to_world_transform,
                                                        const ros::Time &stamp) {

  tf2::Stamped<Eigen::Affine3d> path_to_world_transform_stamped(
      path_to_world_transform, stamp, "world");

  geometry_msgs::TransformStamped path_to_world_transform_msg =
      tf2::toMsg(path_to_world_transform_stamped, "path");

  tf2_broadcaster_.sendTransform(path_to_world_transform_msg);
}

Eigen::Affine3d PathPreprocessingNode::getVehicleToWorldTransformation(const ros::Time &transform_time) const {
  geometry_msgs::TransformStamped vehicle_to_world_transform_message;
  try {
    vehicle_to_world_transform_message = tf2_buffer_.lookupTransform(
        "world", "vehicle", transform_time, ros::Duration(0));
  } catch (const tf2::TransformException & /*ex*/) {
    // ROS_WARN("Can NOT transform vehicle to world: %s", ex.what());
    vehicle_to_world_transform_message = tf2_buffer_.lookupTransform(
        "world", "vehicle", ros::Time(0), ros::Duration(0));
    const auto d =
        (transform_time - vehicle_to_world_transform_message.header.stamp).toNSec() / 1000;
    if (d > 8000) {
      ROS_WARN_THROTTLE(5, "transformation is outdated by %ld us", d);
    }
  }

  return tf2::transformToEigen(vehicle_to_world_transform_message);
}


bool PathPreprocessingNode::turnAt(navigation_msgs::TurnAt::Request &req,
                                   navigation_msgs::TurnAt::Response &) {
  Eigen::Affine3d pose;
  tf2::fromMsg(req.pose, pose);
  if (req.direction == navigation_msgs::TurnAt::Request::LEFT) {
    path_preprocessing_.setTurning(true, pose);
  } else if (req.direction == navigation_msgs::TurnAt::Request::RIGHT) {
    path_preprocessing_.setTurning(false, pose);
  } else {
    path_preprocessing_.resetTurning();
  }

  return true;
}

bool PathPreprocessingNode::parkPerpendicular(navigation_msgs::ParkPerpendicular::Request &req,
                                              navigation_msgs::ParkPerpendicular::Response & /*res*/) {
  Eigen::Affine3d left;
  Eigen::Affine3d right;
  tf2::fromMsg(req.parking_spot.entrance_pose_left, left);
  tf2::fromMsg(req.parking_spot.entrance_pose_right, right);
  path_preprocessing_.parkPerpendicular(left, right);

  return true;
}


CREATE_NODE(PathPreprocessingNode)
