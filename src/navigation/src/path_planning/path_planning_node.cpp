#include "path_planning_node.h"
#include <common/macros.h>

THIRD_PARTY_HEADERS_BEGIN
#include "navigation/PathPlanningConfig.h"
THIRD_PARTY_HEADERS_END

#include "middle_planner.h"
#include "least_squares_planner.h"

#include "navigation/driving_corridor.h"
#include "common/node_creation_makros.h"
#include "common/path_conversion.h"

#include FANCY_DEBUG_INCLUDE("debug/path_planning_node_debug.h")


PathPlanningNode::PathPlanningNode(ros::NodeHandle& node_handle)
    : NodeBase(node_handle),
      path_planner_(std::make_unique<LeastSquaresPlanner>(parameter_handler_)),
      tf2_listener(tf2_buffer) {
  parameter_handler_.registerParam(MAX_ALLOWED_PROCESSING_TIME);
  parameter_handler_.addDynamicReconfigureServer<navigation::PathPlanningConfig>(node_handle_);
}


void PathPlanningNode::startModule() {
  ROS_DEBUG("startModule");
  car_corridor_subscriber_ = node_handle_.subscribe(
      "car_corridor", 1, &PathPlanningNode::carCorridorCallback, this);

  safe_target_path_publisher_ =
      node_handle_.advertise<nav_msgs::Path>("safe_target_path", 1);
  reverse_out_of_parking_spot_server_ = node_handle_.advertiseService(
      "reverse_out_of_parking_spot_service", &PathPlanningNode::reverseOutOfParkingSpot, this);
  out_of_start_box_server_ = node_handle_.advertiseService(
      "out_of_start_box_service", &PathPlanningNode::outOfStartBox, this);
}

void PathPlanningNode::stopModule() {
  car_corridor_subscriber_.shutdown();
  safe_target_path_publisher_.shutdown();
  reverse_out_of_parking_spot_server_.shutdown();
  reverse_out_of_parking_spot = false;
}

PathPlanner::Path PathPlanningNode::generateStraightPath() {
  const double path_length = 3.0;
  const double dist_between_points = 0.05;
  PathPlanner::Path path(
      static_cast<std::size_t>(std::floor(path_length / dist_between_points)),
      dist_between_points * Eigen::Vector2d::UnitX());

  // TODO c++17: std::partial_sum and setting path[0] can be combined to
  // std::exclusive_scan (has init value parameter)

  if (path.empty()) {
    return path;
  }
  const Eigen::Vector2d path_start(-0.5, 0);
  path[0] = path_start;

  // generate path points with a distance between points of
  // 'dist_between_points'
  std::partial_sum(path.begin(), path.end(), path.begin());
  return path;
}

const std::string PathPlanningNode::getName() {
  return std::string("path_planning");
}

void PathPlanningNode::publishReverseParkingPath(const ros::TimerEvent& /*event*/) {
  reverse_out_of_parking_spot_path.header.stamp = ros::Time::now();
  safe_target_path_publisher_.publish(reverse_out_of_parking_spot_path);
}

inline void doTransform(PathPlanner::Path& path,
                        const geometry_msgs::TransformStamped& transform) {
  Eigen::Affine3d world_to_path_3d = tf2::transformToEigen(transform);
  Eigen::Affine2d world_to_path_transform_2d =
      Eigen::Translation2d(world_to_path_3d.translation().topRows<2>()) *
      world_to_path_3d.linear().topLeftCorner<2, 2>();

  for (auto& x : path) {
    x = world_to_path_transform_2d * x;
  }
}

PathPlanningNode::OptionalTransformation PathPlanningNode::lookupTransformationToPath(
    const ros::Time& /*stamp*/, const std::string& source_frame) {
  try {
    // the path->world we want will be publish before we read here, so we just
    // can use the latest recieved.
    return tf2_buffer.lookupTransform(
        "path", source_frame, ros::Time(0), ros::Duration(0.0));
  } catch (const tf2::TransformException& ex) {
    ROS_WARN("Can NOT transform path to world: %s", ex.what());
    return boost::none;
  }
}

void PathPlanningNode::carCorridorCallback(const navigation_msgs::DrivingCorridor::ConstPtr& car_corridor_msg) {
  const ros::WallTime start = ros::WallTime::now();
  if (reverse_out_of_parking_spot) {
    return;
  }
  ROS_DEBUG_THROTTLE(1, "rawTargetPathCallback [throttled to 1 Hz]");

  if (car_corridor_msg->gates.empty()) {
    return;
  }
  const std::string source_frame = straight_path_out_of_start_box ? "vehicle" : "world";
  const OptionalTransformation to_path_transform =
      lookupTransformationToPath(car_corridor_msg->header.stamp, source_frame);

  if (!to_path_transform) {
    return;
  }

  PathPlanner::Path target_path =
      straight_path_out_of_start_box
          ? generateStraightPath()
          : path_planner_->planPathOnCorridor(DrivingCorridor::fromMessage(car_corridor_msg));

  doTransform(target_path, to_path_transform.get());

  safe_target_path_publisher_.publish(common::createPathMsg(
      target_path, car_corridor_msg->header.stamp, "path"));

  const int elapsed = (ros::WallTime::now() - start).toNSec() / 1000;
  if (elapsed > parameter_handler_.getParam(MAX_ALLOWED_PROCESSING_TIME)) {
    ROS_WARN("path_planning took too long. elapsed %d us", elapsed);
  }
}

bool PathPlanningNode::outOfStartBox(std_srvs::SetBoolRequest& req,
                                     std_srvs::SetBoolResponse& /*res*/) {
  straight_path_out_of_start_box = req.data;
  return true;
}

bool PathPlanningNode::reverseOutOfParkingSpot(
    navigation_msgs::ReverseOutOfParkingSpot::Request& req,
    navigation_msgs::ReverseOutOfParkingSpot::Response& /*res*/) {
  reverse_out_of_parking_spot_path = req.path_reverse_out_of_parking_spot;
  reverse_out_of_parking_spot = req.enable;
  if (!reverse_out_of_parking_spot) {
    publishing_reverse_path_timer_.stop();
    return true;
  }
  publishing_reverse_path_timer_ = node_handle_.createTimer(
      ros::Duration(0.1), &PathPlanningNode::publishReverseParkingPath, this);
  return true;
}

CREATE_NODE_WITH_FANCY_DEBUG(PathPlanningNode, PathPlanningNodeDebug)
