#include <common/macros.h>

THIRD_PARTY_HEADERS_BEGIN
#include <navigation/driving_corridor.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_ros/transform_listener.h>
#include <algorithm>
#include <boost/array.hpp>
#include <boost/make_shared.hpp>
#include "navigation/CollisionDetectionConfig.h"
THIRD_PARTY_HEADERS_END

#include "collision_detection_node.h"
#include "navigation/line_segment.h"

#include "common/msg_helper.h"
#include "common/node_creation_makros.h"

CollisionDetectionNode::CollisionDetectionNode(ros::NodeHandle& node_handle)
    : NodeBase(node_handle),
      collision_detection_(&parameter_handler_),
      safety_margin_(&parameter_handler_) {
  parameter_handler_.registerParam(MAX_ALLOWED_PROCESSING_TIME);
  parameter_handler_.addDynamicReconfigureServer<navigation::CollisionDetectionConfig>(node_handle_);
  road_closures_msg_ = boost::make_shared<navigation_msgs::RoadClosures>();
}

void CollisionDetectionNode::startModule() {
  full_corridor_subsriber_ = node_handle_.subscribe(
      "full_corridor", 1, &CollisionDetectionNode::fullCorridorCallback, this);
  tracked_obstacles_subsriber_ = node_handle_.subscribe(
      "obstacles", 1, &CollisionDetectionNode::trackedObstaclesCallback, this);
  tracked_road_closures_subsriber_ = node_handle_.subscribe(
      "tracked_road_closures", 1, &CollisionDetectionNode::trackedRoadClosuresCallback, this);
  stop_line_subscriber_ = node_handle_.subscribe(
      "stopline", 1, &CollisionDetectionNode::junctionsCallback, this);
  no_passing_zone_subscriber_ = node_handle_.subscribe(
      "no_passing_zones", 3, &CollisionDetectionNode::noPassingZonesCallback, this);
  crosswalk_subscriber_ = node_handle_.subscribe(
      "crosswalks", 1, &CollisionDetectionNode::crosswalksCallback, this);
  arrow_marking_subscriber_ = node_handle_.subscribe(
      "arrow_markings", 1, &CollisionDetectionNode::arrowMarkingsCallback, this);
  safe_corridor_publisher_ =
      node_handle_.advertise<navigation_msgs::DrivingCorridor>("safe_corridor", 1);
  car_corridor_publisher_ =
      node_handle_.advertise<navigation_msgs::DrivingCorridor>("car_corridor", 1);
  set_drive_past_service =
      node_handle_.advertiseService("set_drive_past_next_road_closure",
                                    &CollisionDetection::setDrivePastNextRoadClosure,
                                    &collision_detection_);
  set_respect_no_passing_zones_service =
      node_handle_.advertiseService("set_respect_no_passing_zones",
                                    &CollisionDetection::setRespectNoPassingZones,
                                    &collision_detection_);
  obstacles_msg_ = boost::make_shared<navigation_msgs::Obstacles>();
  road_closures_msg_ = boost::make_shared<navigation_msgs::RoadClosures>();
  crosswalks_msg_ = boost::make_shared<navigation_msgs::Crosswalks>();
  arrow_markings_msg_ = boost::make_shared<perception_msgs::ArrowMarkings>();
  stopline_position_ = nullptr;
}

void CollisionDetectionNode::stopModule() {
  full_corridor_subsriber_.shutdown();
  tracked_obstacles_subsriber_.shutdown();
  tracked_road_closures_subsriber_.shutdown();
  stop_line_subscriber_.shutdown();
  no_passing_zone_subscriber_.shutdown();
  crosswalk_subscriber_.shutdown();
  arrow_marking_subscriber_.shutdown();
  safe_corridor_publisher_.shutdown();
  car_corridor_publisher_.shutdown();
  set_drive_past_service.shutdown();
  set_respect_no_passing_zones_service.shutdown();
}

void CollisionDetectionNode::fullCorridorCallback(
    const navigation_msgs::DrivingCorridor::ConstPtr& full_corridor_msg) {

  const ros::WallTime start = ros::WallTime::now();
  DrivingCorridor safe_corridor = DrivingCorridor::fromMessage(full_corridor_msg);

  collision_detection_.cropAroundObstacles(
      safe_corridor, navigation::Obstacle::fromMessage(obstacles_msg_));

  collision_detection_.cropAroundRoadClosures(safe_corridor, road_closures_msg_);

  collision_detection_.cropAtCrosswalks(safe_corridor, crosswalks_msg_);

  // collision_detection_.cropAtArrowMarkings(safe_corridor, arrow_markings_msg_);

  if (stopline_position_ != nullptr) {
    collision_detection_.cropAtIntersections(safe_corridor, *stopline_position_);
  }

  // shrink lane at intersection etc.

  navigation_msgs::DrivingCorridor safe_corridor_msg = safe_corridor.toMessage();
  safe_corridor_msg.header.stamp = full_corridor_msg->header.stamp;
  safe_corridor_msg.header.frame_id = full_corridor_msg->header.frame_id;
  safe_corridor_publisher_.publish(common::toConstPtr(safe_corridor_msg));

  DrivingCorridor car_corridor = safe_corridor;

  safety_margin_.applySafetyMargin(safe_corridor, car_corridor);

  navigation_msgs::DrivingCorridor car_corridor_msg = car_corridor.toMessage();
  car_corridor_msg.header.stamp = full_corridor_msg->header.stamp;
  car_corridor_msg.header.frame_id = full_corridor_msg->header.frame_id;
  car_corridor_publisher_.publish(common::toConstPtr(car_corridor_msg));

  const int elapsed = (ros::WallTime::now() - start).toNSec() / 1000;
  if (elapsed > parameter_handler_.getParam(MAX_ALLOWED_PROCESSING_TIME)) {
    ROS_WARN("collision_detection took too long. elapsed %d us", elapsed);
  }
}

void CollisionDetectionNode::trackedObstaclesCallback(const navigation_msgs::Obstacles::ConstPtr& obstacles_msg) {
  obstacles_msg_ = obstacles_msg;
}


void CollisionDetectionNode::trackedRoadClosuresCallback(
    const navigation_msgs::RoadClosures::ConstPtr& roadClosures_msg) {
  road_closures_msg_ = roadClosures_msg;
}

void CollisionDetectionNode::junctionsCallback(const navigation_msgs::Junctions::ConstPtr& junctions_msg) {
  std::vector<navigation_msgs::Junction> junction_right_lane;
  std::copy_if(junctions_msg->sub_messages.begin(),
               junctions_msg->sub_messages.end(),
               std::back_inserter(junction_right_lane),
               [](const auto& junction) {
                 return junction.junction_type == perception_msgs::Junction::TYPE_GIVEWAY_RIGHT ||
                        junction.junction_type == perception_msgs::Junction::TYPE_STOPLINE_RIGHT;
               });
  if (junction_right_lane.empty()) {
    stopline_position_.reset(nullptr);
    return;
  }
  Eigen::Vector3d position;
  tf2::fromMsg(junction_right_lane[0].pose.position, position);
  stopline_position_ = std::make_unique<Eigen::Vector3d>(position);
  ROS_INFO_THROTTLE(4, "stopline received");
}

void CollisionDetectionNode::noPassingZonesCallback(
    const navigation_msgs::NoPassingZones::ConstPtr& no_passing_zones_msg) {
  collision_detection_.setNoPassingZones(NoPassingZones::fromMessage(*no_passing_zones_msg));
}

void CollisionDetectionNode::crosswalksCallback(
    const navigation_msgs::Crosswalks::ConstPtr& crosswalks_msg) {
  crosswalks_msg_ = crosswalks_msg;
}

void CollisionDetectionNode::arrowMarkingsCallback(
    const perception_msgs::ArrowMarkings::ConstPtr& arrow_markings_msg) {
  arrow_markings_msg_ = arrow_markings_msg;
}

const std::string CollisionDetectionNode::getName() {
  return std::string("collision_detection");
}



CREATE_NODE(CollisionDetectionNode)
