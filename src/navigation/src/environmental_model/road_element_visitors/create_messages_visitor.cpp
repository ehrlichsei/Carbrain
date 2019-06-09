#include "create_messages_visitor.h"

#include <common/macros.h>
THIRD_PARTY_HEADERS_BEGIN
#include <opencv2/imgproc.hpp>
THIRD_PARTY_HEADERS_END
#include "common/angle_conversions.h"
#include <common/tf2_eigen_addon.h>

namespace environmental_model {

CreateMessagesVisitor::CreateMessagesVisitor(const ros::Time &time_stamp) {
  crosswalks.header.stamp = time_stamp;
  junctions.header.stamp = time_stamp;
  road_closures.header.stamp = time_stamp;
  arrow_markings.header.stamp = time_stamp;
  obstacles.header.stamp = time_stamp;
  navigation_obstacles.header.stamp = time_stamp;
  start_lines.header.stamp = time_stamp;
  parking_spots.header.stamp = time_stamp;
  speed_limits.header.stamp = time_stamp;
  unidentifieds.header.stamp = time_stamp;
  crosswalks.header.frame_id = "world";
  junctions.header.frame_id = "world";
  road_closures.header.frame_id = "world";
  arrow_markings.header.frame_id = "world";
  obstacles.header.frame_id = "world";
  navigation_obstacles.header.frame_id = "world";
  start_lines.header.frame_id = "world";
  parking_spots.header.frame_id = "world";
  speed_limits.header.frame_id = "world";
  unidentifieds.header.frame_id = "world";
}

void CreateMessagesVisitor::visit(Unidentified &road_object, TrackingElement &tracking_element) {
  if (!tracking_element.shouldPublishElement()) {
    return;
  }
  auto msg = road_object.getMsg();
  msg.id = tracking_element.getId();
  unidentifieds.sub_messages.push_back(msg);
}

void CreateMessagesVisitor::visit(Obstacle &road_object, TrackingElement &tracking_element) {
  if (!tracking_element.shouldPublishElement() || road_object.stoppedPrediction()) {
    return;
  }
  auto msg = road_object.getMsg();
  if (msg.vertices.size() != 4) {
    ROS_ERROR_STREAM(
        "invalid obstacle message in CreateMessagesVisitor: obstacle has "
        << msg.vertices.size() << " vertices.");
    return;
  }
  msg.id = tracking_element.getId();
  obstacles.sub_messages.push_back(msg);
  auto nav_obstacle = toNavigationObstacle(msg);
  auto dynamic_obst_state = road_object.getDynamicState();
  nav_obstacle.is_dynamic = dynamic_obst_state.prob_dynamic;
  nav_obstacle.velocity = dynamic_obst_state.velocity;

  navigation_obstacles.sub_messages.push_back(nav_obstacle);
}

void CreateMessagesVisitor::visit(Junction &road_object, TrackingElement &tracking_element) {
  if (!tracking_element.shouldPublishElement()) {
    return;
  }
  auto perc_msg = road_object.getMsg();
  navigation_msgs::Junction msg;
  msg.header = perc_msg.header;
  msg.pose = perc_msg.pose;
  msg.id = tracking_element.getId();
  msg.junction_type = perc_msg.junction_type;
  msg.obstacle_waiting = road_object.getObstacleWaiting();
  msg.stopping_point = tf2::toMsg(road_object.getStoppingPoint());
  junctions.sub_messages.push_back(msg);
}

void CreateMessagesVisitor::visit(Crosswalk &road_object, TrackingElement &tracking_element) {
  if (!tracking_element.shouldPublishElement()) {
    return;
  }
  navigation_msgs::Crosswalk msg;
  msg.id = tracking_element.getId();
  msg.pose = road_object.getMsg().pose;
  msg.pedestrian_waiting = road_object.hasPedestrianWaiting();
  crosswalks.sub_messages.push_back(msg);
}

void CreateMessagesVisitor::visit(RoadClosure &road_object, TrackingElement &tracking_element) {
  if (!tracking_element.shouldPublishElement()) {
    return;
  }

  navigation_msgs::RoadClosure msg;
  msg.id = tracking_element.getId();
  msg.hull_polygon = road_object.getMsg().hull_polygon;
  if (msg.hull_polygon.size() != 4) {
    ROS_ERROR_STREAM(
        "invalid road closure message in CreateMessagesVisitor: road closure "
        "has "
        << msg.hull_polygon.size() << " vertices.");
    return;
  }
  msg.header = road_object.getMsg().header;
  msg.has_obstacle = road_object.getObstacleWaiting();
  road_closures.sub_messages.push_back(msg);
}

void CreateMessagesVisitor::visit(SpeedLimit &road_object, TrackingElement &tracking_element) {
  if (!tracking_element.shouldPublishElement()) {
    return;
  }
  auto msg = road_object.getMsg();
  if (msg.speed_limit == 0) {
    ROS_ERROR(
        "invalid speed limit message in CreateMessagesVisitor: speed_limit is "
        "0");
    return;
  }
  msg.id = tracking_element.getId();
  speed_limits.sub_messages.push_back(msg);
}

void CreateMessagesVisitor::visit(Arrow &road_object, TrackingElement &tracking_element) {
  if (!tracking_element.shouldPublishElement()) {
    return;
  }
  auto msg = road_object.getMsg();
  msg.id = tracking_element.getId();
  arrow_markings.sub_messages.push_back(msg);
}

void CreateMessagesVisitor::visit(StartLine &road_object, TrackingElement &tracking_element) {
  if (!tracking_element.shouldPublishElement()) {
    return;
  }
  auto msg = road_object.getMsg();
  msg.id = tracking_element.getId();
  start_lines.sub_messages.push_back(msg);
}

navigation_msgs::Obstacle CreateMessagesVisitor::toNavigationObstacle(
    const perception_msgs::Obstacle &perc_obstacle) {
  navigation_msgs::Obstacle nav_obstacle;
  nav_obstacle.id = perc_obstacle.id;
  std::vector<cv::Point2f> vertices;
  for (const auto &p : perc_obstacle.vertices) {
    vertices.emplace_back(p.x, p.y);
  }
  if (vertices.size() < 4) {
    ROS_WARN_STREAM_THROTTLE(
        1.0, "Can not transform obstacle: vertices.size() is " << vertices.size());
    return nav_obstacle;
  }
  const cv::RotatedRect rotated_rect = cv::minAreaRect(vertices);
  Eigen::Vector3d size;
  size.x() = rotated_rect.size.width;
  size.y() = rotated_rect.size.height;
  size.z() = rotated_rect.size.width;
  nav_obstacle.scale = tf2::toVector3Msg(size);

  Eigen::Affine3d pose(
      Eigen::Translation3d(rotated_rect.center.x, rotated_rect.center.y, 0.0) *
      Eigen::AngleAxisd(common::toRad(rotated_rect.angle), Eigen::Vector3d::UnitZ()));
  nav_obstacle.pose = tf2::toMsg(pose);

  return nav_obstacle;
}
}  // namespace environmental_model
