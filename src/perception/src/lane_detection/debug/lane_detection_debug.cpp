#include <common/macros.h>

THIRD_PARTY_HEADERS_BEGIN
#include <nav_msgs/Path.h>
#include <ros/ros.h>
THIRD_PARTY_HEADERS_END

#include "lane_detection_debug.h"
#include "lane_detection_node_debug.h"
#include "common/path_conversion.h"


LaneDetectionDebug::LaneDetectionDebug(LaneDetection &&lane_detection,
                                       LaneDetectionNodeDebug *lane_detection_node_debug)
    : LaneDetection(std::move(lane_detection)),
      lane_detection_node_debug(lane_detection_node_debug),
      publishers(lane_detection_node_debug->getLaneDetectionDebugPublishers()) {}


LineVehiclePoints LaneDetectionDebug::processImage(const cv::Mat &image) {
  const LineVehiclePoints out_line_data = LaneDetection::processImage(image);

  publishers.rospub_polynomial_lane_left.publish(createLaneMsg(LINESPEC_LEFT));
  publishers.rospub_polynomial_lane_middle.publish(createLaneMsg(LINESPEC_MIDDLE));
  publishers.rospub_polynomial_lane_right.publish(createLaneMsg(LINESPEC_RIGHT));

  return out_line_data;
}

void LaneDetectionDebug::showProjectionPoints(const LineSpec &target,
                                              const VehiclePoints &points) {
  const nav_msgs::Path path = common::createPathMsg(
      points, lane_detection_node_debug->getTimeStamp(), "vehicle");

  switch (target) {
    case LINESPEC_LEFT:
      publishers.rospub_left_proj_points.publish(path);
      break;
    case LINESPEC_MIDDLE:
      publishers.rospub_middle_proj_points.publish(path);
      break;
    case LINESPEC_RIGHT:
      publishers.rospub_right_proj_points.publish(path);
      break;
    case LINESPEC_N:
    case LINESPEC_NO_PASSING:
      ROS_INFO("wrong linespec (LINESPEC_N/LINESPEC_NO_PASSING)!");
      break;
  }
}

perception_msgs::Lane LaneDetectionDebug::createLaneMsg(const LineSpec &line_type) const {
  perception_msgs::Lane lane;
  lane.header.frame_id = "vehicle";
  lane.header.stamp = lane_detection_node_debug->getTimeStamp();
  if (lane_model_[line_type]) {
    lane.parameters = lane_model_.at(line_type)->getCoefficients();
  }
  return lane;
}

void LaneDetectionDebug::Publishers::startPublishers(ros::NodeHandle &node_handle) {
  rospub_left_proj_points =
      node_handle.advertise<nav_msgs::Path>("debug/left_shifted_points", 1);
  rospub_middle_proj_points =
      node_handle.advertise<nav_msgs::Path>("debug/middle_shifted_points", 1);
  rospub_right_proj_points =
      node_handle.advertise<nav_msgs::Path>("debug/right_shifted_points", 1);
  rospub_polynomial_lane_left =
      node_handle.advertise<perception_msgs::Lane>("debug/lane_polynomial_left", 1);
  rospub_polynomial_lane_middle = node_handle.advertise<perception_msgs::Lane>(
      "debug/lane_polynomial_middle", 1);
  rospub_polynomial_lane_right = node_handle.advertise<perception_msgs::Lane>(
      "debug/lane_polynomial_right", 1);
}


void LaneDetectionDebug::Publishers::stopPublishers() {
  rospub_left_proj_points.shutdown();
  rospub_middle_proj_points.shutdown();
  rospub_right_proj_points.shutdown();
  rospub_polynomial_lane_left.shutdown();
  rospub_polynomial_lane_middle.shutdown();
  rospub_polynomial_lane_right.shutdown();
}
