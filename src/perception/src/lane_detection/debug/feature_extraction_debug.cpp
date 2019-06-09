#include "feature_extraction_debug.h"

#include "perception_msgs/ScanLines.h"
#include "perception_msgs/Pixel.h"
#include "opencv_eigen_conversions.h"
#include "perception_message_conversions.h"

#include "lane_detection_node_debug.h"

using namespace perception::message_conversion;

void FeatureExtractionDebug::Publishers::startPublishers(ros::NodeHandle &node_handle) {
  rospub_scan_lines_left =
      node_handle.advertise<perception_msgs::ScanLines>("debug/scan_lines_left", 1);
  rospub_scan_lines_middle = node_handle.advertise<perception_msgs::ScanLines>(
      "debug/scan_lines_middle", 1);
  rospub_scan_lines_right = node_handle.advertise<perception_msgs::ScanLines>(
      "debug/scan_lines_right", 1);
  rospub_feature_points_left = node_handle.advertise<perception_msgs::Pixels>(
      "debug/feature_points_left", 1);
  rospub_feature_points_middle = node_handle.advertise<perception_msgs::Pixels>(
      "debug/feature_points_middle", 1);
  rospub_feature_points_right = node_handle.advertise<perception_msgs::Pixels>(
      "debug/feature_points_right", 1);
  rospub_middle_points_left =
      node_handle.advertise<perception_msgs::Pixels>("debug/middle_points_left", 1);
  rospub_middle_points_right = node_handle.advertise<perception_msgs::Pixels>(
      "debug/middle_points_right", 1);
}

void FeatureExtractionDebug::Publishers::stopPublishers() {
  rospub_scan_lines_left.shutdown();
  rospub_scan_lines_middle.shutdown();
  rospub_scan_lines_right.shutdown();
  rospub_feature_points_left.shutdown();
  rospub_feature_points_middle.shutdown();
  rospub_feature_points_right.shutdown();
  rospub_middle_points_left.shutdown();
  rospub_middle_points_right.shutdown();
}

FeatureExtractionDebug::FeatureExtractionDebug(FeatureExtraction &&feature_extraction,
                                               LaneDetectionNodeDebug *lane_detection_node_debug)
    : FeatureExtraction(std::move(feature_extraction)),
      lane_detection_node_debug(lane_detection_node_debug),
      publishers(lane_detection_node_debug->getFeatureExtractionDebugPublishers()),
      line_type(LINESPEC_LEFT),
      publish_scanlines(false) {}

LineVehiclePoints FeatureExtractionDebug::extractLineMarkings(const cv::Mat &image) const {
  publish_scanlines = true;
  line_type = LINESPEC_LEFT;
  const auto line_points = FeatureExtraction::extractLineMarkings(image);
  publish_scanlines = false;
  return line_points;
}

LineVehiclePoints FeatureExtractionDebug::extractLineMarkings(const cv::Mat &image,
                                                              const LaneModel &line_data) const {
  publish_scanlines = true;
  line_type = LINESPEC_LEFT;
  const auto line_points = FeatureExtraction::extractLineMarkings(image, line_data);
  publish_scanlines = false;
  return line_points;
}


ScanLines FeatureExtractionDebug::createScanLines(const common::DynamicPolynomial &line) const {
  const ScanLines scan_lines = FeatureExtraction::createScanLines(line);
  publishScanLines(scan_lines);
  return scan_lines;
}

ScanLines FeatureExtractionDebug::createScanLines(const VehiclePoints &ground_points) const {
  const ScanLines scan_lines = FeatureExtraction::createScanLines(ground_points);
  publishScanLines(scan_lines);
  return scan_lines;
}

void FeatureExtractionDebug::publishScanLines(const ScanLines &scan_lines) const {
  if (!publish_scanlines) {
    return;
  }

  perception_msgs::ScanLines scan_lines_msg;
  scan_lines_msg.header.stamp = lane_detection_node_debug->getTimeStamp();
  scan_lines_msg.scanlines.reserve(scan_lines.size());
  for (const ScanLine &scan_line : scan_lines) {
    perception_msgs::ScanLine scan_line_msg;
    scan_line_msg.left = toMsg(scan_line.start + ImagePoint(roi.x, roi.y));
    scan_line_msg.right = toMsg(scan_line.end + ImagePoint(roi.x, roi.y));
    scan_lines_msg.scanlines.push_back(scan_line_msg);
  }

  switch (line_type) {
    case LINESPEC_LEFT:
      publishers.rospub_scan_lines_left.publish(scan_lines_msg);
      break;
    case LINESPEC_MIDDLE:
      publishers.rospub_scan_lines_middle.publish(scan_lines_msg);
      break;
    case LINESPEC_RIGHT:
      publishers.rospub_scan_lines_right.publish(scan_lines_msg);
      break;
    case LINESPEC_N:
    case LINESPEC_NO_PASSING:
      ROS_ERROR("Unsupported LineSpec!");
  }
}


ImagePoints FeatureExtractionDebug::extractImagePointsByScanLines(
    const cv::Mat &image,
    const ScanLines &scan_lines,
    const boost::optional<common::DynamicPolynomial> &ref_line,
    const boost::optional<LineSpec> l_spec) const {
  const auto image_points = FeatureExtraction::extractImagePointsByScanLines(
      image, scan_lines, ref_line, l_spec);

  perception_msgs::Pixels pixels_msgs;
  pixels_msgs.header.stamp = lane_detection_node_debug->getTimeStamp();
  pixels_msgs.pixels.reserve(image_points.size());
  for (const ImagePoint &image_point : image_points) {
    pixels_msgs.pixels.push_back(toMsg(image_point + ImagePoint(roi.x, roi.y)));
  }

  switch (line_type) {
    case LINESPEC_LEFT:
      publishers.rospub_feature_points_left.publish(pixels_msgs);
      line_type = LINESPEC_RIGHT;
      break;
    case LINESPEC_MIDDLE:
      publishers.rospub_feature_points_middle.publish(pixels_msgs);
      line_type = LINESPEC_LEFT;
      break;
    case LINESPEC_RIGHT:
      publishers.rospub_feature_points_right.publish(pixels_msgs);
      line_type = LINESPEC_MIDDLE;
      break;
    case LINESPEC_N:
    case LINESPEC_NO_PASSING:
      ROS_ERROR("Unsupported LineSpec LINESPEC_N!");
  }
  return image_points;
}

bool FeatureExtractionDebug::isDoubleLine(const cv::Mat &image,
                                          const ScanLines &lines,
                                          VehiclePoints *left_line,
                                          VehiclePoints *right_line) {
  const bool result = FeatureExtraction::isDoubleLine(image, lines, left_line, right_line);

  ImagePoints left, right;
  view_transform->transformGroundToImage(*left_line, &left);
  view_transform->transformGroundToImage(*right_line, &right);

  publishers.rospub_middle_points_left.publish(
      toMsg(left, lane_detection_node_debug->getTimeStamp()));
  publishers.rospub_middle_points_right.publish(
      toMsg(right, lane_detection_node_debug->getTimeStamp()));
  return result;
}
