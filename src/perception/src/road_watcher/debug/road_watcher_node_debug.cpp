#include "road_watcher_node_debug.h"
#include "classification_debug.h"
#include "road_watcher_debug.h"

#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PolygonStamped.h>

using namespace road_object_detection;

const ParameterString<double> RoadWatcherNodeDebug::PIXEL_WIDTH("pixel_width");
const ParameterString<double> RoadWatcherNodeDebug::OFFSET_BEFORE_CENTER(
    "offset_before_center");
const ParameterString<double> RoadWatcherNodeDebug::OFFSET_AFTER_CENTER(
    "offset_after_center");
const ParameterString<double> RoadWatcherNodeDebug::OFFSET_RIGHT_OF_CENTER(
    "offset_right_of_center");
const ParameterString<double> RoadWatcherNodeDebug::OFFSET_LEFT_OF_CENTER(
    "offset_left_of_center");

const ParameterString<int> RoadWatcherNodeDebug::EXPAND_X("expand_x");
const ParameterString<int> RoadWatcherNodeDebug::EXPAND_Y("expand_y");

RoadWatcherNodeDebug::RoadWatcherNodeDebug(ros::NodeHandle &node_handle)
    : RoadWatcherNode(node_handle) {
  parameter_handler_.registerParam(PIXEL_WIDTH);
  parameter_handler_.registerParam(OFFSET_BEFORE_CENTER);
  parameter_handler_.registerParam(OFFSET_AFTER_CENTER);
  parameter_handler_.registerParam(OFFSET_RIGHT_OF_CENTER);
  parameter_handler_.registerParam(OFFSET_LEFT_OF_CENTER);
  parameter_handler_.registerParam(EXPAND_X);
  parameter_handler_.registerParam(EXPAND_Y);

  road_watcher_ = std::make_unique<RoadWatcherDebug>(
      std::move(*road_watcher_), debug_images.getCameraImage());
  classification_ =
      std::make_unique<ClassificationDebug>(std::move(*classification_),
                                            &parameter_handler_,
                                            camera_transformation_.get(),
                                            /*world_coordinates_helper_.get(),*/
                                            &debug_images);
}

void RoadWatcherNodeDebug::handleImage(const sensor_msgs::ImageConstPtr &image_raw_msg,
                                       const nav_msgs::PathConstPtr &msg_right_points,
                                       const nav_msgs::PathConstPtr &msg_middle_points,
                                       const nav_msgs::PathConstPtr &msg_left_points,
                                       const nav_msgs::PathConstPtr &no_passing_points) {
  // bridge image_raw ROS to CV message
  cv_bridge::CvImageConstPtr cv_ptr_raw;
  try {
    cv_ptr_raw = cv_bridge::toCvShare(image_raw_msg, sensor_msgs::image_encodings::MONO8);
  } catch (cv_bridge::Exception &e) {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }

  debug_images.clear();
  debug_images.setCameraImage(cv_ptr_raw->image);

  RoadWatcherNode::handleImage(
      image_raw_msg, msg_right_points, msg_middle_points, msg_left_points, no_passing_points);
  cv_bridge::CvImage debug_msg;
  debug_msg.header = std_msgs::Header();
  debug_msg.encoding = sensor_msgs::image_encodings::BGR8;
  debug_msg.image = *debug_images.getCameraImage();
  rospub_img_debug_.publish(debug_msg.toImageMsg());

  // compose birdsview patches
  // All in meters!!!
  const double pixel_width = parameter_handler_.getParam(PIXEL_WIDTH);
  const double offset_before_center = parameter_handler_.getParam(OFFSET_BEFORE_CENTER);
  const double offset_after_center = parameter_handler_.getParam(OFFSET_AFTER_CENTER);
  const double offset_right_of_center = parameter_handler_.getParam(OFFSET_RIGHT_OF_CENTER);
  const double offset_left_of_center = parameter_handler_.getParam(OFFSET_LEFT_OF_CENTER);
  int height = static_cast<int>(
      std::round((offset_before_center + offset_after_center) / pixel_width));
  int width = static_cast<int>(
      std::round((offset_left_of_center + offset_right_of_center) / pixel_width));

  cv::Mat birdsviews = cv::Mat::zeros(height * 2, width * 3, CV_8UC3);
  for (unsigned int i = 0; i < debug_images.getBirdsviewPatches()->size() && i < 6; i++) {
    debug_images.getBirdsviewPatches()->at(i).copyTo(
        birdsviews(cv::Rect((i % 3) * width, (i / 3) * height, width, height)));
  }

  cv_bridge::CvImage birdsview_debug_msg;
  birdsview_debug_msg.header = std_msgs::Header();
  birdsview_debug_msg.encoding = sensor_msgs::image_encodings::BGR8;
  birdsview_debug_msg.image = birdsviews;
  rospub_birdsview_debug_.publish(birdsview_debug_msg.toImageMsg());

  // compose canny patches
  height = debug_images.getCameraImage()->rows / 2;
  width = debug_images.getCameraImage()->cols / 3;

  cv::Mat cannys = cv::Mat::zeros(height * 2, width * 3, CV_8UC3);
  for (unsigned int i = 0; i < debug_images.getCannyPatches()->size() && i < 6; i++) {
    const cv::Point p((i % 3) * width, (i / 3) * height);
    const cv::Rect debug_rect =
        cv::Rect(p, debug_images.getCannyPatches()->at(i).size()) &
        cv::Rect(cv::Point(0, 0), cannys.size());

    cv::Mat from = debug_images.getCannyPatches()->at(i)(
        cv::Rect(cv::Point(0, 0), debug_rect.size()));
    from.copyTo(cannys(debug_rect));
  }


  cv_bridge::CvImage canny_debug_msg;
  canny_debug_msg.header = std_msgs::Header();
  canny_debug_msg.encoding = sensor_msgs::image_encodings::BGR8;
  canny_debug_msg.image = cannys;
  rospub_canny_debug_.publish(canny_debug_msg.toImageMsg());

  geometry_msgs::PolygonStamped field_of_vision_msg;
  field_of_vision_msg.header.stamp = image_raw_msg->header.stamp;
  field_of_vision_msg.header.frame_id = "vehicle";
  for (const auto &p : road_watcher_->getFieldOfVision()) {
    geometry_msgs::Point32 p_msg;
    p_msg.x = p.x;
    p_msg.y = p.y;
    p_msg.z = 0.0;
    field_of_vision_msg.polygon.points.push_back(p_msg);
  }
  rospub_field_of_vision_.publish(field_of_vision_msg);


  height = debug_images.getCameraImage()->rows / 2;
  width = debug_images.getCameraImage()->cols / 3;

  cv::Mat image_patches = cv::Mat::zeros(height * 2, width * 3, CV_8UC3);
  for (unsigned int i = 0; i < debug_images.getImagePatches()->size() && i < 6; i++) {
    const cv::Point p((i % 3) * width, (i / 3) * height);
    const cv::Rect debug_rect =
        cv::Rect(p, debug_images.getImagePatches()->at(i).size()) &
        cv::Rect(cv::Point(0, 0), image_patches.size());

    cv::Mat from = debug_images.getImagePatches()->at(i)(
        cv::Rect(cv::Point(0, 0), debug_rect.size()));
    from.copyTo(image_patches(debug_rect));
  }

  cv_bridge::CvImage image_patches_debug_msg;
  image_patches_debug_msg.header = std_msgs::Header();
  image_patches_debug_msg.encoding = sensor_msgs::image_encodings::BGR8;
  image_patches_debug_msg.image = image_patches;
  rospub_img_patch_.publish(image_patches_debug_msg.toImageMsg());
}

void RoadWatcherNodeDebug::startModule() {
  RoadWatcherNode::startModule();
  rospub_img_debug_ = node_handle_.advertise<sensor_msgs::Image>("img_debug", 1);
  rospub_birdsview_debug_ =
      node_handle_.advertise<sensor_msgs::Image>("birdsview_debug", 1);
  rospub_canny_debug_ =
      node_handle_.advertise<sensor_msgs::Image>("canny_debug", 1);

  rospub_field_of_vision_ =
      node_handle_.advertise<geometry_msgs::PolygonStamped>("field_of_vision", 1);
  rospub_img_patch_ = node_handle_.advertise<sensor_msgs::Image>("img_patch", 1);
}

void RoadWatcherNodeDebug::stopModule() {
  RoadWatcherNode::stopModule();
  rospub_img_debug_.shutdown();
  rospub_birdsview_debug_.shutdown();
  rospub_canny_debug_.shutdown();
  rospub_field_of_vision_.shutdown();
}
