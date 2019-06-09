#include "debug_images_publisher.h"

#include "common/angle_conversions.h"

THIRD_PARTY_HEADERS_BEGIN
#include <perception_msgs/LookAtRegionsDebug.h>
THIRD_PARTY_HEADERS_END

namespace look_at {

const ParameterString<double> DebugImagesPublisher::PIXEL_WIDTH("pixel_width");
const ParameterString<double> DebugImagesPublisher::OFFSET_BEFORE_CENTER(
    "offset_before_center");
const ParameterString<double> DebugImagesPublisher::OFFSET_AFTER_CENTER(
    "offset_after_center");
const ParameterString<double> DebugImagesPublisher::OFFSET_RIGHT_OF_CENTER(
    "offset_right_of_center");
const ParameterString<double> DebugImagesPublisher::OFFSET_LEFT_OF_CENTER(
    "offset_left_of_center");

DebugImagesPublisher::DebugImagesPublisher(const ros::NodeHandle &node_handle,
                                           ParameterInterface *parameters)
    : node_handle_(node_handle), parameters_ptr_(parameters) {
  parameters->registerParam(PIXEL_WIDTH);
  parameters->registerParam(OFFSET_BEFORE_CENTER);
  parameters->registerParam(OFFSET_AFTER_CENTER);
  parameters->registerParam(OFFSET_RIGHT_OF_CENTER);
  parameters->registerParam(OFFSET_LEFT_OF_CENTER);
}

void DebugImagesPublisher::advertise(const std::string &action_name) {
  rospub_img_debug_ =
      node_handle_.advertise<sensor_msgs::Image>(action_name + "_img_debug", 1);
  rospub_birdsview_debug_ = node_handle_.advertise<sensor_msgs::Image>(
      action_name + "_birdsview_debug", 1);
  rospub_canny_debug_ =
      node_handle_.advertise<sensor_msgs::Image>(action_name + "_canny_debug", 1);
  rospub_area_of_interest_ = node_handle_.advertise<perception_msgs::LookAtRegionsDebug>(
      action_name + "_areas_of_interest", 1);
  rospub_img_patch_ =
      node_handle_.advertise<sensor_msgs::Image>(action_name + "_img_patch", 1);
  ROS_DEBUG("advertising debug_images for %s-server.", action_name.c_str());
}

void DebugImagesPublisher::shutdown() {
  rospub_img_debug_.shutdown();
  rospub_birdsview_debug_.shutdown();
  rospub_canny_debug_.shutdown();
  rospub_area_of_interest_.shutdown();
}

void DebugImagesPublisher::publishDebugImages(const RegionsToClassify &rois,
                                              const ros::Time &stamp) {

  ROS_DEBUG("debug_images size is (%d,%d)",
            debug_images.getCameraImage()->size().height,
            debug_images.getCameraImage()->size().width);

  cv_bridge::CvImage debug_msg;
  debug_msg.header = std_msgs::Header();
  debug_msg.encoding = sensor_msgs::image_encodings::BGR8;
  debug_msg.image = *debug_images.getCameraImage();
  rospub_img_debug_.publish(debug_msg.toImageMsg());

  // compose birdsview patches
  double pixel_width = parameters_ptr_->getParam(PIXEL_WIDTH);  // meters
  double offset_before_center = parameters_ptr_->getParam(OFFSET_BEFORE_CENTER);  // meters
  double offset_after_center = parameters_ptr_->getParam(OFFSET_AFTER_CENTER);  // meters
  double offset_right_of_center = parameters_ptr_->getParam(OFFSET_RIGHT_OF_CENTER);  // meters
  double offset_left_of_center = parameters_ptr_->getParam(OFFSET_LEFT_OF_CENTER);  // meters
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

  // publish canny image
  cv_bridge::CvImage canny_debug_msg;
  canny_debug_msg.header = std_msgs::Header();
  canny_debug_msg.encoding = sensor_msgs::image_encodings::BGR8;
  canny_debug_msg.image = cannys;
  rospub_canny_debug_.publish(canny_debug_msg.toImageMsg());

  // publish rois
  perception_msgs::LookAtRegionsDebug debug_rois;
  debug_rois.header.stamp = stamp;
  debug_rois.header.frame_id = "world";
  debug_rois.regions.reserve(rois.size());
  for (const auto &roi : rois) {
    perception_msgs::LookAtRegionDebug actual;
    actual.id = roi.id;
    std::array<cv::Point2f, 4> points = getROIInWorld(roi);

    for (const cv::Point2f &p : points) {
      geometry_msgs::Point32 p_msg;
      p_msg.x = p.x;
      p_msg.y = p.y;
      actual.polygon.points.push_back(p_msg);
    }

    debug_rois.regions.push_back(actual);
  }
  rospub_area_of_interest_.publish(debug_rois);

  // publish image patches
  height = debug_images.getCameraImage()->size().height / 2;
  width = debug_images.getCameraImage()->size().width / 3;

  ROS_DEBUG("width is %d", width);

  cv::Mat image_patches = cv::Mat::zeros(height * 2, width * 3, CV_8UC3);
  for (unsigned int i = 0; i < debug_images.getImagePatches()->size() && i < 6; i++) {
    const cv::Mat current = debug_images.getImagePatches()->at(i);
    const cv::Point p((i % 3) * width, (i / 3) * height);
    const cv::Rect debug_rect =
        cv::Rect(p.x, p.y, current.size().width, current.size().height);
    ROS_DEBUG("size of current debug_rect with id #%d is (%d,%d)",
              i,
              debug_rect.size().height,
              debug_rect.size().width);

    // hier debug rect size wenn gleich wie image dann nimm image
    cv::Mat from = debug_rect.size().width == current.size().width &&
                           debug_rect.size().height == current.size().height
                       ? current
                       : current(cv::Rect(cv::Point(0, 0), debug_rect.size()));
    ROS_DEBUG("width of from is %d", from.size().width);
    cv::Mat rescaled;
    const double scale =
        static_cast<double>(from.size().width) / static_cast<double>(width);
    ROS_DEBUG("scale is %f", scale);
    if (scale > 1) {
      cv::resize(from, rescaled, cv::Size(width, height));
      ROS_DEBUG(
          "size of rescaled is (%d,%d)", rescaled.size().width, rescaled.size().height);
      const cv::Rect to(p, rescaled.size());
      rescaled.copyTo(image_patches(to));
    } else {
      from.copyTo(image_patches(cv::Rect(p, from.size())));
    }
  }

  cv_bridge::CvImage image_patches_debug_msg;
  image_patches_debug_msg.header = std_msgs::Header();
  image_patches_debug_msg.encoding = sensor_msgs::image_encodings::BGR8;
  image_patches_debug_msg.image = image_patches;
  rospub_img_patch_.publish(image_patches_debug_msg.toImageMsg());
}

std::array<cv::Point2f, 4> look_at::DebugImagesPublisher::getROIInWorld(
    const RegionToClassify &roi_in_world) const {
  Eigen::Affine3f as_float = roi_in_world.pose.cast<float>();
  const Eigen::Matrix3f rotation = as_float.linear();
  const cv::RotatedRect roi_as_rect{
      cv::Point2f{as_float.translation().x(), as_float.translation().y()},
      cv::Size2f{static_cast<float>(roi_in_world.width),
                 static_cast<float>(roi_in_world.height)},
      common::toDegree(common::toYaw(rotation))};

  std::array<cv::Point2f, 4> ret;
  roi_as_rect.points(ret.data());
  return ret;
};

}  // namespace look_at
