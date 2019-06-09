#include "pedestrian_server_debug.h"

#include "../../../road_object_detection/debug/classifiers/pedestrian_classifier_debug.h"
#include "../look_at_debug.h"

THIRD_PARTY_HEADERS_BEGIN

THIRD_PARTY_HEADERS_END

namespace look_at {

PedestrianServerDebug::PedestrianServerDebug(const ros::NodeHandle &node_handle,
                                             common::node_base::ParameterInterface *parameters,
                                             const std::string &action_name)
    : PedestrianServer(node_handle, parameters, action_name),
      DebugImagesPublisher(node_handle, parameters) {
  classifier_ = std::make_unique<road_object_detection::PedestrianClassifierDebug>(
      camera_transformation_.get(), parameters, &debug_images);
  look_at_ = std::make_unique<LookAtDebug>(std::move(*look_at_), &debug_images);
}

void PedestrianServerDebug::regionsOfInterestCallback(
    const sensor_msgs::ImageConstPtr &image_msg,
    const nav_msgs::PathConstPtr &right_points,
    const nav_msgs::PathConstPtr &middle_points,
    const nav_msgs::PathConstPtr &left_points,
    const nav_msgs::PathConstPtr &no_passing_points) {
  cv_bridge::CvImageConstPtr cv_ptr_debug;
  try {
    cv_ptr_debug = cv_bridge::toCvShare(image_msg, sensor_msgs::image_encodings::MONO8);
  } catch (const cv_bridge::Exception &e) {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }

  debug_images.clear();
  debug_images.setCameraImage(cv_ptr_debug->image);

  PedestrianServer::regionsOfInterestCallback(
      image_msg, right_points, middle_points, left_points, no_passing_points);
  publishDebugImages(regions_to_classify_, cv_ptr_debug->header.stamp);
}

void PedestrianServerDebug::advertise() {
  DebugImagesPublisher::advertise(action_name_);
  PedestrianServer::advertise();
}

void PedestrianServerDebug::shutdown() {
  DebugImagesPublisher::shutdown();
  PedestrianServer::shutdown();
}

}  // namespace look_at
