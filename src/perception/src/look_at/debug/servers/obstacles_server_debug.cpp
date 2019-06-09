#include "obstacles_server_debug.h"
#include "../../../road_object_detection/debug/classifiers/obstacle_classifier_new_debug.h"
#include "../look_at_debug.h"

THIRD_PARTY_HEADERS_BEGIN
#include <geometry_msgs/PolygonStamped.h>
THIRD_PARTY_HEADERS_END

namespace look_at {
ObstaclesServerDebug::ObstaclesServerDebug(const ros::NodeHandle &node_handle,
                                           ParameterInterface *parameters,
                                           const std::string &action_name)
    : DebugImagesPublisher(node_handle, parameters),
      ObstaclesServer(node_handle, parameters, action_name) {
  classifier_ = std::make_unique<road_object_detection::ObstacleClassifierNewDebug>(
      camera_transformation_.get(), parameters, &debug_images);
  look_at_ = std::make_unique<LookAtDebug>(std::move(*look_at_), &debug_images);
}

void ObstaclesServerDebug::regionsOfInterestCallback(
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

  ObstaclesServer::regionsOfInterestCallback(
      image_msg, right_points, middle_points, left_points, no_passing_points);
  publishDebugImages(regions_to_classify_, cv_ptr_debug->header.stamp);

  // publish field of vision, this need not be added by other debug-servers
  geometry_msgs::PolygonStamped field_of_vision_msg;
  field_of_vision_msg.header.stamp = image_msg->header.stamp;
  field_of_vision_msg.header.frame_id = "vehicle";

  const auto fov = look_at_->getFieldOfVision();
  for (const auto &p : fov) {
    geometry_msgs::Point32 p_msg;
    p_msg.x = p.x;
    p_msg.y = p.y;
    p_msg.z = 0.0;
    field_of_vision_msg.polygon.points.push_back(p_msg);
  }
  rospub_field_of_vision_.publish(field_of_vision_msg);
}

void ObstaclesServerDebug::advertise() {
  rospub_field_of_vision_ =
      nh_.advertise<geometry_msgs::PolygonStamped>("field_of_vision", 1);
  DebugImagesPublisher::advertise(action_name_);
  ObstaclesServer::advertise();
}
void ObstaclesServerDebug::shutdown() {
  rospub_field_of_vision_.shutdown();
  DebugImagesPublisher::shutdown();
  ObstaclesServer::shutdown();
}
}  // namespace look_at
