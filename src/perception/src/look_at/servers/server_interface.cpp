#include "server_interface.h"

#include "vehicle_point_conversions.h"

THIRD_PARTY_HEADERS_BEGIN
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <cv_bridge/cv_bridge.h>
THIRD_PARTY_HEADERS_END

#include "common/angle_conversions.h"

namespace look_at {

const tf_helper::FrameIDs ServerInterface::FRAME_IDS = {"world", "vehicle"};

ServerInterface::ServerInterface(const ros::NodeHandle &node_handle, ParameterInterface *parameters)
    : nh_(node_handle),
      tf_helper_(&tf2_buffer_, parameters, FRAME_IDS),
      sync_(image_raw_sub_, right_points_sub_, middle_points_sub_, left_points_sub_, no_passing_points_sub_, 10),
      tf2_buffer_(ros::Duration(30)),
      tf2_listener_(tf2_buffer_) {
  camera_transformation_ = std::make_unique<common::CameraTransformation>(parameters);
  ego_vehicle_ = std::make_unique<EgoVehicle>(parameters);
  world_coordinates_helper_ =
      std::make_unique<road_object_detection::WorldCoordinatesHelper>(&tf_helper_);
}

void ServerInterface::advertise() {
  image_raw_sub_.subscribe(nh_, "image_raw", 10);
  right_points_sub_.subscribe(nh_, "road_lane_right", 10);
  middle_points_sub_.subscribe(nh_, "road_lane_middle", 10);
  left_points_sub_.subscribe(nh_, "road_lane_left", 10);
  no_passing_points_sub_.subscribe(nh_, "road_lane_middle_no_passing", 10);
}

void ServerInterface::process(const sensor_msgs::ImageConstPtr &image_msg,
                              const nav_msgs::PathConstPtr &right_points,
                              const nav_msgs::PathConstPtr &middle_points,
                              const nav_msgs::PathConstPtr &left_points,
                              const nav_msgs::PathConstPtr &no_passing_points) {

  ego_vehicle_->update();
  camera_transformation_->update();
  const auto &stamp = image_msg->header.stamp;
  // update of world_coordinates_helper

  tf_helper_.update(stamp);

  cv_bridge::CvImageConstPtr cv_ptr_raw;
  try {
    cv_ptr_raw = cv_bridge::toCvShare(image_msg, sensor_msgs::image_encodings::MONO8);
  } catch (const cv_bridge::Exception &e) {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }
  const auto points = createLineVehiclePoints(
      right_points, middle_points, left_points, no_passing_points);

  classifications_ = look_at_->classifyROI(
      cv_ptr_raw->image, stamp, regions_to_classify_, points, classifier_);
}

void ServerInterface::shutdown() {
  image_raw_sub_.unsubscribe();
  right_points_sub_.unsubscribe();
  middle_points_sub_.unsubscribe();
  left_points_sub_.unsubscribe();
  no_passing_points_sub_.unsubscribe();
}

RegionsToClassify ServerInterface::extractROIs(const perception_msgs::LookAtRegions &regions) const {
  RegionsToClassify rois_to_classify;

  rois_to_classify.reserve(regions.regions.size());
  for (const auto &r : regions.regions) {
    // construct region
    RegionToClassify actual;
    tf2::fromMsg(r.pose, actual.pose);
    actual.height = r.rect.x;
    actual.width = r.rect.y;
    actual.id = static_cast<std::size_t>(r.id);
    // push_back into vector

    ROS_DEBUG_STREAM("position of roi: " << actual.pose.translation() << " ; orientation: "
                                         << common::toYaw(actual.pose.rotation()));

    rois_to_classify.push_back(actual);
  }

  return rois_to_classify;
}

}  // namespace look_at
