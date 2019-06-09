#include "road_watcher_node.h"
#include <common/macros.h>


THIRD_PARTY_HEADERS_BEGIN
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <dynamic_reconfigure/server.h>
#include "perception/RoadWatcherConfig.h"

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
THIRD_PARTY_HEADERS_END

#include "common/angle_conversions.h"
#include "common/node_creation_makros.h"

#include FANCY_DEBUG_INCLUDE("debug/road_watcher_node_debug.h")
#include "perception_types.h"
#include "vehicle_point_conversions.h"

const tf_helper::FrameIDs RoadWatcherNode::FRAME_IDS = {"world", "vehicle"};

const ParameterString<double> RoadWatcherNode::MAX_TF_LOOKUP_DURATION(
    "max_tf_lookup_duration");

RoadWatcherNode::RoadWatcherNode(const ros::NodeHandle& node_handle)
    : NodeBase(node_handle),
      tf2_buffer_(ros::Duration(30)),
      tf2_listener_(tf2_buffer_),
      tf_helper_(&tf2_buffer_, &parameter_handler_, FRAME_IDS),
      message_handler_(node_handle_),
      sync(image_raw_sub_, right_points_sub, middle_points_sub, left_points_sub, no_passing_points_sub, 10) {

  camera_transformation_ =
      std::make_unique<common::CameraTransformation>(&parameter_handler_);
  ego_vehicle_ = std::make_unique<EgoVehicle>(&parameter_handler_);
  road_watcher_ = std::make_unique<RoadWatcher>(
      &parameter_handler_, camera_transformation_.get(), ego_vehicle_.get());
  world_coordinates_helper_ =
      std::make_unique<road_object_detection::WorldCoordinatesHelper>(&tf_helper_);
  classification_ = std::make_unique<Classification>(&parameter_handler_,
                                                     camera_transformation_.get(),
                                                     ego_vehicle_.get(),
                                                     world_coordinates_helper_.get());
  sync.registerCallback(boost::bind(&RoadWatcherNode::handleImage, this, _1, _2, _3, _4, _5));

  parameter_handler_.addDynamicReconfigureServer<perception::RoadWatcherConfig>(node_handle_);
}

void RoadWatcherNode::startModule() {
  message_handler_.advertise();

  image_raw_sub_.subscribe(node_handle_, "image_raw", 10);
  right_points_sub.subscribe(node_handle_, "road_lane_right", 10);
  middle_points_sub.subscribe(node_handle_, "road_lane_middle", 10);
  left_points_sub.subscribe(node_handle_, "road_lane_left", 10);
  no_passing_points_sub.subscribe(node_handle_, "road_lane_middle_no_passing", 10);
}

void RoadWatcherNode::stopModule() {
  image_raw_sub_.unsubscribe();
  right_points_sub.unsubscribe();
  middle_points_sub.unsubscribe();
  left_points_sub.unsubscribe();
  no_passing_points_sub.unsubscribe();

  message_handler_.shutdown();
}

void RoadWatcherNode::handleImage(const sensor_msgs::ImageConstPtr& image_raw_msg,
                                  const nav_msgs::PathConstPtr& right_points,
                                  const nav_msgs::PathConstPtr& middle_points,
                                  const nav_msgs::PathConstPtr& left_points,
                                  const nav_msgs::PathConstPtr& no_passing_points) {
  ego_vehicle_->update();
  camera_transformation_->update();
  const auto& stamp = image_raw_msg->header.stamp;
  // update of world_coordinates_helper

  tf_helper_.update(stamp);
  cv_bridge::CvImageConstPtr cv_ptr_raw;
  try {
    cv_ptr_raw = cv_bridge::toCvShare(image_raw_msg, sensor_msgs::image_encodings::MONO8);
  } catch (const cv_bridge::Exception& e) {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }
  const auto points = createLineVehiclePoints(
      right_points, middle_points, left_points, no_passing_points);
  std::vector<FeaturePointCluster> clusters;
  common::DynamicPolynomial middle_polynomial;
  road_watcher_->scanRoadAndFindClusters(cv_ptr_raw->image, points, clusters, middle_polynomial);
  road_object_detection::RoadObjects classifications = classification_->classify(
      clusters, cv_ptr_raw->image, stamp, middle_polynomial, points);
  world_coordinates_helper_->calcWorldCoordinates(classifications);

  message_handler_.publishRoadObjects(classifications, stamp);
}


const std::string RoadWatcherNode::getName() {
  return std::string("road_watcher");
}

CREATE_NODE_WITH_FANCY_DEBUG(RoadWatcherNode, RoadWatcherNodeDebug)
