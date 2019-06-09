#include "perpendicular_parking_node.h"

#include "common/node_creation_makros.h"

THIRD_PARTY_HEADERS_BEGIN
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <cv_bridge/cv_bridge.h>
THIRD_PARTY_HEADERS_END

#include "vehicle_point_conversions.h"
#include FANCY_DEBUG_INCLUDE("debug/perpendicular_parking_node_debug.h")

const ParameterString<int> PerpendicularParkingNode::INPUT_QUEUE_SIZE(
    "input_queue_size");

const tf_helper::FrameIDs PerpendicularParkingNode::FRAME_IDS = {"world",
                                                                 "vehicle"};

uint32_t PerpendicularParkingNode::getInputQueueSize(ParameterInterface *const parameter_ptr) {
  parameter_ptr->registerParam(INPUT_QUEUE_SIZE);
  return static_cast<uint32_t>(parameter_ptr->getParam(INPUT_QUEUE_SIZE));
}

PerpendicularParkingNode::PerpendicularParkingNode(ros::NodeHandle &node_handle)
    : NodeBase(node_handle),
      tf2_buffer_(ros::Duration(30)),
      camera_transformation_(std::make_unique<common::CameraTransformation>(&parameter_handler_)),
      tf2_listener_(tf2_buffer_),
      world_coordinates_helper_(&tf2_buffer_, &parameter_handler_, FRAME_IDS),
      synchronizer(image_raw_sub,
                   lane_left_sub,
                   lane_middle_sub,
                   lane_right_sub,
                   lane_no_passing_sub,
                   getInputQueueSize(&parameter_handler_)) {
  perpendicular_parking_ = std::make_unique<perpendicular_parking::PerpendicularParking>(
      &world_coordinates_helper_, &parameter_handler_, camera_transformation_.get());

  synchronizer.registerCallback(boost::bind(
      &PerpendicularParkingNode::handleLeftLaneAndImage, this, _1, _2, _3, _4, _5));
}

void PerpendicularParkingNode::startModule() {

  // Publishers
  parking_spot_helper.advertise(node_handle_, "free_parking_spots");
  parking_lot_found_pub = node_handle_.advertise<perception_msgs::ParkingLotFound>(
      "parking_lot_found", 8);

  const uint32_t input_queue_size =
      static_cast<uint32_t>(parameter_handler_.getParam(INPUT_QUEUE_SIZE));

  // Subscribers, queue size can be set in launch file
  image_raw_sub.subscribe(node_handle_, "image_raw", input_queue_size);
  lane_left_sub.subscribe(node_handle_, "road_lane_left", input_queue_size);
  lane_middle_sub.subscribe(node_handle_, "road_lane_middle", input_queue_size);
  lane_right_sub.subscribe(node_handle_, "road_lane_right", input_queue_size);
  lane_no_passing_sub.subscribe(node_handle_, "road_lane_middle_no_passing", input_queue_size);

  parking_service = node_handle_.advertiseService(
      "parking_service", &PerpendicularParkingNode::activateService, this);
}

void PerpendicularParkingNode::stopModule() {
  parking_spot_helper.shutdown();
  parking_lot_found_pub.shutdown();

  image_raw_sub.unsubscribe();
  lane_left_sub.unsubscribe();
  lane_middle_sub.unsubscribe();
  lane_right_sub.unsubscribe();
  lane_no_passing_sub.unsubscribe();

  parking_service.shutdown();
}

void PerpendicularParkingNode::handleLeftLaneAndImage(
    const sensor_msgs::ImageConstPtr &image_raw_msg,
    const nav_msgs::PathConstPtr &left_lane_msg,
    const nav_msgs::PathConstPtr &middle_lane_msg,
    const nav_msgs::PathConstPtr &right_lane_msg,
    const nav_msgs::PathConstPtr &no_passing_lane_msg) {

  camera_transformation_->update();

  cv_bridge::CvImageConstPtr cv_ptr_image;
  try {
    cv_ptr_image =
        cv_bridge::toCvShare(image_raw_msg, sensor_msgs::image_encodings::MONO8);
  } catch (const cv_bridge::Exception &e) {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }

  const auto lanes = createLineVehiclePoints(
      right_lane_msg, middle_lane_msg, left_lane_msg, no_passing_lane_msg);

  // lower layers processing image and left lane
  auto free_parking_spots = perpendicular_parking_->findFreeParkingSpots(
      cv_ptr_image->image, lanes, cv_ptr_image->header.stamp);
  // creating submessages and adding them to parking_spot_helper
  perception_msgs::PerpendicularParkingSpots free_spots_msg =
      toPerpendicularParkingSpots(free_parking_spots);
  createMsg(free_spots_msg, cv_ptr_image->header.stamp);

  const perception_msgs::ParkingLotFound parking_lot_found = toParkingLotFound(
      perpendicular_parking_->isParkingLotDetected(), cv_ptr_image->header.stamp);

  // publishing message
  parking_spot_helper.publishMessage(
      parking_spot_helper.generateAndClearMessage(cv_ptr_image->header.stamp));
  parking_lot_found_pub.publish(parking_lot_found);
}

bool PerpendicularParkingNode::activateService(perception_msgs::SearchParkingSpots::Request &request,
                                               perception_msgs::SearchParkingSpots::Response &) {
  perpendicular_parking_->setActivationStatus(static_cast<bool>(request.search));
  return true;
}

void PerpendicularParkingNode::createMsg(perception_msgs::PerpendicularParkingSpots &free_parking_spots,
                                         const ros::Time &timestamp) {
  free_parking_spots.header.stamp = timestamp;
  for (auto &spot : free_parking_spots.sub_messages) {
    spot.header.stamp = timestamp;
    parking_spot_helper.addSubMessage(spot);
  }
}

const perception_msgs::PerpendicularParkingSpots PerpendicularParkingNode::toPerpendicularParkingSpots(
    perpendicular_parking::ParkingSpotsConstRef free_spots) {
  perception_msgs::PerpendicularParkingSpots perpendicular_parking_spots;
  perpendicular_parking_spots.header.frame_id = "world";
  perpendicular_parking_spots.sub_messages.reserve(free_spots.size());
  for (const auto &free_spot : free_spots) {
    perception_msgs::PerpendicularParkingSpot spot = free_spot.get().asMsg();

    spot.header.frame_id = "world";

    Eigen::Affine3d left_entrance, right_entrance;

    tf2::fromMsg(spot.entrance_pose_left, left_entrance);
    tf2::fromMsg(spot.entrance_pose_right, right_entrance);

    spot.entrance_pose_left = tf2::toMsg(perpendicular_parking_->mapPose() * left_entrance);
    spot.entrance_pose_right =
        tf2::toMsg(perpendicular_parking_->mapPose() * right_entrance);

    perpendicular_parking_spots.sub_messages.push_back(spot);
  }
  return perpendicular_parking_spots;
}

const perception_msgs::ParkingLotFound PerpendicularParkingNode::toParkingLotFound(
    const bool found, const ros::Time &timestamp) {
  perception_msgs::ParkingLotFound found_as_msg;
  found_as_msg.header.stamp = timestamp;
  found_as_msg.found.data = static_cast<std_msgs::Bool::_data_type>(found);
  return found_as_msg;
}



const std::string PerpendicularParkingNode::getName() {
  return std::string("perpendicular_parking");
}

CREATE_NODE_WITH_FANCY_DEBUG(PerpendicularParkingNode, perpendicular_parking::PerpendicularParkingNodeDebug)
