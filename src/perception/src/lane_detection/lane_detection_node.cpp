#include "lane_detection_node.h"
#include <common/macros.h>

THIRD_PARTY_HEADERS_BEGIN
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <tf2_eigen/tf2_eigen.h>
THIRD_PARTY_HEADERS_END

// Kitcar Libraries
#include "common/node_creation_makros.h"
#include "common/path_conversion.h"
#include "common/msg_helper.h"

// Internal Includes
#include "roi_birds_view_transformation.h"
#include "feature_extraction.h"
#include "perception_message_conversions.h"
#include "opencv_eigen_conversions.h"

// Needed for fancy debug
#include FANCY_DEBUG_INCLUDE("debug/lane_detection_node_debug.h")

using namespace perception::message_conversion;

const ParameterString<int> LaneDetectionNode::INPUT_QUEUE_SIZE(
    "input_queue_size");
const ParameterString<int> LaneDetectionNode::OUTPUT_QUEUE_SIZE(
    "output_queue_size");

LaneDetectionNode::LaneDetectionNode(ros::NodeHandle& node_handle)
    : NodeBase(node_handle), timesync_img(image_sub, roi_sub, 1) {
  roi_view_transformation =
      std::make_unique<ROIBirdsViewTransformation>(&parameter_handler_);
  ego_vehicle_ = std::make_unique<EgoVehicle>(&parameter_handler_);
  feature_extraction_ = std::make_unique<FeatureExtraction>(
      &parameter_handler_, roi_view_transformation.get(), ego_vehicle_.get());
  lane_detection_ = std::make_unique<LaneDetection>(&parameter_handler_,
                                                    feature_extraction_.get());

  timesync_img.registerCallback(boost::bind(&LaneDetectionNode::handleImage, this, _1, _2));

  parameter_handler_.registerParam(INPUT_QUEUE_SIZE);
  parameter_handler_.registerParam(OUTPUT_QUEUE_SIZE);
}

void LaneDetectionNode::startModule() {
  // sets your node in running mode. Activate publishers, subscribers, service
  // servers, etc here.

  const uint32_t input_queue_size =
      static_cast<uint32_t>(parameter_handler_.getParam(INPUT_QUEUE_SIZE));
  const uint32_t output_queue_size =
      static_cast<uint32_t>(parameter_handler_.getParam(OUTPUT_QUEUE_SIZE));

  // Publishers
  rospub_lane_left =
      node_handle_.advertise<nav_msgs::Path>("road_lane_left", output_queue_size);
  rospub_lane_middle =
      node_handle_.advertise<nav_msgs::Path>("road_lane_middle", output_queue_size);
  rospub_lane_middle_no_passing = node_handle_.advertise<nav_msgs::Path>(
      "road_lane_middle_no_passing", output_queue_size);
  rospub_lane_right =
      node_handle_.advertise<nav_msgs::Path>("road_lane_right", output_queue_size);

  // Subscribers
  rossub_reset =
      node_handle_.subscribe("auto_reset", 1, &LaneDetectionNode::handleReset, this);
  rossub_set_start_box = node_handle_.subscribe(
      "set_start_box", 1, &LaneDetectionNode::handleSetStartBucht, this);
  image_sub.subscribe(node_handle_, "preprocessed_image", input_queue_size);
  roi_sub.subscribe(node_handle_, "region_of_interest", input_queue_size);

  lane_detection_->startModule();

  rosserv_set_middle_line = node_handle_.advertiseService(
      "set_middle_line", &LaneDetectionNode::setMiddleLine, this);
  reset_server_ = node_handle_.advertiseService("reset", &LaneDetectionNode::reset, this);
}

void LaneDetectionNode::stopModule() {

  // Publishers
  rospub_lane_left.shutdown();
  rospub_lane_middle.shutdown();
  rospub_lane_middle_no_passing.shutdown();
  rospub_lane_right.shutdown();

  // Subscribers
  rossub_reset.shutdown();
  rossub_set_start_box.shutdown();
  image_sub.unsubscribe();
  roi_sub.unsubscribe();

  // Services
  rosserv_set_middle_line.shutdown();
}

void LaneDetectionNode::handleImage(const sensor_msgs::ImageConstPtr& image_msg,
                                    const perception_msgs::RegionOfInterestStampedConstPtr& roi_msg) {

  cv_bridge::CvImageConstPtr cv_ptr;
  try {
    cv_ptr = cv_bridge::toCvShare(image_msg, sensor_msgs::image_encodings::MONO8);
  } catch (cv_bridge::Exception& e) {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }
  const cv::Rect image_limits = fromMsg(roi_msg->roi);

  roi_view_transformation->update();
  roi_view_transformation->reconfigure(toEigen(image_limits.tl()));
  ego_vehicle_->update(image_limits);
  feature_extraction_->updateROI(image_limits);

  const LineVehiclePoints line_data = lane_detection_->processImage(cv_ptr->image);

  const std::string frame_id = "vehicle";
  const ros::Time time_stamp = image_msg->header.stamp;
  using namespace common;
  rospub_lane_left.publish(
      toConstPtr(createPathMsg(line_data[LINESPEC_LEFT], time_stamp, frame_id)));
  rospub_lane_middle.publish(
      toConstPtr(createPathMsg(line_data[LINESPEC_MIDDLE], time_stamp, frame_id)));
  rospub_lane_right.publish(
      toConstPtr(createPathMsg(line_data[LINESPEC_RIGHT], time_stamp, frame_id)));
  rospub_lane_middle_no_passing.publish(toConstPtr(
      createPathMsg(line_data[LINESPEC_NO_PASSING], time_stamp, frame_id)));
}


void LaneDetectionNode::handleReset(const std_msgs::Empty) {
  ROS_INFO("Lane_Detection has been resetted.");
  lane_detection_->setInitFlag();
  lane_detection_->setStartBox(false);
}

void LaneDetectionNode::handleSetStartBucht(const std_msgs::BoolConstPtr& msg_start_bucht) {
  lane_detection_->setStartBox(msg_start_bucht->data);
}

bool LaneDetectionNode::setMiddleLine(perception_msgs::SetMiddleLine::Request& req,
                                      perception_msgs::SetMiddleLine::Response&) {
  VehiclePoints points;
  tf2::fromMsg(req.middle_line, points);
  lane_detection_->setMiddleLine(points);
  return true;
}

const std::string LaneDetectionNode::getName() {
  return std::string("lane_detection");
}

bool LaneDetectionNode::reset(std_srvs::Empty::Request&, std_srvs::Empty::Response&) {
  ROS_INFO("Lane_Detection has been resetted.");
  lane_detection_->setInitFlag();
  lane_detection_->setStartBox(false);
  return true;
}

CREATE_NODE_WITH_FANCY_DEBUG(LaneDetectionNode, LaneDetectionNodeDebug)
