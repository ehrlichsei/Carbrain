#include "preprocessing_node.h"
#include <common/macros.h>

THIRD_PARTY_HEADERS_BEGIN
#include <thread>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include "perception_msgs/RegionOfInterestStamped.h"

// dynamic_reconfigure config headers
#include "perception/PreprocessingBinarisationConfig.h"
#include "perception/PreprocessingRoiConfig.h"
#include "perception/PreprocessingMedianBlurConfig.h"
#include "perception/PreprocessingMaskModeConfig.h"
#include "perception/PreprocessingTrapezoidMaskConfig.h"
#include "perception/PreprocessingPixelwiseMaskConfig.h"
THIRD_PARTY_HEADERS_END

#include "common/node_creation_makros.h"
#include "perception_message_conversions.h"

using namespace perception::message_conversion;

const ParameterString<int> PreprocessingNode::INPUT_QUEUE_SIZE(
    "input_queue_size");
const ParameterString<int> PreprocessingNode::OUTPUT_QUEUE_SIZE(
    "output_queue_size");

PreprocessingNode::PreprocessingNode(ros::NodeHandle &node_handle)
    : NodeBase(node_handle), preprocessing_(&parameter_handler_) {

  ros::NodeHandle node_handle_package(common::getParent(node_handle_));
  ros::NodeHandle node_handle_trapezoid_mask(node_handle_package,
                                             "ego_vehicle/trapezoid_mask");
  ros::NodeHandle node_handle_roi(node_handle_, "roi");
  ros::NodeHandle node_handle_binarisation(node_handle_, "binarisation");
  ros::NodeHandle node_handle_median_blur(node_handle_, "median_blur");
  ros::NodeHandle node_handle_pixelwise_mask(node_handle_package,
                                             "ego_vehicle/pixelwise_mask");
  ros::NodeHandle node_handle_mask_mode(node_handle_package, "ego_vehicle");

  // init dynamic reconfigure
  parameter_handler_.addDynamicReconfigureServer<perception::PreprocessingRoiConfig>(node_handle_roi);
  parameter_handler_.addDynamicReconfigureServer<perception::PreprocessingTrapezoidMaskConfig>(
      node_handle_trapezoid_mask);
  parameter_handler_.addDynamicReconfigureServer<perception::PreprocessingBinarisationConfig>(
      node_handle_binarisation);
  parameter_handler_.addDynamicReconfigureServer<perception::PreprocessingMedianBlurConfig>(
      node_handle_median_blur);
  parameter_handler_.addDynamicReconfigureServer<perception::PreprocessingPixelwiseMaskConfig>(
      node_handle_pixelwise_mask);
  parameter_handler_.addDynamicReconfigureServer<perception::PreprocessingMaskModeConfig>(
      node_handle_mask_mode);

  parameter_handler_.registerParam(INPUT_QUEUE_SIZE);
  parameter_handler_.registerParam(OUTPUT_QUEUE_SIZE);
}

void PreprocessingNode::startModule() {

  const int output_queue_size = parameter_handler_.getParam(OUTPUT_QUEUE_SIZE);
  rospub_preprocessed_image = node_handle_.advertise<sensor_msgs::Image>(
      "preprocessed_image", output_queue_size);
  rospub_region_of_interest =
      node_handle_.advertise<perception_msgs::RegionOfInterestStamped>(
          "region_of_interest", output_queue_size);

  const int input_queue_size = parameter_handler_.getParam(INPUT_QUEUE_SIZE);

  image_transport::ImageTransport img_trans(node_handle_);
  rossub_image = img_trans.subscribe(
      "image_raw", input_queue_size, &PreprocessingNode::handleImage, this);
}

void PreprocessingNode::stopModule() {
  rossub_image.shutdown();
  rospub_preprocessed_image.shutdown();
}

void PreprocessingNode::handleImage(const sensor_msgs::ImageConstPtr &msg) {
  cv_bridge::CvImageConstPtr cv_ptr;
  try {
    cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::MONO8);
  } catch (const cv_bridge::Exception &e) {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }
  cv_bridge::CvImage out_msg;
  out_msg.header = msg->header;
  out_msg.encoding = sensor_msgs::image_encodings::MONO8;

  preprocessing_.preprocessImage(cv_ptr->image, out_msg.image);

  sensor_msgs::ImagePtr out_ptr(out_msg.toImageMsg());
  rospub_preprocessed_image.publish(out_ptr);

  const cv::Rect &roi = preprocessing_.getRegionOfInterest();
  rospub_region_of_interest.publish(toMsg(roi, msg->header.frame_id, msg->header.stamp));
}

const std::string PreprocessingNode::getName() {
  return std::string("preprocessing");
}

CREATE_NODE(PreprocessingNode)
