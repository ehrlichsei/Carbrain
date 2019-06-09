#include "lane_detection_node_debug.h"

#include "lane_detection_debug.h"
#include "feature_extraction_debug.h"

LaneDetectionNodeDebug::LaneDetectionNodeDebug(ros::NodeHandle &node_handle)
    : LaneDetectionNode(node_handle) {
  feature_extraction_ =
      std::make_unique<FeatureExtractionDebug>(std::move(*feature_extraction_), this);
  lane_detection_->setFeatureExtraction(feature_extraction_.get());
  lane_detection_ =
      std::make_unique<LaneDetectionDebug>(std::move(*lane_detection_), this);
}

void LaneDetectionNodeDebug::handleImage(const sensor_msgs::ImageConstPtr &image_msg,
                                         const perception_msgs::RegionOfInterestStampedConstPtr &roi_msg) {
  this->timestamp = image_msg->header.stamp;
  LaneDetectionNode::handleImage(image_msg, roi_msg);
}

const ros::Time &LaneDetectionNodeDebug::getTimeStamp() { return timestamp; }

const FeatureExtractionDebug::Publishers &LaneDetectionNodeDebug::getFeatureExtractionDebugPublishers() const {
  return feature_extraction_debug_publishers;
}

const LaneDetectionDebug::Publishers &LaneDetectionNodeDebug::getLaneDetectionDebugPublishers() const {
  return lane_detection_debug_publishers;
}

void LaneDetectionNodeDebug::startModule() {
  LaneDetectionNode::startModule();
  feature_extraction_debug_publishers.startPublishers(node_handle_);
  lane_detection_debug_publishers.startPublishers(node_handle_);
}

void LaneDetectionNodeDebug::stopModule() {
  LaneDetectionNode::stopModule();
  feature_extraction_debug_publishers.stopPublishers();
  lane_detection_debug_publishers.stopPublishers();
}
