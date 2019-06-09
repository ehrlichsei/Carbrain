#ifndef LANE_DETECTION_DEBUG_H
#define LANE_DETECTION_DEBUG_H
#include <common/macros.h>

THIRD_PARTY_HEADERS_BEGIN
#include <ros/ros.h>
THIRD_PARTY_HEADERS_END

#include "../lane_detection_node.h"
#include "feature_extraction_debug.h"
#include "lane_detection_debug.h"

class LaneDetectionNodeDebug : public LaneDetectionNode {
 public:
  LaneDetectionNodeDebug(ros::NodeHandle& node_handle);

  virtual void handleImage(const sensor_msgs::ImageConstPtr& image_msg,
                           const perception_msgs::RegionOfInterestStampedConstPtr& roi_msg) override;

  const ros::Time& getTimeStamp();

  const FeatureExtractionDebug::Publishers& getFeatureExtractionDebugPublishers() const;
  const LaneDetectionDebug::Publishers& getLaneDetectionDebugPublishers() const;

 protected:
  /*!
   * \brief startModule is called, if the node shall be turned active. In here
   * the subrscribers an publishers are started.
   */
  virtual void startModule() override;
  /*!
   * \brief stopModule is called, if the node shall be turned inactive. In this
   * function subscribers and publishers are shutdown.
   */
  virtual void stopModule() override;

 private:
  ros::Time timestamp;

  FeatureExtractionDebug::Publishers feature_extraction_debug_publishers;
  LaneDetectionDebug::Publishers lane_detection_debug_publishers;
};
#endif  // LANE_DETECTION_DEBUG_H
