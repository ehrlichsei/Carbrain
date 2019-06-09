#ifndef LANE_TRACKING_DEBUG_H
#define LANE_TRACKING_DEBUG_H
#include <common/macros.h>

THIRD_PARTY_HEADERS_BEGIN
#include "perception_msgs/Lane.h"
#include <ros/ros.h>
THIRD_PARTY_HEADERS_END

#include "../lane_detection.h"

class LaneDetectionNodeDebug;

class LaneDetectionDebug : public LaneDetection {
 public:
  struct Publishers {
    ros::Publisher rospub_left_proj_points;
    ros::Publisher rospub_middle_proj_points;
    ros::Publisher rospub_right_proj_points;

    ros::Publisher rospub_polynomial_lane_left;
    ros::Publisher rospub_polynomial_lane_middle;
    ros::Publisher rospub_polynomial_lane_right;

    void startPublishers(ros::NodeHandle& node_handle);
    void stopPublishers();
  };

  LaneDetectionDebug(LaneDetection&& lane_detection,
                     LaneDetectionNodeDebug* lane_detection_node_debug);

  LineVehiclePoints processImage(const cv::Mat& image) override;

 private:
  virtual void showProjectionPoints(const LineSpec& target, const VehiclePoints& points) override;

  perception_msgs::Lane createLaneMsg(const LineSpec& line_type) const;

  LaneDetectionNodeDebug* lane_detection_node_debug;

  const Publishers& publishers;
};

#endif  // LANE_TRACKING_DEBUG_H
