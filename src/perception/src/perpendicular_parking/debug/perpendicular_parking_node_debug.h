#ifndef PERPENDICULAR_PARKING_NODE_DEBUG_H
#define PERPENDICULAR_PARKING_NODE_DEBUG_H

#include "../perpendicular_parking_node.h"
#include "perpendicular_parking_debug.h"

THIRD_PARTY_HEADERS_BEGIN
#include <perception_msgs/Pixels.h>
#include <perception_msgs/PointsClusters.h>
#include <perception_msgs/ScanLines.h>
THIRD_PARTY_HEADERS_END

namespace perpendicular_parking {

class PerpendicularParkingNodeDebug : public PerpendicularParkingNode {

  using FeaturePointClusters = std::vector<FeaturePointCluster>;

 public:
  PerpendicularParkingNodeDebug(ros::NodeHandle& node_handle);

 protected:
  void startModule() override;

  void stopModule() override;

  void handleLeftLaneAndImage(const sensor_msgs::ImageConstPtr& image_raw_msg,
                              const nav_msgs::PathConstPtr& left_lane_msg,
                              const nav_msgs::PathConstPtr& middle_lane_msg,
                              const nav_msgs::PathConstPtr& right_lane_msg,
                              const nav_msgs::PathConstPtr& no_passing_lane_msg) override;

 private:
  void createParkingSpotsMsg();
  perception_msgs::ScanLines createScanLinesMsgs(const ScanLines& scan_lines,
                                                 const ros::Time& stamp);
  perception_msgs::Pixels createPixelsMsgs(const ImagePoints& points, const ros::Time& stamp);
  perception_msgs::PointsClusters createPointsClustersMsg(const FeaturePointClusters& clusters,
                                                          const ros::Time& stamp);

  perception_msgs::Pixels toPixelsMsg(const ImagePoints& points) const;
  perception_msgs::Pixel toPixelMsg(const ImagePoint& point) const;
  perception_msgs::ScanLines toScanLinesMsg(const ScanLines& scan_lines);

  road_object_detection::MessageHelper<perception_msgs::PerpendicularParkingSpots, perception_msgs::PerpendicularParkingSpot> all_parking_spots_helper;

  ros::Publisher parking_start_line_points_pub;
  ros::Publisher parking_start_feature_points_pub;
  ros::Publisher parking_lot_feature_points_pub;
  ros::Publisher parking_start_scan_lines_pub;
  ros::Publisher parking_lot_marking_scan_lines_pub;
  ros::Publisher parking_slot_scan_lines_pub;
  ros::Publisher parking_start_clusters_pub;
  ros::Publisher parking_start_crucial_points_pub;
  ros::Publisher parking_lot_marking_detections_pub;
  ros::Publisher parking_end_line_points_pub;
  ros::Publisher parking_end_feature_points_pub;
  ros::Publisher parking_end_scan_lines_pub;
  ros::Publisher parking_end_crucial_points_pub;
  ros::Publisher parking_end_clusters_pub;
  ros::Publisher left_lane_polynom_pub;
  ros::Publisher parking_spots_lines_pub;
  // free spots are published by super class and can be subscribed by
  // debugger/perpendicular_parking_debug anway
};
}  // namespace perpendicular_parking

#endif  // PERPENDICULAR_PARKING_NODE_DEBUG_H
