#ifndef FEATUREEXTRACTIONDEBUG_H
#define FEATUREEXTRACTIONDEBUG_H
#include <common/macros.h>

THIRD_PARTY_HEADERS_BEGIN
#include <memory>
#include <ros/ros.h>
#include "perception_msgs/ScanLines.h"
#include "perception_msgs/Pixels.h"
THIRD_PARTY_HEADERS_END

#include "../feature_extraction.h"


class LaneDetectionNodeDebug;

class FeatureExtractionDebug : public FeatureExtraction {
 public:
  struct Publishers {
    ros::Publisher rospub_scan_lines_left;
    ros::Publisher rospub_scan_lines_middle;
    ros::Publisher rospub_scan_lines_right;
    ros::Publisher rospub_feature_points_left;
    ros::Publisher rospub_feature_points_middle;
    ros::Publisher rospub_feature_points_right;

    ros::Publisher rospub_middle_points_left;
    ros::Publisher rospub_middle_points_right;

    void startPublishers(ros::NodeHandle& node_handle);
    void stopPublishers();
  };

  FeatureExtractionDebug(FeatureExtraction&& feature_extraction,
                         LaneDetectionNodeDebug* lane_detection_node_debug);


  virtual LineVehiclePoints extractLineMarkings(const cv::Mat& image) const override;
  virtual LineVehiclePoints extractLineMarkings(const cv::Mat& image,
                                                const LaneModel& line_data) const override;

  virtual ScanLines createScanLines(const common::DynamicPolynomial& line) const override;
  virtual ScanLines createScanLines(const VehiclePoints& ground_points) const override;
  virtual ImagePoints extractImagePointsByScanLines(
      const cv::Mat& image,
      const ScanLines& scan_lines,
      const boost::optional<common::DynamicPolynomial>& ref_line = boost::none,
      const boost::optional<LineSpec> l_spec = boost::none) const override;
  virtual bool isDoubleLine(const cv::Mat& image,
                            const ScanLines& lines,
                            VehiclePoints* left_line,
                            VehiclePoints* right_line) override;


 private:
  void publishScanLines(const ScanLines& scan_lines) const;


  LaneDetectionNodeDebug* lane_detection_node_debug;
  const FeatureExtractionDebug::Publishers& publishers;
  mutable LineSpec line_type;
  mutable bool publish_scanlines;
};

#endif  // FEATUREEXTRACTIONDEBUG_H
