#ifndef PARKING_START_CLASSIFIER_DEBUG_H
#define PARKING_START_CLASSIFIER_DEBUG_H

#include "../../../include/scan_line.h"
#include "../parking_start_classifier.h"

namespace perpendicular_parking {

using FeaturePointsClusters = std::vector<FeaturePointCluster>;

class ParkingStartClassifierDebug : public ParkingStartClassifier {
 public:
  ParkingStartClassifierDebug(ParkingStartClassifier &&parking_start_classifier);

  Type classify(FeaturePointCluster &cluster,
                const common::DynamicPolynomial &left_lane_polynom,
                const ros::Time &timestamp) override;
  WorldPose startLinePose() const override;
  ImagePoints startLine() const;
  FeaturePointClusters clusters() const;
  ImagePoints crucialPoints() const;

  ScanLines debugScanLines() const { return this->debug_scanlines_; }

 private:
  virtual std::vector<common::DynamicPolynomial> fitLinesR(const std::size_t nr_lines,
                                                           const VehiclePoints &points) const override;

  mutable WorldPose start_pose_;
  mutable ScanLines debug_scanlines_;
  mutable FeaturePointClusters points_clusters;
  cv::Rect img_size_;
  ImagePoints toLinePoints(const WorldPose &pose) const;
};
}  // namespace perpendicular_parking

#endif  // PARKING_START_CLASSIFIER_DEBUG_H
