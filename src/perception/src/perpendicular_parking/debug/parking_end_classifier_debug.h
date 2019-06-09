#ifndef PARKING_END_CLASSIFIER_DEBUG_H
#define PARKING_END_CLASSIFIER_DEBUG_H

#include "../parking_end_classifier.h"

namespace perpendicular_parking {
class ParkingEndClassifierDebug : public ParkingEndClassifier {
 public:
  ParkingEndClassifierDebug(ParkingEndClassifier &&parking_end_classifier);

  boost::optional<WorldPose> detectEnd(const ParkingSpotsConstRef &all_parking_spots,
                                       const cv::Mat &img,
                                       const ros::Time &timestamp,
                                       const common::DynamicPolynomial &left_lane_polynom,
                                       const LineVehiclePoints &lanes,
                                       const Eigen::Affine3d &map_T_world) override;

  const ImagePoints debugEndLinePoints() const {
    return this->debug_end_line_points;
  }
  const ImagePoints debugFeaturePoints() const {
    return this->debug_feature_points;
  }
  const ScanLines debugScanLines() const { return this->debug_scan_lines; }

  const ImagePoints debugCrucialPoints() const {
    return this->debug_crucial_points;
  }

  const FeaturePointClusters debugEndlineClusters() const {
    return this->debug_endline_clusters;
  }

  void reset() override;

 private:
  ImagePoints apply1DGradientDetector(const cv::Mat &img,
                                      const ScanLines &scan_lines,
                                      const bool all) const override;

  ScanLines createScanLines(const ImagePoints &start_points,
                            const ImagePoints &end_points) const override;

  ScanLines getParallelScanlines(const common::DynamicPolynomial &polynom,
                                 const VehiclePoint &start_point,
                                 const VehiclePoint &end_point) const override;

  ImagePoints toLinePoints(const WorldPose &pose);

  virtual std::vector<common::DynamicPolynomial> fitLinesR(const std::size_t nr_lines,
                                                           const VehiclePoints &points) const override;


  mutable ImagePoints debug_end_line_points;
  mutable ImagePoints debug_feature_points;
  mutable ImagePoints debug_crucial_points;
  mutable ScanLines debug_scan_lines;
  mutable FeaturePointClusters debug_endline_clusters;
  cv::Rect img_size_;
};
}  // namespace perpendicular_parking

#endif  // PARKING_END_CLASSIFIER_DEBUG_H
