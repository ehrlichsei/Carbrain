#ifndef LOOK_AT_DEBUG_H
#define LOOK_AT_DEBUG_H

#include "../../road_object_detection/debug/debug_images.h"
#include "../look_at.h"

namespace look_at {


class LookAtDebug : public virtual LookAt {
 public:
  LookAtDebug(LookAt &&look_at, road_object_detection::DebugImages *debug_images);

  virtual ~LookAtDebug() override = default;

  virtual ClassificationResults classifyROI(
      const cv::Mat &img,
      const ros::Time &timestamp,
      const RegionsToClassify &regions,
      const LineVehiclePoints &lane_points,
      const std::unique_ptr<road_object_detection::Classifier> &classifier) override;


 private:
  virtual VehiclePoints allPointsToMiddle(LineVehiclePoints points) const override;

  virtual ImagePoints apply1dGradientDetector(const cv::Mat &img,
                                              const ScanLines &scan_lines) const override;

  virtual std::vector<FeaturePointCluster> clusterFeaturePoints(const FeaturePointCluster &raw) const override;

  virtual ScanLines createScanLineGrid(const RegionToClassify &roi,
                                       const double step_size) const override;


  mutable cv::Mat *debug_img_;
  mutable std::vector<FeaturePointCluster> clusters_;

  void visualize(const ScanLines &scan_lines) const;

  void visualize(const ImagePoints &feature_points,
                 const cv::Scalar &color = CV_RGB(0, 255, 255)) const;

  void visualize(const std::vector<FeaturePointCluster> &clusters) const;
};
}  // namespace look_at

#endif  // LOOK_AT_DEBUG_H
