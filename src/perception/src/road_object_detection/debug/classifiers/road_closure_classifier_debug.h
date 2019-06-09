#ifndef ROADCLOSURECLASSIFIERDEBUG_H
#define ROADCLOSURECLASSIFIERDEBUG_H

#include "../../classifiers/road_closure_classifier.h"
#include "../classifier_debug.h"

namespace road_object_detection {

class RoadClosureClassifierDebug : public virtual ClassifierDebug,
                                   public virtual RoadClosureClassifier {
 public:
  RoadClosureClassifierDebug(DebugImages *debug_images,
                             const common::CameraTransformation *cam_transform,
                             ParameterInterface *parameter_interface);
  virtual RoadObjects classify(const Features &features) override;

 private:
  virtual std::pair<VehiclePoints, VehiclePoints> splitCluster(
      const FeaturePointCluster &refined_cluster) const override;

  virtual boost::optional<RCLine> searchBoundary(const VehiclePoints &road_closure_boundary,
                                                 const Features &features) const override;

  virtual RCLines extractInnerLines(const std::vector<FeaturePointCluster> &inner_clusters,
                                    const RCLine &bounding_line) const override;

  virtual VehiclePoints removeClosePointsPolynomial(const VehiclePoints &ref_points,
                                                    const VehiclePoints &cluster_points,
                                                    const double thresh,
                                                    const common::PolynomialDegree poly_deg) const override;

  virtual VehiclePoints searchAdditionalFeaturePoints(const FeaturePointCluster &dominant_cluster,
                                                      const Features &features) const override;

  virtual ScanLines createAdditionalScanlines(const FeaturePointCluster &dominant_cluster,
                                              const Features &features) const override;

  void visualize(const ImagePoints &feature_points, const cv::Scalar &color) const;
};

}  // namespace road_object_detection

#endif  // ROADCLOSURECLASSIFIERDEBUG_H
