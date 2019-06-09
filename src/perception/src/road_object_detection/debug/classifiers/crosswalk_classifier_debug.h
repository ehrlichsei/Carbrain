#ifndef CROSSWALKCLASSIFIERDEBUG_H
#define CROSSWALKCLASSIFIERDEBUG_H

#include "../../classifiers/crosswalk_classifier.h"
#include "../classifier_debug.h"

namespace road_object_detection {


class CrosswalkClassifierDebug : public virtual ClassifierDebug,
                                 public virtual CrosswalkClassifier {
 public:
  CrosswalkClassifierDebug(DebugImages* debug_images,
                           const common::CameraTransformation* cam_transform,
                           ParameterInterface* parameter_interface);

  virtual ~CrosswalkClassifierDebug() override = default;

  virtual RoadObjects classify(const Features& features) override;

 private:
  virtual std::vector<CVPoints> onlyPointsOnLane(const std::vector<CVPoints>& crosswalk_lines,
                                                 const Features& features) const override;

  virtual CVPoints linePoints(const Features& features, const int row_idx) const override;

  virtual std::pair<common::DynamicPolynomial, common::DynamicPolynomial> extractBoundingPolynomials(
      const Features& features) const override;
};

}  // namespace road_object_detection

#endif  // CROSSWALKCLASSIFIERDEBUG_H
