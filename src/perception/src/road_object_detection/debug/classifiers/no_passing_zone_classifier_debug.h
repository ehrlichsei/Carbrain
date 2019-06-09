#ifndef NO_PASSING_ZONE_CLASSIFIER_DEBUG_H
#define NO_PASSING_ZONE_CLASSIFIER_DEBUG_H

#include "../../classifiers/no_passing_zone_classifier.h"
#include "../classifier_debug.h"
#include "common/types.h"
#include "vehicle_scan_line.h"


namespace road_object_detection {

class NoPassingZoneClassifierDebug : public virtual NoPassingZoneClassifier,
                                     public virtual ClassifierDebug {
 public:
  NoPassingZoneClassifierDebug(const common::CameraTransformation* const camera_transformation,
                               ParameterInterface* const parameters_ptr,
                               DebugImages* debug_images);

  NoPassingZoneClassifierDebug() = delete;

  virtual ~NoPassingZoneClassifierDebug() override = default;

  virtual RoadObjects classify(const Features& features) override;

 private:
  // void np_debug(std::string str);
  void drawSteps(const std::vector<VehiclePoints>& step_clstrs_gnd, const cv::Vec3b& color);

  void np_debug(const std::string& str) const;

  // virtual std::vector<VehiclePoints> getSteps(const Features& features) const override;
};
}  // namespace road_object_detection

#endif  // NO_PASSING_ZONE_CLASSIFIER_DEBUG_H
