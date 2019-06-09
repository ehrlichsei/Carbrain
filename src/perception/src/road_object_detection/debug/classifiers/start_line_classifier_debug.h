#ifndef START_LINE_CLASSIFIER_DEBUG_H
#define START_LINE_CLASSIFIER_DEBUG_H

#include "../../classifiers/start_line_classifier.h"
#include "../classifier_debug.h"

namespace road_object_detection {

class StartLineClassifierDebug : public virtual ClassifierDebug,
                                 public virtual StartLineClassifier {
 public:
  StartLineClassifierDebug(DebugImages* debug_images,
                           const common::CameraTransformation* cam_transform,
                           ParameterInterface* parameter_interface);

  virtual RoadObjects classify(const Features& features) override;

 private:
  CVPoints steps_debug;
  mutable Eigen::Vector3d pc;

  bool isLine(const Features& features, Eigen::Vector3d& direction) const override;
  //  double meanStepSize(const Eigen::Vector2d& pc, const Features& features)
  //  const override;


//  CVPoints stepPoints(const Features& features,
//                      const VehiclePoint& start_dir,
//                      const VehiclePoint& end_dir) const override;

//  cv::Rect bvROI(const Features& features) const override;

  void draw(cv::Mat* debug_image, const Features& features) const;
};


} //namespace road_object_detection

#endif  // START_LINE_CLASSIFIER_DEBUG_H
