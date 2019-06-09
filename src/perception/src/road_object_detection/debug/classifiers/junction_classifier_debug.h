#ifndef JUNCTION_CLASSIFIER_DEBUG_H
#define JUNCTION_CLASSIFIER_DEBUG_H

#include "../../classifiers/junction_classifier.h"
#include "../classifier_debug.h"
#include "../../../utils/step_detection.h"

#include <deque>

namespace road_object_detection {

class JunctionClassifierDebug : public virtual JunctionClassifier, ClassifierDebug {
 public:
  JunctionClassifierDebug(ParameterInterface* parameter_interface,
                          const common::CameraTransformation* const cam_transform,
                          DebugImages* debug_images);

  virtual RoadObjects classify(const Features& features) override;


 protected:
  virtual FourCandidates findFourCandidates(const Features& features,
                                            const ImagePoints& pos_steps_box,
                                            const ImagePoints& neg_steps_box) const override;

 private:
  struct Boarders {
    int max_x;
    int min_x;
    int max_y;
    int min_y;
    int min_neg;
    int min_pos;
  };

  void drawBox(cv::Mat* debug_birdsview, const Boarders& boarders) const;
  void drawPoints(cv::Mat* debug_birdsview, const ImagePoints& Points, const cv::Scalar& color) const;
};


}  // namespace road_object_detection

#endif  // JUNCTION_CLASSIFIER_DEBUG_H
