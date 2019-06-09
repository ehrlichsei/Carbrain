#ifndef OTSU_CLASSIFIER_DEBUG_H
#define OTSU_CLASSIFIER_DEBUG_H

#include "../../classifiers/otsu_classifier.h"
#include "../classifier_debug.h"

namespace road_object_detection {

class OtsuClassifierDebug : public virtual OtsuClassifier, public virtual ClassifierDebug {
 public:
  OtsuClassifierDebug(DebugImages* debug_images);
  virtual ~OtsuClassifierDebug() = default;

  virtual RoadObjects classify(const Features& features);
};


} //namespace road_object_detection

#endif  // OTSU_CLASSIFIER_DEBUG_H
