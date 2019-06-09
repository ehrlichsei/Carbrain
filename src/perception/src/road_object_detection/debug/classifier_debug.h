#ifndef CLASSIFIER_DEBUG_H
#define CLASSIFIER_DEBUG_H

#include "../classifier.h"
#include "debug_images.h"

namespace road_object_detection {

class ClassifierDebug : public virtual Classifier {
 public:
  ClassifierDebug(DebugImages* debug_images);
  virtual ~ClassifierDebug() override = default;

  virtual RoadObjects classify(const Features& features) override = 0;
  // =0 makes the class Classifier abstract
  // (you can't instantiate a general Classifier object, only subclasses)

 protected:
  mutable DebugImages* debug_images;
};

}  // namespace road_object_detection

#endif  // CLASSIFIER_DEBUG_H
