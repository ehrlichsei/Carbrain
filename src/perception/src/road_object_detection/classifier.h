#ifndef CLASSIFIER_H
#define CLASSIFIER_H
#include "features/features.h"
#include "road_objects/road_object.h"
#include "road_object_visitors/world_coordinates_helper.h"

namespace road_object_detection {

//! \brief Absract (see below) base class to derive classifiers from.
class Classifier {
 public:
  Classifier() = default;
  virtual ~Classifier() = default;

  virtual RoadObjects classify(const Features& features) = 0;
  // =0 makes the class Classifier abstract
  // (you can't instantiate a general Classifier object, only subclasses)

  /* you sould use the following implementation
   * size_t YOUR_CLASSIFIER::getClassifierId() const final override {
   *   return typeid(YOUR_CLASSIFIER).hash_code();
   * }
   */
  virtual size_t getClassifierId() const = 0;
};


} //namespace road_object_detection

#endif  // CLASSIFIER_H
