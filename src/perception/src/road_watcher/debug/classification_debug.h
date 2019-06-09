#ifndef CLASSIFICATION_DEBUG_H
#define CLASSIFICATION_DEBUG_H

#include "../../road_object_detection/debug/debug_images.h"
#include "../classification.h"

class ClassificationDebug : public Classification {
 public:
  ClassificationDebug(Classification&& classification,
                      ParameterInterface* parameters_ptr,
                      const common::CameraTransformation* camera_transform,
                      //const road_object_detection::WorldCoordinatesHelper* const world_coordinates_helper,
                      road_object_detection::DebugImages* debug_images);

  virtual road_object_detection::RoadObjects classify(
      const std::vector<FeaturePointCluster>& clusters,
      const cv::Mat& camera_image,
      const ros::Time& timestamp,
      common::DynamicPolynomial& middle_lane_polynomial,
      const LineVehiclePoints& points) override;

  void addClassifier(std::unique_ptr<road_object_detection::Classifier> classifier);

 protected:
  virtual road_object_detection::RoadObjects runClassifiersOnCluster(
      const road_object_detection::Features& features) override;

 private:
  road_object_detection::DebugImages* debug_images;
};

#endif  // CLASSIFICATION_DEBUG_H
