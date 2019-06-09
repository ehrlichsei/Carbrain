#ifndef CLASSIFICATION_H
#define CLASSIFICATION_H
#include <common/macros.h>

THIRD_PARTY_HEADERS_BEGIN
#include <ros/time.h>
#include <memory>
#include <vector>
THIRD_PARTY_HEADERS_END

//#include "../road_object_detection/classifier.h"
#include "../road_object_detection/classifiers/no_passing_zone_classifier.h"
#include "../road_object_detection/features/feature_extractor.h"
#include "../road_object_detection/features/features.h"
#include "../utils/dbscan_clusterer.h"
#include "common/parameter_interface.h"
#include "common/polynomial.h"


//! \brief Handles the classification of feature point clusters by classifiers
namespace voting {
/*!
 * \brief voteOnIntersectingRoadObjects if the base area of two RoadObjects
 * intersects, delete the RoadObject with the lower score
 * \param road_objects the road objects to look at.
 */
void voteOnIntersectingRoadObjects(road_object_detection::RoadObjects &road_objects);

bool checkConvexPolygonForIntersection(const VehiclePoints &first_polygon,
                                       const VehiclePoints &second_polygon);

bool testPolygonSidesForSeperatingAxis(const VehiclePoints &axis_polygon,
                                       const VehiclePoints &point_polygon);

}  // namespace voting

class Classification {
 public:
  Classification(ParameterInterface *parameters_ptr,
                 const common::CameraTransformation *camera_transform,
                 const EgoVehicle *const ego_vehicle,
                 const road_object_detection::WorldCoordinatesHelper *const world_coordinates_helper);

  Classification(Classification &&) = default;

  virtual ~Classification() = default;

  /*!
   * \brief classify
   * These results are expressed in pass by ref 'classifications'.
   * \param clusters input vector of feature point clusters
   * \param camera_image original gray level camera image
   * \param timestamp timestamp of camera_image
   * \param middle_lane_polynomial polyomial representing middle lane
   * \return classifications
   */
  virtual road_object_detection::RoadObjects classify(const std::vector<FeaturePointCluster> &clusters,
                                                      const cv::Mat &camera_image,
                                                      const ros::Time &timestamp,
                                                      common::DynamicPolynomial &middle_lane_polynomial,
                                                      const LineVehiclePoints &points);

  // RoadObjects applyClassifiersToCluster(const FeaturePointCluster& cluster);
  // setterMethod for crosswalkClassifier

 protected:
  ParameterInterface *parameters_ptr_;
  const common::CameraTransformation *camera_transform;
  std::unique_ptr<road_object_detection::FeatureExtractor> feature_extractor;
  const road_object_detection::WorldCoordinatesHelper *world_coordinates_helper_;
  std::vector<std::unique_ptr<road_object_detection::Classifier>> classifiers;

  //  std::unique_ptr<road_object_detection::NoPassingZoneClassifier> no_passing_zone_classifier;

  //! \brief Classifies give set of features and returns RoadObject of the
  //! highes scoring class.
  virtual road_object_detection::RoadObjects runClassifiersOnCluster(
      const road_object_detection::Features &features);

 private:
  //! \brief Returns a full-fledged RoadObject of type Unidentified
  std::unique_ptr<road_object_detection::RoadObject> makeUnidentified(
      const road_object_detection::Features &features) const;

  void addClassifier(std::unique_ptr<road_object_detection::Classifier> classifier);

  static const ParameterString<double> UNIDENTIFIED_SCORE_THRESHOLD;
  static const ParameterString<bool> ALL_CLASSIFIERS;
};

#endif  // CLASSIFICATION_H
