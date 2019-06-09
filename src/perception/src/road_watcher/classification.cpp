#include "classification.h"
#include <memory>
#include "common/basic_statistics_eigen.h"
#include "common/console_colors.h"
#include "common/container.h"
#include "common/math.h"
#include "common/move_range.h"
#include "common/unique_erase.h"

THIRD_PARTY_HEADERS_BEGIN
#include <boost/algorithm/cxx11/all_of.hpp>
#include <boost/algorithm/cxx11/none_of.hpp>
#include <boost/range/algorithm/max_element.hpp>
#include <boost/range/algorithm_ext/erase.hpp>
#include <boost/range/algorithm_ext/push_back.hpp>
#include <boost/range/iterator_range.hpp>
#include "boost/range/algorithm.hpp"
THIRD_PARTY_HEADERS_END

#include "opencv_eigen_conversions.h"

#include "../road_object_detection/classifiers/contour_classifier.h"
#include "../road_object_detection/classifiers/crosswalk_classifier.h"
#include "../road_object_detection/classifiers/junction_classifier.h"
#include "../road_object_detection/classifiers/no_passing_zone_classifier.h"
#include "../road_object_detection/classifiers/obstacle_classifier_new.h"
#include "../road_object_detection/classifiers/road_closure_classifier.h"
#include "../road_object_detection/classifiers/start_line_classifier.h"

using namespace road_object_detection;
using namespace contour_classifier;

const ParameterString<double> Classification::UNIDENTIFIED_SCORE_THRESHOLD(
    "unidentified_score_threshold");
const ParameterString<bool> Classification::ALL_CLASSIFIERS("all_classifiers");

Classification::Classification(ParameterInterface *parameters_ptr,
                               const common::CameraTransformation *camera_transform,
                               const EgoVehicle *const ego_vehicle,
                               const road_object_detection::WorldCoordinatesHelper *const world_coordinates_helper)
    : parameters_ptr_(parameters_ptr),
      camera_transform(camera_transform),
      feature_extractor(std::make_unique<FeatureExtractor>(
          parameters_ptr,
          camera_transform,
          ego_vehicle,
          world_coordinates_helper->getHelper())),
      world_coordinates_helper_(world_coordinates_helper) {
  parameters_ptr->registerParam(UNIDENTIFIED_SCORE_THRESHOLD);
  parameters_ptr_->registerParam(ALL_CLASSIFIERS);
  // USE addClassifier() TO ADD NEW CLASSIFIER, DON'T FORGET TO ADD DEBUG
  // CLASSIFIER TO ClassificationDebug
  addClassifier(std::make_unique<JunctionClassifier>(parameters_ptr, camera_transform));
  addClassifier(std::make_unique<CrosswalkClassifier>(camera_transform, parameters_ptr));
  addClassifier(std::make_unique<RoadClosureClassifier>(camera_transform, parameters_ptr));
  addClassifier(std::make_unique<ObstacleClassifierNew>(camera_transform, parameters_ptr));
  addClassifier(std::make_unique<ContourClassifier>(parameters_ptr, camera_transform));
  addClassifier(std::make_unique<StartLineClassifier>(camera_transform, parameters_ptr));
  addClassifier(std::make_unique<NoPassingZoneClassifier>(camera_transform, parameters_ptr));
}

road_object_detection::RoadObjects Classification::classify(
    const std::vector<FeaturePointCluster> &clusters,
    const cv::Mat &camera_image,
    const ros::Time &timestamp,
    common::DynamicPolynomial &middle_lane_polynomial,
    const LineVehiclePoints &points) {
  if (clusters.empty() && !points[LINESPEC_NO_PASSING].empty()) {
    const auto no_passing_line_features = feature_extractor->extractFeatures(
        FeaturePointCluster(*camera_transform, points[LINESPEC_NO_PASSING]),
        camera_image,
        timestamp,
        middle_lane_polynomial,
        points);
    const auto no_passing_zone_classifier = boost::find_if(classifiers, [](const auto &c) {
      return c->getClassifierId() == typeid(NoPassingZoneClassifier).hash_code();
    });

    return (*no_passing_zone_classifier)->classify(no_passing_line_features);
  }

  RoadObjects classifications;
  classifications.reserve(
      clusters.size() *
      (parameters_ptr_->getParam(ALL_CLASSIFIERS) ? classifiers.size() + 1 : 2));
  for (const FeaturePointCluster &cluster : clusters) {
    Features features_of_cluster = feature_extractor->extractFeatures(
        cluster, camera_image, timestamp, middle_lane_polynomial, points);
    boost::push_back(classifications,
                     common::move_range(runClassifiersOnCluster(features_of_cluster)));
  }

  // remove RoadObjects with empty base hull polygon
  const auto rm_it = boost::partition(classifications, [](const auto &rm) {
    return !rm->base_hull_polygon_in_vehicle.empty();
  });

  if (rm_it != classifications.end()) {
    const auto warn = common::toString(rm_it, classifications.end(), "\t\n");
    ROS_WARN_STREAM_THROTTLE(4, "empty base hull polygon for road object(s):" << warn);
    classifications.erase(rm_it, classifications.end());
  }

  return classifications;
}

RoadObjects Classification::runClassifiersOnCluster(const Features &features) {
  RoadObjects road_objects;
  road_objects.push_back(makeUnidentified(features));
  // decide whether full or lean version of road watcher shoul be used
  const bool road_watcher_full = parameters_ptr_->getParam(ALL_CLASSIFIERS);
  if (road_watcher_full) {
    for (const std::unique_ptr<Classifier> &classifier : classifiers) {
      // debug: measure time of classifiers
      //      const ros::WallTime start_time = ros::WallTime::now();
      boost::push_back(road_objects, common::move_range(classifier->classify(features)));
      //      const ros::WallTime end_time = ros::WallTime::now();
      //      const double time_in_usec = 0.001 * (end_time -
      //      start_time).toNSec();

      //      ROS_DEBUG("%f us:   %s", time_in_usec,
      //      typeid(*classifier).name());
    }
  } else {
    auto start_line_classifier = boost::find_if(classifiers, [](const auto &c) {
      return c->getClassifierId() == typeid(StartLineClassifier).hash_code();
    });
    boost::push_back(road_objects,
                     common::move_range((*start_line_classifier)->classify(features)));
  }
  return road_objects;
}

std::unique_ptr<road_object_detection::RoadObject> Classification::makeUnidentified(
    const road_object_detection::Features &features) const {

  std::vector<cv::Point> convex_hull_cv;
  cv::convexHull(toCV(features.cluster.feature_points_img), convex_hull_cv);

  VehiclePoints convex_hull_ground;
  camera_transform->transformImageToGround(toEigen(convex_hull_cv), &convex_hull_ground);

  // Use the centroid of the convex hull as the position
  const VehiclePoint position_in_vehicle = common::mean(convex_hull_ground);
  VehiclePose pose_in_vehicle = Eigen::Affine3d::Identity();
  pose_in_vehicle.translation() = position_in_vehicle;

  return std::make_unique<Unidentified>(features.timestamp,
                                        parameters_ptr_->getParam(UNIDENTIFIED_SCORE_THRESHOLD),
                                        pose_in_vehicle,
                                        convex_hull_ground);
}

void Classification::addClassifier(std::unique_ptr<Classifier> classifier) {
  classifiers.push_back(std::move(classifier));
}
