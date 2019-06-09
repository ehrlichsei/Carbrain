#include "classification_debug.h"
#include "../../road_object_detection/debug/features/feature_extractor_debug.h"

#include "../../road_object_detection/debug/classifiers/contour_classifier_debug.h"
#include "../../road_object_detection/debug/classifiers/crosswalk_classifier_debug.h"
#include "../../road_object_detection/debug/classifiers/junction_classifier_debug.h"
#include "../../road_object_detection/debug/classifiers/no_passing_zone_classifier_debug.h"
#include "../../road_object_detection/debug/classifiers/obstacle_classifier_new_debug.h"
#include "../../road_object_detection/debug/classifiers/road_closure_classifier_debug.h"
#include "../../road_object_detection/debug/classifiers/start_line_classifier_debug.h"


#include "common/adaptors.h"
#include "common/basic_statistics_eigen.h"
#include "common/best_score.h"
#include "common/discretisize.h"
#include "common/join.h"

using namespace road_object_detection;
using namespace contour_classifier;

ClassificationDebug::ClassificationDebug(
    Classification &&classification,
    ParameterInterface *parameters_ptr,
    const common::CameraTransformation *camera_transform,
    // const WorldCoordinatesHelper *const world_coordinates_helper,
    DebugImages *debug_images)
    : Classification(std::move(classification)), debug_images(debug_images) {
  feature_extractor = std::make_unique<FeatureExtractorDebug>(
      std::move(*feature_extractor), debug_images);
  // USE addClassifier() TO ADD NEW CLASSIFIER, DON'T FORGET TO ADD NORMAL
  // CLASSIFIER TO Classification
  addClassifier(std::make_unique<CrosswalkClassifierDebug>(
      debug_images, camera_transform, parameters_ptr));
  addClassifier(std::make_unique<RoadClosureClassifierDebug>(
      debug_images, camera_transform, parameters_ptr));
  addClassifier(std::make_unique<StartLineClassifierDebug>(
      debug_images, camera_transform, parameters_ptr));
  addClassifier(std::make_unique<ContourClassifierDebug>(
      parameters_ptr, camera_transform, debug_images));
  addClassifier(std::make_unique<JunctionClassifierDebug>(
      parameters_ptr, camera_transform, debug_images));
  addClassifier(std::make_unique<NoPassingZoneClassifierDebug>(
      camera_transform, parameters_ptr, debug_images));
  addClassifier(std::make_unique<ObstacleClassifierNewDebug>(
      camera_transform, parameters_ptr, debug_images));
  ROS_DEBUG("number of classifiers: %zu", classifiers.size());
}

road_object_detection::RoadObjects ClassificationDebug::classify(
    const std::vector<FeaturePointCluster> &clusters,
    const cv::Mat &camera_image,
    const ros::Time &timestamp,
    common::DynamicPolynomial &middle_lane_polynomial,
    const LineVehiclePoints &points) {
  RoadObjects ret(Classification::classify(
      clusters, camera_image, timestamp, middle_lane_polynomial, points));
  const auto lp = join(points);
  if (lp.size() < 2) {
    return ret;
  }
  const auto toX = [](const auto &p) { return p[0]; };
  const auto start_end = common::minmax_element(lp | transformed(std::cref(toX)));

  const common::DiscretizationParams params{*start_end.first, *start_end.second, 0.02};
  VehiclePoints mid_lane_pol;
  common::discretisize(middle_lane_polynomial, params, &mid_lane_pol);

  for (const auto &p : mid_lane_pol) {
    cv::circle(*debug_images->getCameraImage(),
               toCV(camera_transform->transformGroundToImage(p)),
               2,
               cv::Scalar(0, 255, 0));
  }

  for (auto &road_object : ret) {
    if (road_object->score > 0.2) {
      Eigen::Vector2i position_in_img = camera_transform->transformVehicleToImage(
          common::mean<VehiclePoint>(road_object->base_hull_polygon_in_vehicle));

      cv::putText(*debug_images->getCameraImage(),
                  road_object->getDescription(),
                  cv::Point2i(position_in_img(0), position_in_img(1)),
                  2,
                  1,
                  cv::Scalar(0, 255, 0),
                  2);

      cv::circle(*debug_images->getCameraImage(),
                 cv::Point2i(position_in_img(0), position_in_img(1)),
                 5,
                 cv::Scalar(0, 255, 0),
                 5);
    }
  }
  // visualize pose of best classification
  //  for (const auto &road_object : ret) {
  //    const VehiclePoint start_point =
  //    road_object->pose_in_vehicle.translation();
  //    const VehiclePoint end_point =
  //        start_point + road_object->pose_in_vehicle.linear() *
  //        Eigen::Vector3d::UnitX();
  //    const ImagePoint start_img =
  //    camera_transform->transformGroundToImage(start_point);
  //    const ImagePoint end_img =
  //    camera_transform->transformGroundToImage(end_point);
  //    cv::arrowedLine(*debug_images->getCameraImage(),
  //                    toCV(start_img),
  //                    toCV(end_img),
  //                    cv::Scalar(0, 255, 255),
  //                    4);
  //  }
  return ret;
}

road_object_detection::RoadObjects ClassificationDebug::runClassifiersOnCluster(
    const road_object_detection::Features &features) {
  RoadObjects ret(Classification::runClassifiersOnCluster(features));

  //  for (const VehiclePoint &left_point : features.points_left) {
  //    Eigen::Vector2i
  //    lp(camera_transform->transformGroundToImage(left_point));
  //    cv::circle(*debug_images->getCameraImage(),
  //               cv::Point2i(lp(0), lp(1)),
  //               1,
  //               cv::Scalar(255, 0, 0),
  //               -1);
  //  }
  //  for (const VehiclePoint &right_point : features.points_right) {
  //    Eigen::Vector2i
  //    rp(camera_transform->transformGroundToImage(right_point));
  //    cv::circle(*debug_images->getCameraImage(),
  //               cv::Point2i(rp(0), rp(1)),
  //               1,
  //               cv::Scalar(255, 0, 0),
  //               -1);
  //  }
  for (const auto &ro : ret) {
    if (ro->score >= 0.2) {
      ImagePoints polygon_image_points_eigen;
      camera_transform->transformGroundToImage(ro->base_hull_polygon_in_vehicle,
                                               &polygon_image_points_eigen);
      std::vector<cv::Point> polygon_image_points_cv = toCV(polygon_image_points_eigen);
      cv::polylines(*debug_images->getCameraImage(),
                    polygon_image_points_cv,
                    true,
                    cv::Scalar(0, 255, 0));
    }
  }
  return ret;
}

void ClassificationDebug::addClassifier(std::unique_ptr<Classifier> classifier) {
  auto i = std::find_if(classifiers.begin(),
                        classifiers.end(),
                        [&classifier](std::unique_ptr<Classifier> &c) {
                          return classifier->getClassifierId() == c->getClassifierId();
                        });
  if (i == classifiers.end()) {
    classifiers.push_back(std::move(classifier));
    ROS_WARN(
        "adding debug classifier without added a non debug classifier with "
        "same type in class Classification");
  } else {
    *i = std::move(classifier);
  }
}
