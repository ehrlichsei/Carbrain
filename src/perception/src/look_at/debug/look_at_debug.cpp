#include "look_at_debug.h"
#include "../../road_object_detection/debug/features/feature_extractor_debug.h"
#include "common/move_range.h"
#include "distinct_colors.h"

THIRD_PARTY_HEADERS_BEGIN
#include <boost/range/algorithm_ext/push_back.hpp>
THIRD_PARTY_HEADERS_END

namespace look_at {
LookAtDebug::LookAtDebug(LookAt &&look_at, road_object_detection::DebugImages *debug_images)
    : LookAt(std::move(look_at)), debug_img_(debug_images->getCameraImage()) {
  feature_extractor_ = std::make_unique<road_object_detection::FeatureExtractorDebug>(
      std::move(*feature_extractor_), debug_images);
}

ClassificationResults LookAtDebug::classifyROI(
    const cv::Mat &img,
    const ros::Time &timestamp,
    const RegionsToClassify &regions,
    const LineVehiclePoints &lane_points,
    const std::unique_ptr<road_object_detection::Classifier> &classifier) {
  clusters_.clear();
  ROS_DEBUG("classifyROI called in LookAtDebug! Classifier is of type %zu",
            classifier->getClassifierId());
  auto results = LookAt::classifyROI(img, timestamp, regions, lane_points, classifier);
  visualize(clusters_);

  // visuaize base_hull_poly
  for (auto &result : results) {
    for (const auto &ro : result.detected_objects) {
      ImagePoints polygon_image_points_eigen;
      camera_transformation_->transformGroundToImage(
          ro->base_hull_polygon_in_vehicle, &polygon_image_points_eigen);

      //      const auto position_in_img =
      //      camera_transformation_->transformGroundToImage(
      //          VehiclePoint{ro->pose_in_vehicle.translation()});

      std::vector<cv::Point> polygon_image_points_cv = toCV(polygon_image_points_eigen);
      if (polygon_image_points_cv.size() >= 1) {
        cv::polylines(
            *debug_img_, polygon_image_points_cv, true, cv::Scalar(0, 255, 0), 1);
        cv::putText(*debug_img_,
                    ro->getDescription(),
                    polygon_image_points_cv[0],
                    2,
                    1,
                    cv::Scalar(0, 255, 0),
                    2);
      }
    }
  }

  return results;
}

VehiclePoints LookAtDebug::allPointsToMiddle(LineVehiclePoints points) const {
  const auto middle_points = LookAt::allPointsToMiddle(points);
  ImagePoints middle_points_img;
  camera_transformation_->transformGroundToImage(middle_points, &middle_points_img);
  //  visualize(middle_points_img, cv::Scalar(255, 0, 0));
  return middle_points;
}

ImagePoints LookAtDebug::apply1dGradientDetector(const cv::Mat &img,
                                                 const ScanLines &scan_lines) const {
  const auto featurepoints = LookAt::apply1dGradientDetector(img, scan_lines);
  //  visualize(featurepoints,cv::Scalar(0,200,0,80));
  return featurepoints;
}

std::vector<FeaturePointCluster> LookAtDebug::clusterFeaturePoints(const FeaturePointCluster &raw) const {
  const auto cluster = LookAt::clusterFeaturePoints(raw);
  boost::push_back(clusters_, common::move_range(cluster));
  return cluster;
}

ScanLines LookAtDebug::createScanLineGrid(const RegionToClassify &roi,
                                          const double step_size) const {
  //  const Eigen::Affine3d vehicle_T_world =
  //  tf_helper_->getTransform().inverse(); const Eigen::Affine3d roi_pose_vec =
  //  vehicle_T_world * roi.pose; const VehiclePoint x_vec_end =
  //      roi_pose_vec.translation() +
  //      0.5 * (roi_pose_vec.linear() * VehiclePoint::UnitX()).normalized();
  //  const VehiclePoint y_vec_end =
  //      roi_pose_vec.translation() +
  //      0.5 * (roi_pose_vec.linear() * VehiclePoint::UnitY()).normalized();
  //  const ImagePoint base = camera_transformation_->transformGroundToImage(
  //      VehiclePoint{roi_pose_vec.translation()});

  //  cv::arrowedLine(*debug_img_,
  //                  toCV(base),
  //                  toCV(camera_transformation_->transformGroundToImage(x_vec_end)),
  //                  cv::Scalar(0, 255, 255));
  //  cv::arrowedLine(*debug_img_,
  //                  toCV(base),
  //                  toCV(camera_transformation_->transformGroundToImage(y_vec_end)),
  //                  cv::Scalar(0, 0, 255));

  const auto scanlines = LookAt::createScanLineGrid(roi, step_size);
  visualize(scanlines);
  return scanlines;
}


void LookAtDebug::visualize(const ScanLines &scan_lines) const {
  for (const ScanLine &scan_line : scan_lines) {
    cv::line(
        *debug_img_, toCV(scan_line.start), toCV(scan_line.end), cv::Scalar(255, 0, 0, 10), 1);
  }
}

void LookAtDebug::visualize(const ImagePoints &feature_points, const cv::Scalar &color) const {
  for (const ImagePoint &feature_point : feature_points) {
    cv::circle(*debug_img_, toCV(feature_point), 3, color, 1);
  }
}

void LookAtDebug::visualize(const std::vector<FeaturePointCluster> &clusters) const {
  //  const auto colors = perception::nDistinctColors(clusters.size(), 240.0f);
  for (const auto &cluster : clusters) {
    visualize(cluster.feature_points_img, cv::Scalar(0, 0, 255));
  }
}
}  // namespace look_at
