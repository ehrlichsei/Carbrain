#include "crosswalk_classifier_debug.h"

#include "opencv_eigen_conversions.h"

namespace road_object_detection {

CrosswalkClassifierDebug::CrosswalkClassifierDebug(DebugImages *debug_images,
                                                   const common::CameraTransformation *cam_transform,
                                                   ParameterInterface *parameter_interface)
    : ClassifierDebug(debug_images),
      CrosswalkClassifier(cam_transform, parameter_interface) {}
RoadObjects CrosswalkClassifierDebug::classify(const Features &features) {
  auto ret = CrosswalkClassifier::classify(features);

  return ret;
}

std::vector<CVPoints> CrosswalkClassifierDebug::onlyPointsOnLane(
    const std::vector<CVPoints> &crosswalk_lines, const Features &features) const {
  const auto filtered = CrosswalkClassifier::onlyPointsOnLane(crosswalk_lines, features);

  for (const auto &points : filtered) {
    for (const auto &point : points) {
      cv::circle(*debug_images->getBirdsviewPatch(features.cluster.id),
                 point,
                 2,
                 cv::Vec3b(0, 255, 0),
                 -1);
    }
  }

  return filtered;
}

CVPoints CrosswalkClassifierDebug::linePoints(const Features &features, const int row_idx) const {
  const auto lp = CrosswalkClassifier::linePoints(features, row_idx);

  const auto closest_point_bv = features.birdsview_patch.vehicleToImage(closest_point);

  cv::line(*debug_images->getBirdsviewPatch(features.cluster.id),
           cv::Point(0, closest_point_bv.y() - row_idx),
           cv::Point(debug_images->getBirdsviewPatch(features.cluster.id)->cols,
                     closest_point_bv.y() - row_idx),
           cv::Vec3b(0, 0, 255));

  return lp;
}

std::pair<common::DynamicPolynomial, common::DynamicPolynomial>
CrosswalkClassifierDebug::extractBoundingPolynomials(const Features &features) const {
  const auto ret = CrosswalkClassifier::extractBoundingPolynomials(features);

  VehiclePoints left, right;

  const auto step = 0.02;
  const common::DiscretizationParams params = {
      closest_point.x(), std::max(closest_point.x() + step, farest_point.x()), step};

  common::discretisize(ret.first, params, &left);
  common::discretisize(ret.second, params, &right);

  const std::vector<cv::Point> img_boundaries = {
      cv::Point(0, 0),
      cv::Point(0, features.birdsview_patch.getImage()->rows),
      cv::Point(features.birdsview_patch.getImage()->cols,
                features.birdsview_patch.getImage()->rows),
      cv::Point(features.birdsview_patch.getImage()->cols, 0)};

  for (const auto &p : left) {
    cv::circle(*debug_images->getCameraImage(),
               toCV(camera_transform->transformGroundToImage(p)),
               2,
               cv::Scalar(255, 0, 255),
               -1);
    const cv::Point as_cv = toCV(features.birdsview_patch.vehicleToImage(p));
    if (cv::pointPolygonTest(img_boundaries, cv::Point2f(as_cv.x, as_cv.y), true) > 0) {
      cv::circle(*debug_images->getBirdsviewPatch(features.cluster.id),
                 as_cv,
                 2,
                 cv::Vec3b(255, 0, 255),
                 -1);
    }
  }

  for (const auto &p : right) {
    cv::circle(*debug_images->getCameraImage(),
               toCV(camera_transform->transformGroundToImage(p)),
               2,
               cv::Scalar(255, 0, 255),
               -1);
    const cv::Point as_cv = toCV(features.birdsview_patch.vehicleToImage(p));
    if (cv::pointPolygonTest(img_boundaries, cv::Point2f(as_cv.x, as_cv.y), true) > 0) {
      cv::circle(*debug_images->getBirdsviewPatch(features.cluster.id),
                 as_cv,
                 2,
                 cv::Vec3b(255, 0, 255),
                 -1);
    }
  }


  return ret;
}

}  // namespace road_object_detection
