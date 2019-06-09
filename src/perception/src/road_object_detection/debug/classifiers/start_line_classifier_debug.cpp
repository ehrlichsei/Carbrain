#include "start_line_classifier_debug.h"
#include "opencv_eigen_conversions.h"

THIRD_PARTY_HEADERS_BEGIN
#include <boost/range/algorithm.hpp>
THIRD_PARTY_HEADERS_END

#include "../../../utils/step_detection.h"
#include "common/basic_statistics.h"
#include "common/console_colors.h"
#include "common/eigen_utils.h"
#include "common/pca_eigen.h"
#include "opencv_eigen_conversions.h"
#include "scan_line.h"

namespace road_object_detection {

StartLineClassifierDebug::StartLineClassifierDebug(DebugImages *debug_images,
                                                   const common::CameraTransformation *cam_transform,
                                                   ParameterInterface *parameter_interface)
    : ClassifierDebug(debug_images),
      StartLineClassifier(cam_transform, parameter_interface) {}

RoadObjects StartLineClassifierDebug::classify(const Features &features) {
  RoadObjects ret = StartLineClassifier::classify(features);
  //  draw(debug_images->getCameraImage(), features);
  return ret;
}

bool StartLineClassifierDebug::isLine(const Features &features,
                                      Eigen::Vector3d &direction) const {
  const bool ret = StartLineClassifier::isLine(features, direction);
  pc = direction;
  return ret;
}

// double StartLineClassifierDebug::meanStepSize(const Eigen::Vector2d &pc,
//                                                    const Features &features)
//                                                    const {
////  using Direction3d = Eigen::Vector3d;
////  steps_debug.clear();
////  // creating line with well defined distance around cluster_center with
////  // orientation of pc
////  const Direction3d pc_3d = to3D(pc);
////  // defining start and end point of scan line
////  const double shift = parameters_ptr_->getParam(LINE_SHIFT);
////  const VehiclePoint start_vec = features.cluster_center3d + (shift /
/// pc_3d.norm()) * pc_3d;
////  const VehiclePoint end_vec = features.cluster_center3d - (shift /
/// pc_3d.norm()) * pc_3d;
////  // transformation to image coordinates
////  const ImagePoint start =
/// camera_transform_->transformGroundToImage(start_vec) -
////                           toEigen(features.image_patch.position);
////  const ImagePoint end = camera_transform_->transformGroundToImage(end_vec)
///-
////                         toEigen(features.image_patch.position);
////  const ScanLine step_det_line{start, end};
////  // step detection parameters
////  const int step_det_func_len =
/// parameters_ptr_->getParam(REFERENCE_FUNCTION_LENGTH);
////  const double step_det_thld =
/// parameters_ptr_->getParam(STEP_DETECTION_THLD);
////  std::vector<float> ref_func =
/// step_detection::createReferenceFunction(step_det_func_len);
////  ImagePoints st = step_detection::detectStep(
////      features.image_patch.image, step_det_line, ref_func, step_det_thld,
/// false);
////  const auto feature_offset = toEigen(features.image_patch.position);
////  ImagePoints steps;
////  boost::transform(
////      st,
////      std::back_inserter(steps),
////      [&feature_offset](const auto &s) { return s + feature_offset; });
////  VehiclePoints steps_vec;
////  camera_transform_->transformImageToGround(steps, &steps_vec);
////  for (const auto &step : steps) {
////    steps_debug.push_back(imagePointToCvPoint(step));
////  }
////  if (steps.size() <= 2) {
////    return std::numeric_limits<double>::max();
////  }
////  std::vector<double> step_distances;
////  step_distances.reserve(steps.size() - 1);
////  for (auto step_it = std::next(steps_vec.begin()); step_it !=
/// steps_vec.end(); step_it++) {
////    step_distances.push_back((*step_it - *(std::prev(step_it))).norm());
////  }
//  return common::mean<double>(step_distances);
//}

// CVPoints StartLineClassifierDebug::stepPoints(const Features &features,
//                                              const VehiclePoint &start_dir,
//                                              const VehiclePoint &end_dir) const {
//  const auto steps = StartLineClassifier::stepPoints(features, start_dir, end_dir);
//  cv::circle(*debug_images->getBirdsviewPatch(features.cluster.id),
//             toCV(features.birdsview_patch.vehicleToImage(start_dir)),
//             2,
//             cv::Vec3b(0, 0, 255),
//             -1);
//  cv::circle(*debug_images->getBirdsviewPatch(features.cluster.id),
//             toCV(features.birdsview_patch.vehicleToImage(end_dir)),
//             2,
//             cv::Vec3b(255, 0, 0),
//             -1);
//  for (const auto &step : steps) {
//    cv::circle(*debug_images->getBirdsviewPatch(features.cluster.id),
//               step,
//               2,
//               cv::Vec3b(0, 255, 0),
//               -1);
//  }
//  return steps;
//}
//
// cv::Rect StartLineClassifierDebug::bvROI(const Features &features) const {
//  const auto roi = StartLineClassifier::bvROI(features);
//  ROS_DEBUG("drawing start line bounding rect");
//  cv::rectangle(*debug_images->getBirdsviewPatch(features.cluster.id),
//                roi.tl(),
//                roi.br(),
//                cv::Vec3b(255, 0, 0));
//  return roi;
//}

void StartLineClassifierDebug::draw(cv::Mat *debug_image, const Features &features) const {
  VehiclePoint end = features.cluster_center3d + 0.1 * pc / pc.norm();
  ImagePoint end_img = camera_transform_->transformGroundToImage(end);
  cv::arrowedLine(*debug_image,
                  imagePointExactToCvPoint(features.cluster_center2i),
                  toCV(end_img),
                  cv::Scalar(255, 0, 255),
                  2);
  for (const auto &step : steps_debug) {
    cv::circle(*debug_image, step, 2, cv::Scalar(0, 0, 255), -1);
  }
}


}  // namespace road_object_detection
