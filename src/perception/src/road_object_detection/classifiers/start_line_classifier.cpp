#include "start_line_classifier.h"

THIRD_PARTY_HEADERS_BEGIN
#include <ros/console.h>
#include <boost/range/adaptor/transformed.hpp>
#include <boost/range/algorithm.hpp>
#include <opencv2/imgproc.hpp>
THIRD_PARTY_HEADERS_END

#include "../../utils/step_detection.h"
#include "common/basic_statistics.h"
#include "common/console_colors.h"
#include "common/eigen_utils.h"
#include "common/pca_eigen.h"
#include "common/eigen_adaptors.h"
#include "common/basic_statistics.h"
#include "common/make_vector.h"
#include "opencv_eigen_conversions.h"
#include "scan_line.h"

namespace road_object_detection {
//using boost::adaptors::transformed;
//using common::make_vector;

const std::string StartLineClassifier::NAMESPACE("start_line_classifier");

const ParameterString<double>
    StartLineClassifier::MIN_SCORE_RATIO(NAMESPACE + "/min_score_ratio");
const ParameterString<double>
    StartLineClassifier::MAX_ANGLE_DELTA(NAMESPACE + "/max_angle_delta");
const ParameterString<int> StartLineClassifier::HARRIS_CORNER_THRESHOLD(
    NAMESPACE + "/harris_corner_threshold");
const ParameterString<double>
    StartLineClassifier::MEAN_STEP_SIZE(NAMESPACE + "/mean_step_size");
const ParameterString<int>
    StartLineClassifier::HARRIS_BLOCK_SIZE(NAMESPACE + "/harris_block_size");
const ParameterString<int> StartLineClassifier::SOBEL_WINDOW(NAMESPACE +
                                                             "/sobel_window");
const ParameterString<int> StartLineClassifier::HARRIS_K(NAMESPACE +
                                                         "/harris_k");
const ParameterString<double>
    StartLineClassifier::MINIMAL_HARRIS_MAX(NAMESPACE + "/minimal_harris_max");
const ParameterString<int> StartLineClassifier::MIN_NR_POINTS(NAMESPACE +
                                                              "/min_nr_points");
const ParameterString<double> StartLineClassifier::LINE_SHIFT(NAMESPACE +
                                                              "/line_shift");
const ParameterString<int> StartLineClassifier::REFERENCE_FUNCTION_LENGTH(
    NAMESPACE + "/step_det_ref_func_length");
const ParameterString<double>
    StartLineClassifier::STEP_DETECTION_THLD(NAMESPACE + "/step_det_thld");
const ParameterString<double> StartLineClassifier::FIXED_SCORE_START_LINE(
    NAMESPACE + "/fixed_score_start_line");
const ParameterString<bool> StartLineClassifier::STEP_DET_ABS_FLAG(
    NAMESPACE + "/step_det_use_abs");
const ParameterString<double> StartLineClassifier::LANE_WIDTH("lane_width");
const ParameterString<double> StartLineClassifier::UNIDENTIFIED_SCORE_THRESHOLD(
    "unidentified_score_threshold");
const ParameterString<double> StartLineClassifier::HALF_STARTLINE_LENGTH(
    NAMESPACE + "/half_startline_length");

StartLineClassifier::StartLineClassifier(
    const common::CameraTransformation *camera_transformation,
    ParameterInterface *parameter_interface)
    : camera_transform_(camera_transformation),
      parameters_ptr_(parameter_interface) {
  parameters_ptr_->registerParam(MIN_SCORE_RATIO);
  parameters_ptr_->registerParam(MAX_ANGLE_DELTA);
  parameters_ptr_->registerParam(HARRIS_CORNER_THRESHOLD);
  parameters_ptr_->registerParam(MEAN_STEP_SIZE);
  parameters_ptr_->registerParam(HARRIS_BLOCK_SIZE);
  parameters_ptr_->registerParam(SOBEL_WINDOW);
  parameters_ptr_->registerParam(HARRIS_K);
  parameters_ptr_->registerParam(MINIMAL_HARRIS_MAX);
  parameters_ptr_->registerParam(MIN_NR_POINTS);
  parameters_ptr_->registerParam(LINE_SHIFT);
  parameters_ptr_->registerParam(REFERENCE_FUNCTION_LENGTH);
  parameters_ptr_->registerParam(STEP_DETECTION_THLD);
  parameters_ptr_->registerParam(FIXED_SCORE_START_LINE);
  parameters_ptr_->registerParam(STEP_DET_ABS_FLAG);
  parameters_ptr_->registerParam(HALF_STARTLINE_LENGTH);
}

RoadObjects StartLineClassifier::classify(const Features &features) {
  RoadObjects start_line;
  if (!hasFeaturePointsOnBothLanes(features)) {
    ROS_DEBUG("cluster does not contain feature_points on both parts of lane! "
              "Returning 0!");
    //    start_line.push_back(toStartLine(0.0, features, {}));
    start_line.push_back(toStartLine(0.01, features));
    return start_line;
  }

  // define direction vector for start_line
  Eigen::Vector3d direction;

  // if cluster is not recognized as a startline, score has to be 0
  if (!isLine(features, direction)) {
    start_line.push_back(toStartLine(0.01, features));
    return start_line;
  }
  const auto step_size = meanStepSize(direction.unitOrthogonal(), features);
  ROS_DEBUG_STREAM(COLOR_YELLOW << "mean step size: " << step_size << COLOR_DEBUG);
  const double mean_step_size = parameters_ptr_->getParam(MEAN_STEP_SIZE);
  if (step_size > mean_step_size) {
    start_line.push_back(toStartLine(0.01, features));
    return start_line;
  }

  // cropping imagepublic virtual StartLineClassifier
  const cv::Mat roi = features.image_patch.image;
  // harris corner detection
  cv::Mat roi_with_corners;
  const int block_size = parameters_ptr_->getParam(HARRIS_BLOCK_SIZE);
  const int sobel_window = parameters_ptr_->getParam(SOBEL_WINDOW);
  const int harris_k = parameters_ptr_->getParam(HARRIS_K);
  cv::cornerHarris(roi, roi_with_corners, block_size, sobel_window, harris_k, cv::BORDER_DEFAULT);
  double min=0.0, max=0.0;
  cv::minMaxLoc(roi_with_corners, &min, &max);
  const double minimal_harris_response =
      parameters_ptr_->getParam(MINIMAL_HARRIS_MAX);
  if (max < minimal_harris_response) {
    start_line.push_back(toStartLine(0.01, features));
    return start_line;
  }
  // normalizing image
  cv::Mat roi_with_corners_norm;
  cv::normalize(roi_with_corners, roi_with_corners_norm, 0, 255,
                cv::NORM_MINMAX);
  cv::Mat corner_roi_scaled;
  cv::convertScaleAbs(roi_with_corners_norm, corner_roi_scaled);
  const uchar corner_threshold =
      static_cast<uchar>(parameters_ptr_->getParam(HARRIS_CORNER_THRESHOLD));
  CVPoints corner_points;
  for (cv::MatIterator_<uchar> it = corner_roi_scaled.begin<uchar>();
       it != corner_roi_scaled.end<uchar>(); it++) {
    if (*it >= corner_threshold) {
      corner_points.push_back(it.pos() + features.image_patch.position);
    }
  }
  ROS_DEBUG_STREAM(COLOR_YELLOW << " start line classifier found "
                                << corner_points.size() << " corner points." << COLOR_DEBUG);
  // this is very easy, we will see if it works
  const unsigned long minimal_points_nr =
      static_cast<unsigned int>(parameters_ptr_->getParam(MIN_NR_POINTS));

  if (corner_points.size() < minimal_points_nr) {
    const auto score = parameters_ptr_->getParam(FIXED_SCORE_START_LINE) / 3.0;
    start_line.push_back(toStartLine(score, features, direction));
    return start_line;
  }
  const double score = parameters_ptr_->getParam(FIXED_SCORE_START_LINE);
  start_line.push_back(toStartLine(score, features, direction));

  return start_line;
}

size_t StartLineClassifier::getClassifierId() const {
  return typeid(StartLineClassifier).hash_code();
}

bool StartLineClassifier::isLine(const Features &features, Eigen::Vector3d &direction) const {
  using Direction2d = Eigen::Vector2d;
  // computing pca
  const Eigen::MatrixXd feature_points_data =
      common::toMatrix2D(features.cluster.feature_points_vehicle);
  Eigen::MatrixXd components;
  Eigen::VectorXd scores;
  common::pca_svd(feature_points_data, &components, &scores);
  const Direction2d p_c = components.block<2, 1>(0, 0);
  direction = to3D(components.block<2, 1>(0, 1));
  ROS_DEBUG_STREAM(COLOR_MAGENTA << "scores of components are: " << scores << COLOR_DEBUG);
  double pca_angle = std::fabs(common::getAngleToPrincipalComponent(
      Eigen::Vector2d::UnitX(), feature_points_data));
  if (p_c.dot(Eigen::Vector2d::UnitY()) < 0) {
    pca_angle = -pca_angle + M_PI;
    direction = -direction;
  }
  ROS_DEBUG_STREAM(COLOR_MAGENTA << "pca_angle is: " << pca_angle << COLOR_DEBUG);
  ROS_DEBUG_STREAM(COLOR_MAGENTA << "orientation of normal on lane polynom is: "
                                 << std::fabs(features.cluster_center_lane_orientation + M_PI_2)
                                 << COLOR_DEBUG);
  const double angle_delta =
      std::fabs(pca_angle - std::fabs(features.cluster_center_lane_orientation + M_PI_2));
  const double min_score_ratio = parameters_ptr_->getParam(MIN_SCORE_RATIO);
  const double max_angle_delta = parameters_ptr_->getParam(MAX_ANGLE_DELTA);
  return ((scores[0] / scores[1]) > min_score_ratio) &&
         (angle_delta < max_angle_delta);
}

double StartLineClassifier::meanStepSize(const Eigen::Vector3d &pc,
                                         const Features &features) const {
  // creating line with well defined distance around cluster_center with
  // orientation of pc
  // defining start and end point of scan line
  const double shift = parameters_ptr_->getParam(LINE_SHIFT);
  const VehiclePoint start_vec = features.cluster_center3d + (shift / pc.norm()) * pc;
  const VehiclePoint end_vec = features.cluster_center3d - (shift / pc.norm()) * pc;

  // transformation to image coordinates
  const ImagePoint start =
      camera_transform_->transformGroundToImage(start_vec) -
      toEigen(features.image_patch.position);
  const ImagePoint end = camera_transform_->transformGroundToImage(end_vec) -
                         toEigen(features.image_patch.position);
  //  const CVPoints steps_bv = stepPoints(features, start_vec, end_vec);
  //  if (steps_bv.size() > 3) {
  //    VehiclePoints bv_steps_vc;
  //    bv_steps_vc.reserve(steps_bv.size());
  //    boost::transform(steps_bv,
  //                     std::back_inserter(bv_steps_vc),
  //                     [&features](const auto s) {
  //                       return
  //                       features.birdsview_patch.imageToVehicle(toEigen(s));
  //                     });
  //    std::vector<double> step_distances_alt;
  //    step_distances_alt.reserve(bv_steps_vc.size() - 1);
  //    for (auto step_it_alt = std::next(bv_steps_vc.begin());
  //         step_it_alt != bv_steps_vc.end();
  //         step_it_alt++) {
  //      step_distances_alt.push_back((*step_it_alt -
  //      *(std::prev(step_it_alt))).norm());
  //    }
  //    ROS_DEBUG("mean step size of points extracted from bv_patch is %f",
  //              common::mean<double>(step_distances_alt));
  //  }
  const ScanLine step_det_line{start, end};
  // step detection parameters
  const int step_det_func_len =
      parameters_ptr_->getParam(REFERENCE_FUNCTION_LENGTH);
  const double step_det_thld = parameters_ptr_->getParam(STEP_DETECTION_THLD);
  const bool use_abs = parameters_ptr_->getParam(STEP_DET_ABS_FLAG);

  const auto ref_func = step_detection::createReferenceFunction(step_det_func_len);
  ImagePoints steps = step_detection::detectStep(
      features.image_patch.image, step_det_line, ref_func, step_det_thld, use_abs);
  const auto feature_offset = toEigen(features.image_patch.position);
  boost::transform(
      steps,
      steps.begin(),
      [&feature_offset](const auto &s) { return s + feature_offset; });
  VehiclePoints steps_vec;
  camera_transform_->transformImageToGround(steps, &steps_vec);
  if (steps.size() <= 2) {
    return std::numeric_limits<double>::max();
  }
  std::vector<double> step_distances;
  step_distances.reserve(steps_vec.size());
  for (auto step_it = std::next(steps_vec.begin()); step_it != steps_vec.end();
       step_it++) {
    step_distances.push_back((*step_it - *(std::prev(step_it))).norm());
  }
  return common::mean<double>(step_distances);
}

std::unique_ptr<StartLine> StartLineClassifier::toStartLine(const double score,
                                                            const Features &features,
                                                            const Eigen::Vector3d &direction) {
  VehiclePoints hull_points;
  const auto min_score = parameters_ptr_->getParam(UNIDENTIFIED_SCORE_THRESHOLD);
  const VehiclePose pose_in_vehicle =
      score > min_score
          ? setHullPointsAndPose(hull_points, features, direction)
          : VehiclePose(Eigen::Translation3d(features.cluster_center3d) *
                        Eigen::AngleAxisd(features.cluster_center_lane_orientation,
                                          Eigen::Vector3d::UnitZ()));

  if (hull_points.empty()) {
    CVPoints convex_hull_image;
    cv::convexHull(toCV(features.cluster.feature_points_img), convex_hull_image);
    camera_transform_->transformImageToGround(toEigen(convex_hull_image), &hull_points);
  }

  return std::make_unique<StartLine>(features.timestamp, score, pose_in_vehicle, hull_points);
}




bool StartLineClassifier::hasFeaturePointsOnBothLanes(const Features &features) const {
  // transformation from vehicle_frame to cluster_center_frame
  const Eigen::Affine3d cc_T_vehicle =
      (Eigen::Translation3d{features.cluster_center_middle_lane_foot_point} *
       Eigen::AngleAxisd{features.cluster_center_lane_orientation,
                         Eigen::Vector3d::UnitZ()}).inverse();
  // count elements with y-coordinate greater than zero in
  // cluster_center_frame--> points on left lane
  const std::size_t nr_points_on_left = boost::count_if(
      features.cluster.feature_points_vehicle | common::eigen_transformed(cc_T_vehicle),
      [](const auto &fp_in_cc) { return fp_in_cc.x() > 0; });
  const std::size_t nr_points_on_right =
      features.cluster.feature_points_vehicle.size() - nr_points_on_left;
  const std::size_t min_nr_per_lane = 10;
  ROS_DEBUG(
      "number of feature_points on left part of lane is %zu; on right part: "
      "%zu",
      nr_points_on_left, nr_points_on_right);
  return nr_points_on_left > min_nr_per_lane &&
         nr_points_on_right > min_nr_per_lane;
}

VehiclePose StartLineClassifier::setHullPointsAndPose(VehiclePoints &hull_points,
                                                      const Features &features,
                                                      const Eigen::Vector3d &direction) const {
  const double lane_width = parameters_ptr_->getParam(LANE_WIDTH);
  // get left and right points, assuming that lane width is fitting

  Eigen::Matrix3d rotation;
  rotation = Eigen::Quaterniond::FromTwoVectors(Eigen::Vector3d::UnitX(), direction);
  const Eigen::Affine3d startline_pose =
      Eigen::Translation3d{features.cluster_center3d} * rotation;
  // unit vector in direction of crosswalk
  const VehiclePoint unit_dir =
      (startline_pose.linear() * VehiclePoint::UnitX()).normalized();
  const VehiclePoint right =
      startline_pose.translation() - lane_width * unit_dir.unitOrthogonal();
  const VehiclePoint left =
      startline_pose.translation() + lane_width * unit_dir.unitOrthogonal();

  // length of crosswalk from cup regulations
  const double half_length = parameters_ptr_->getParam(HALF_STARTLINE_LENGTH);
  // return all points
  hull_points = {{right - half_length * unit_dir,
                  left - half_length * unit_dir,
                  left + half_length * unit_dir,
                  right + half_length * unit_dir}};

  return startline_pose;
}


//CVPoints StartLineClassifier::stepPoints(const Features &features,
//                                         const VehiclePoint & /*start_dir*/,
//                                         const VehiclePoint & /*end_dir*/) const {
//  const uchar white_pixel = 255;
//  const uchar black_pixel = 0;
//
//
//  const int thresh_low = 75;
//  const int thresh_high = 100;
//  const int mask_size = 3;
//
//  auto birdsview_patch = *features.birdsview_patch.getImage();
//
//  const cv::Rect start_line_roi = bvROI(features);
//
//  const cv::Point start_pt{0, 0};
//  //      toCV(features.birdsview_patch.vehicleToImage(start_dir)) -
//  //      start_line_roi.tl();
//  const cv::Point end_pt = start_line_roi.br() - start_line_roi.tl();
//  //      toCV(features.birdsview_patch.vehicleToImage(end_dir)) -
//  //      start_line_roi.tl();
//
//  const cv::Mat start_line_patch = birdsview_patch(start_line_roi);
//  cv::Mat roi_after_canny;
//
//  cv::Canny(start_line_patch, roi_after_canny, thresh_low, thresh_high, mask_size, true);
//  cv::LineIterator canny_it{roi_after_canny, start_pt, end_pt};
//  CVPoints steps;
//  for (int i = 1; i < canny_it.count - 2; i++) {
//    const auto actual = **canny_it;
//    const auto next = **(++canny_it);
//    if (actual >= white_pixel && next <= black_pixel) {
//      steps.push_back(canny_it.pos());
//    }
//  }
//  return steps;
//}

//cv::Rect StartLineClassifier::bvROI(const Features &features) const {
//  // transform feature_points to CVPoints in birdsview frame
//  const CVPoints feature_points_bv = common::make_vector(
//      features.cluster.feature_points_vehicle | transformed([&features](const auto &fp) {
//        return toCV(features.birdsview_patch.vehicleToImage(fp));
//      }));
//  return cv::boundingRect(feature_points_bv);
//}
//

} // namespace road_object_detection

