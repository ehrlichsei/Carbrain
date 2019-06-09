#include "feature_extractor.h"
#include <common/macros.h>

THIRD_PARTY_HEADERS_BEGIN
#include <ros/console.h>
#include <boost/range/adaptor/transformed.hpp>
#include <boost/range/algorithm/min_element.hpp>
THIRD_PARTY_HEADERS_END

#include "../../utils/foot_finder.h"
#include "common/basic_statistics_eigen.h"
#include "common/eigen_adaptors.h"
#include "common/eigen_functors.h"
#include "common/eigen_utils.h"
#include "common/make_vector.h"
#include "opencv2/core/eigen.hpp"
#include "opencv_eigen_conversions.h"

namespace road_object_detection {

using boost::adaptors::transformed;

const ParameterString<double> FeatureExtractor::EPSILON_NEWTON_METHOD(
    "epsilon_newton_method");
const ParameterString<int> FeatureExtractor::MAX_ITERATIONS_NEWTON_METHOD(
    "max_iterations_newton_method");

const ParameterString<double> FeatureExtractor::PIXEL_WIDTH("pixel_width");
const ParameterString<double> FeatureExtractor::OFFSET_BEFORE_CENTER(
    "offset_before_center");
const ParameterString<double> FeatureExtractor::OFFSET_AFTER_CENTER(
    "offset_after_center");
const ParameterString<double> FeatureExtractor::OFFSET_RIGHT_OF_CENTER(
    "offset_right_of_center");
const ParameterString<double> FeatureExtractor::OFFSET_LEFT_OF_CENTER(
    "offset_left_of_center");

const ParameterString<double> FeatureExtractor::THRESHOLD1("threshold1");
const ParameterString<double> FeatureExtractor::THRESHOLD2("threshold2");

const ParameterString<int> FeatureExtractor::EXPAND_X("expand_x");
const ParameterString<int> FeatureExtractor::EXPAND_Y("expand_y");

const ParameterString<double> FeatureExtractor::LANE_WIDTH("lane_width");

FeatureExtractor::FeatureExtractor(ParameterInterface *parameters_ptr,
                                   const common::CameraTransformation *camera_tranformation,
                                   const EgoVehicle *ego_vehicle,
                                   const tf_helper::TFHelperInterface<double> *const tf_helper)
    : parameters_ptr_(parameters_ptr),
      camera_transform_(camera_tranformation),
      ego_vehicle(ego_vehicle),
      tf_helper_(tf_helper) {
  registerParameters(parameters_ptr);
}

void FeatureExtractor::registerParameters(ParameterInterface *parameters_ptr) {
  parameters_ptr->registerParam(EPSILON_NEWTON_METHOD);
  parameters_ptr->registerParam(MAX_ITERATIONS_NEWTON_METHOD);
  parameters_ptr->registerParam(PIXEL_WIDTH);
  parameters_ptr->registerParam(OFFSET_BEFORE_CENTER);
  parameters_ptr->registerParam(OFFSET_AFTER_CENTER);
  parameters_ptr->registerParam(OFFSET_RIGHT_OF_CENTER);
  parameters_ptr->registerParam(OFFSET_LEFT_OF_CENTER);
  //  parameters_ptr->registerParam(THRESHOLD1);
  //  parameters_ptr->registerParam(THRESHOLD2);
  parameters_ptr->registerParam(EXPAND_X);
  parameters_ptr->registerParam(EXPAND_Y);
  parameters_ptr->registerParam(LANE_WIDTH);
}

Features FeatureExtractor::extractFeatures(const FeaturePointCluster &cluster,
                                           const cv::Mat &camera_image,
                                           const ros::Time &timestamp,
                                           const common::DynamicPolynomial &middle_lane_polynomial,
                                           const LineVehiclePoints &points,
                                           const boost::optional<ROIParameters> &roi_params) {

  const Eigen::Vector2d cluster_center2i = calculateCenter2d(cluster);
  const Eigen::Vector3d cluster_center3d = calculateCenter3d(cluster);
  const Eigen::Vector3d cluster_nearest_point = calculateNearestPoint(cluster);

  const Eigen::Vector3d cluster_center_middle_lane_foot_point =
      calculateCenterFootPoint(cluster_center3d, middle_lane_polynomial);

  const ImagePatch image_patch = calculateImagePatch(camera_image, cluster);
  // const ImagePatch canny_patch = calculateCannyPatch(image_patch);

  const double cluster_center_lane_orientation = calculateCenterLaneOrientation(
      cluster_center_middle_lane_foot_point, middle_lane_polynomial);

  VehiclePoint birdsview_ref_point;
  const double lane_width = parameters_ptr_->getParam(LANE_WIDTH);
  // when cluster center is located at a distance bigger than lane width, look
  // at service
  // has been called before. Therefore, another reference point is taken for
  // birdsview patch, if this is the case

  if ((cluster_center_middle_lane_foot_point - cluster_center3d).norm() < lane_width) {
    birdsview_ref_point =
        calculateCenterFootPoint(cluster_nearest_point, middle_lane_polynomial);
  } else {
    ROS_WARN_THROTTLE(
        2,
        "distance of cluster center to middle lane polynomial is %f",
        (cluster_center_middle_lane_foot_point - cluster_center3d).norm());
    birdsview_ref_point = cluster_nearest_point;
  }
  const BirdsviewPatch birdsview_patch = calculateBirdsviewPatch(
      camera_image, cluster_center_lane_orientation, birdsview_ref_point);

  const boost::optional<ROI> roi_with_img =
      roi_params
          ? boost::make_optional<ROI>(calculateRoi(camera_image, roi_params.get()))
          : boost::none;

  return Features(timestamp,
                  cluster,
                  cluster_center2i,
                  cluster_center3d,
                  middle_lane_polynomial,
                  points,
                  cluster_center_middle_lane_foot_point,
                  cluster_center_lane_orientation,
                  image_patch,
                  // canny_patch,
                  birdsview_patch,
                  camera_image,
                  roi_with_img);
}

ImagePatch FeatureExtractor::calculateImagePatch(const cv::Mat &camera_image,
                                                 const FeaturePointCluster &cluster) {
  cv::Rect bounding_rect =
      cv::boundingRect(imagePointToCvPoint(cluster.feature_points_img));
  int expand_x = parameters_ptr_->getParam(EXPAND_X);
  int expand_y = parameters_ptr_->getParam(EXPAND_Y);
  cv::Size expand = cv::Size(2 * expand_x, 2 * expand_y);
  cv::Point centering = cv::Point(expand_x, expand_y);
  cv::Rect patch_rect = bounding_rect + expand - centering;
  patch_rect &= cv::Rect(cv::Point(0, 0), camera_image.size());  // crop rect to image
  return ImagePatch(patch_rect.tl(), camera_image(patch_rect));
}

ImagePatch FeatureExtractor::calculateCannyPatch(const ImagePatch &image_patch) {
  double threshold1 = parameters_ptr_->getParam(THRESHOLD1);
  double threshold2 = parameters_ptr_->getParam(THRESHOLD2);

  cv::Mat canny;
  cv::Canny(image_patch.image, canny, threshold1, threshold2);
  return ImagePatch(image_patch.position, canny);
}

Eigen::Affine3d extrinsicCalibration(const common::CameraTransformation *const camera_transform) {
  Eigen::Affine3d camera_to_vehicle = Eigen::Affine3d::Identity();
  camera_to_vehicle.linear() = camera_transform->getRotationMatrix();
  camera_to_vehicle.translation() = camera_transform->getTranslationVector();
  return camera_to_vehicle;
}

Eigen::Matrix3d createHomography(const Eigen::Affine3d &extrinsic,
                                 const Eigen::Matrix3d &intrinsic) {
  const Eigen::Matrix3d R = extrinsic.linear();
  const Eigen::Vector3d t = extrinsic.translation();
  // see Multiple View Geometry by Hartley and Zisserman page 196
  Eigen::Matrix3d r1_r2_t;
  r1_r2_t << R.col(0), R.col(1), t;
  const Eigen::Matrix3d H = intrinsic * r1_r2_t;
  const Eigen::Matrix3d H_inv = H.fullPivHouseholderQr().inverse();
  return H_inv;
}

BirdsviewPatch FeatureExtractor::calculateBirdsviewPatch(const cv::Mat &camera_image,
                                                         double cluster_center_lane_orientation,
                                                         const VehiclePoint &cluster_center3d) {
  double pixel_width = parameters_ptr_->getParam(PIXEL_WIDTH);  // meters
  double offset_before_center = parameters_ptr_->getParam(OFFSET_BEFORE_CENTER);  // meters
  double offset_after_center = parameters_ptr_->getParam(OFFSET_AFTER_CENTER);  // meters
  double offset_right_of_center = parameters_ptr_->getParam(OFFSET_RIGHT_OF_CENTER);  // meters
  double offset_left_of_center = parameters_ptr_->getParam(OFFSET_LEFT_OF_CENTER);  // meters

  BirdsviewPatch patch(cluster_center3d,
                       cluster_center_lane_orientation,
                       offset_before_center,
                       offset_after_center,
                       offset_left_of_center,
                       offset_right_of_center,
                       pixel_width);

  const Eigen::Affine3d patch_to_camera =
      extrinsicCalibration(camera_transform_) * patch.vehicle_to_bv_patch_.inverse();
  const Eigen::Matrix3d H_inv = createHomography(
      patch_to_camera, camera_transform_->getIntrinsicCalibrationMatrix());

  cv::Matx<double, 3, 3> trans;  // Matx is stackallocated (cv::Mat is not)
  cv::eigen2cv(H_inv, trans);

  cv::warpPerspective(
      camera_image, *patch.getImage(), trans, patch.getImage()->size(), cv::INTER_NEAREST);
  return patch;
}

ROI FeatureExtractor::calculateRoi(const cv::Mat &camera_image,
                                   const ROIParameters &roi_params) const {
  // compute image patch from roi
  //  const cv::Point center = (roi.br() + roi.tl()) / 2;
  const auto roi_rect = extractROIInImage(camera_image, roi_params);

  // construct roi
  const ImagePatch img_patch{roi_rect.tl(), camera_image(roi_rect).clone()};

  const VehiclePose pose =
      tf_helper_->getTransform().inverse() * roi_params.first.pose;

  return {img_patch, pose, computeROIBoundary(roi_params)};
}

cv::Rect FeatureExtractor::extractROIInImage(const cv::Mat &img,
                                             const ROIParameters &roi) const {
  // compute boudnfing points of roi
  const Eigen::Affine3d world_T_roi = roi.first.pose;
  const WorldPoint half_size{0.5 * roi.first.height, 0.5 * roi.first.width, 0.0};
  const WorldPoint first = world_T_roi * (-half_size);
  const WorldPoint second = world_T_roi * half_size;
  const WorldPoint third = world_T_roi * WorldPoint{half_size.x(), -half_size.y(), 0.0};
  const WorldPoint fourth = world_T_roi * WorldPoint{-half_size.x(), half_size.y(), 0.0};

  const WorldPoints bounding_points = {first, second, third, fourth};
  const Eigen::Affine3d vehicle_T_world = tf_helper_->getTransform().inverse();

  // transform to points to imagePoints and convert to cv_data
  const auto toImagePatchCV = [this](const auto &vp) {
    return toCV(camera_transform_->transformGroundToImage(vp));
  };
  const auto bounding_points_img =
      common::make_vector(bounding_points | common::eigen_transformed(vehicle_T_world) |
                          transformed(toImagePatchCV));

  // compute bounding rectangle
  const cv::Rect bounding_rect_raw = cv::boundingRect(bounding_points_img);

  const int half_width = roi.second.width / 2;
  const cv::Point tl = bounding_rect_raw.tl() - cv::Point{half_width, roi.second.height};
  const cv::Rect bounding_rect(tl.x,
                               tl.y,
                               bounding_rect_raw.width + roi.second.width,
                               bounding_rect_raw.height + roi.second.height);
  ROS_DEBUG("size of roi in image is width: %d; height: %d; br: (%d,%d)",
            bounding_rect.width,
            bounding_rect.height,
            bounding_rect.br().x,
            bounding_rect.br().y);

  // intersect with image in order to avoid points out if image limits
  return bounding_rect & cv::Rect(cv::Point(0, 0), img.size());
}

VehiclePoints FeatureExtractor::computeROIBoundary(const ROIParameters &roi) const {
  using ROIPoints = common::EigenAlignedVector<Eigen::Vector3d>;
  Eigen::Vector3d roi_x{0.5 * roi.first.height, 0, 0};
  Eigen::Vector3d roi_y{0, 0.5 * roi.first.width, 0};

  const ROIPoints boundary_P_roi{-roi_x - roi_y, -roi_x + roi_y, roi_x + roi_y, roi_x - roi_y};
  const Eigen::Affine3d vehicle_T_roi =
      tf_helper_->getTransform().inverse() * roi.first.pose;

  return common::make_eigen_vector(boundary_P_roi | common::eigen_transformed(vehicle_T_roi));
}

ImagePointExact FeatureExtractor::calculateCenter2d(const FeaturePointCluster &cluster) {
  return common::mean(cluster.feature_points_img | common::eigen_casted<double>);
}

VehiclePoint FeatureExtractor::calculateCenter3d(const FeaturePointCluster &cluster) {
  return common::mean(cluster.feature_points_vehicle);
}

VehiclePoint FeatureExtractor::calculateNearestPoint(const FeaturePointCluster &cluster) {
  return *boost::min_element(cluster.feature_points_vehicle, common::less_x());
}

VehiclePoint FeatureExtractor::calculateCenterFootPoint(const VehiclePoint &cluster_center,
                                                        const common::DynamicPolynomial &middle_lane_polynomial) {
  const double epsilon_newton_method = parameters_ptr_->getParam(EPSILON_NEWTON_METHOD);
  const int max_iterations_newton_method =
      parameters_ptr_->getParam(MAX_ITERATIONS_NEWTON_METHOD);

  return utils::findLotfusspunkt(
      middle_lane_polynomial, cluster_center, epsilon_newton_method, max_iterations_newton_method);
}

double FeatureExtractor::calculateCenterLaneOrientation(
    const VehiclePoint &cluster_center_middle_lane_foot_point,
    const common::DynamicPolynomial &middle_lane_polynomial) {
  return std::atan(middle_lane_polynomial.derivate().evaluate(
      cluster_center_middle_lane_foot_point(0)));
}

}  // namespace road_object_detection
