#include "look_at.h"

#include "../utils/step_detection.h"
#include "common/angle_conversions.h"
#include "common/eigen_functors.h"
#include "common/eigen_utils.h"
#include "common/move_range.h"
#include "common/normal_shift.h"
#include "common/polynomialfit.h"
#include "common/unique_erase.h"
#include "opencv_eigen_conversions.h"
#include "opencv_utils.h"

THIRD_PARTY_HEADERS_BEGIN
#include <opencv2/imgproc/imgproc.hpp>

#include <boost/algorithm/cxx11/all_of.hpp>
#include <boost/algorithm/cxx11/any_of.hpp>
#include <boost/range/adaptor/transformed.hpp>
#include <boost/range/algorithm_ext/erase.hpp>
#include <boost/range/algorithm_ext/push_back.hpp>
THIRD_PARTY_HEADERS_END

namespace look_at {

using boost::adaptors::transformed;

const ParameterString<int> LookAt::FIELD_OF_VISION_TOP("field_of_vision/top");
const ParameterString<int> LookAt::FIELD_OF_VISION_BOTTOM(
    "field_of_vision/bottom");
const ParameterString<int> LookAt::FIELD_OF_VISION_LEFT("field_of_vision/left");
const ParameterString<int> LookAt::FIELD_OF_VISION_RIGHT(
    "field_of_vision/right");
const ParameterString<double> LookAt::LANE_WIDTH("lane_width");
const ParameterString<double> LookAt::SIDE_TRIM("side_trim");
const ParameterString<double> LookAt::AREA_OF_INTEREST_STEP_SIZE(
    "area_of_interest_step_size");
const ParameterString<double> LookAt::GRADIENT_DETECTION_THLD(
    "gradient_detection_thld");
const ParameterString<double> LookAt::STEP_REFERENCE_FUNCTION_LENGTH(
    "step_reference_function_length");
const ParameterString<int> LookAt::DEGREE_OF_POLYNOMIAL("degree_of_polynomial");

LookAt::LookAt(ParameterInterface *const parameters,
               const common::CameraTransformation *const camera_transform,
               const tf_helper::TFHelperInterface<double> *const tf_helper,
               const EgoVehicle *const ego_vehicle)
    : parameters_ptr_(parameters),
      camera_transformation_(camera_transform),
      tf_helper_(tf_helper),
      ego_vehicle_(ego_vehicle),
      feature_extractor_(std::make_unique<road_object_detection::FeatureExtractor>(
          parameters, camera_transform, ego_vehicle, tf_helper)),
      clusterer_(parameters) {

  parameters->registerParam(FIELD_OF_VISION_TOP);
  parameters->registerParam(FIELD_OF_VISION_BOTTOM);
  parameters->registerParam(FIELD_OF_VISION_LEFT);
  parameters->registerParam(FIELD_OF_VISION_RIGHT);

  parameters->registerParam(SIDE_TRIM);
  parameters->registerParam(LANE_WIDTH);
  parameters->registerParam(AREA_OF_INTEREST_STEP_SIZE);

  parameters->registerParam(GRADIENT_DETECTION_THLD);
  parameters->registerParam(STEP_REFERENCE_FUNCTION_LENGTH);

  parameters->registerParam(DEGREE_OF_POLYNOMIAL);

  const int fov_top = parameters->getParam(FIELD_OF_VISION_TOP);
  const int fov_bottom = parameters->getParam(FIELD_OF_VISION_BOTTOM);
  const int fov_left = parameters->getParam(FIELD_OF_VISION_LEFT);
  const int fov_right = parameters->getParam(FIELD_OF_VISION_RIGHT);

  const common::EigenAlignedVector<ImagePoint> ground_in_image = {
      ImagePoint(fov_left, fov_top),
      ImagePoint(fov_right, fov_top),
      ImagePoint(fov_right, fov_bottom),
      ImagePoint(fov_left, fov_bottom)};

  common::EigenAlignedVector<VehiclePoint> fov_on_ground;
  camera_transformation_->transformImageToGround(ground_in_image, &fov_on_ground);
  field_of_vision = {{toCV(to2D(fov_on_ground[0])),
                      toCV(to2D(fov_on_ground[1])),
                      toCV(to2D(fov_on_ground[2])),
                      toCV(to2D(fov_on_ground[3]))}};

  assert(boost::algorithm::all_of(field_of_vision,
                                  [](const auto &a) { return a.x > 0; }) &&
         "false field of vision chosen. Check parameters!");
}

ClassificationResults LookAt::classifyROI(
    const cv::Mat &img,
    const ros::Time &timestamp,
    const RegionsToClassify &regions,
    const LineVehiclePoints &lane_points,
    const std::unique_ptr<road_object_detection::Classifier> &classifier) {
  ClassificationResults classifications;
  const auto middle_polynomial = createMiddlePolynomialByFusion(lane_points);
  if (!middle_polynomial) {
    ROS_WARN_THROTTLE(1, "No middle lane polynom available");
    return classifications;
  }

  const double step_size = parameters_ptr_->getParam(AREA_OF_INTEREST_STEP_SIZE);
  std::vector<FeaturePointCluster> clusters;
  classifications.reserve(regions.size());
  for (const auto &roi : regions) {
    // create scanlines
    ScanLines scan_lines = createScanLineGrid(roi, step_size);

    // extract feature_points
    ImagePoints feature_points = apply1dGradientDetector(img, scan_lines);
    boost::remove_erase_if(feature_points, [this](const Eigen::Vector2i &point) {
      return ego_vehicle_->contains(toCV(point));
    });

    // cluster points in roi
    FeaturePointCluster raw_cluster{*camera_transformation_, feature_points};
    boost::push_back(clusters, common::move_range(clusterFeaturePoints(raw_cluster)));

    // extract features and run classifier on them
    ClassificationResult result;
    for (const auto &cluster : clusters) {
      const auto cluster_features = feature_extractor_->extractFeatures(
          cluster,
          img,
          timestamp,
          middle_polynomial.get(),
          lane_points,
          boost::make_optional<road_object_detection::ROIParameters>(
              std::make_pair(roi, roi_offset_)));
      boost::push_back(result.detected_objects,
                       common::move_range(classifier->classify(cluster_features)));
    }
    result.id = roi.id;
    classifications.push_back(std::move(result));
  }

  return classifications;
}

boost::optional<common::DynamicPolynomial> LookAt::createMiddlePolynomialByFusion(
    const LineVehiclePoints &points) const {
  VehiclePoints middle_points = allPointsToMiddle(points);
  const int degree_of_polynomial = parameters_ptr_->getParam(DEGREE_OF_POLYNOMIAL);
  if (middle_points.size() <= static_cast<std::size_t>(degree_of_polynomial)) {
    return boost::none;
  }
  return common::fitToPoints(middle_points, degree_of_polynomial);
}

VehiclePoints LookAt::allPointsToMiddle(LineVehiclePoints points) const {
  // too close points are make the shift instable, so remove them
  const double min_point_distance = 0.05;
  common::unique_erase(points[LINESPEC_LEFT], common::areClose(min_point_distance));
  common::unique_erase(points[LINESPEC_RIGHT], common::areClose(min_point_distance));
  VehiclePoints fused_points = std::move(points[LINESPEC_MIDDLE]);
  //! TODO might be variable in extended Carolo Cup mode
  const double shift_by = parameters_ptr_->getParam(LANE_WIDTH);
  common::normalShift(points[LINESPEC_LEFT], -shift_by, &fused_points);
  common::normalShift(points[LINESPEC_RIGHT], shift_by, &fused_points);
  return fused_points;
}

ImagePoints LookAt::apply1dGradientDetector(const cv::Mat &img,
                                            const ScanLines &scan_lines) const {
  const auto threshold =
      static_cast<float>(parameters_ptr_->getParam(GRADIENT_DETECTION_THLD));

  const auto range_length = static_cast<unsigned int>(
      parameters_ptr_->getParam(STEP_REFERENCE_FUNCTION_LENGTH));
  const std::vector<float> reference_function =
      step_detection::createReferenceFunction(range_length);

  ImagePoints feature_points;
  for (const ScanLine &scan_line : scan_lines) {
    boost::push_back(feature_points,
                     step_detection::detectStep(
                         img, scan_line, reference_function, threshold, true));
  }

  return feature_points;
}

std::vector<FeaturePointCluster> LookAt::clusterFeaturePoints(const FeaturePointCluster &raw) const {
  return clusterer_.cluster(raw);
}

ScanLines LookAt::createScanLineGrid(const RegionToClassify &roi, const double step_size) const {
  using ROIPoint = Eigen::Vector3d;
  const Eigen::Affine3d vehicle_T_world = tf_helper_->getTransform().inverse();
  const Eigen::Affine3d roi_pose_vehicle = vehicle_T_world * roi.pose;

  const double side_trim = parameters_ptr_->getParam(SIDE_TRIM);
  const ROIPoint size{roi.height, roi.width - side_trim, 0.0};
  const ROIPoint start = -0.5 * size;
  const std::size_t steps =
      static_cast<std::size_t>(size.norm() / static_cast<double>(step_size));
  const VehiclePoint step_x(size.x() / static_cast<double>(steps), 0.0, 0.0);
  const VehiclePoint step_y(0.0, size.y() / static_cast<double>(steps), 0.0);
  const VehiclePoint up_shift(size.x(), 0.0, 0.0);
  const VehiclePoint side_shift(0.0, size.y(), 0.0);
  VehicleScanLines vehicle_lines;
  vehicle_lines.reserve(2 * steps);
  for (std::size_t i = 0; i < steps; i++) {
    const VehiclePoint current = start + i * step_x;
    vehicle_lines.push_back(roi_pose_vehicle * VehicleScanLine(current, current + side_shift));
  }

  for (std::size_t i = 0; i < steps; i++) {
    const VehiclePoint current = start + i * step_y;
    vehicle_lines.push_back(roi_pose_vehicle * VehicleScanLine(current, current + up_shift));
  }

  ScanLines scan_lines;

  for (auto &l : vehicle_lines) {

    //    if (!isInFrontOfCamera(l)) {
    clipVehicleScanLineFOV(l);
    //    }
    scan_lines.push_back(transformGroundToImage(camera_transformation_, l));
  }
  return scan_lines;
}

bool LookAt::isInFieldOfView(const cv::RotatedRect &rrect) const {
  std::array<cv::Point2f, 4> points;
  rrect.points(points.data());
  return boost::algorithm::any_of(points, [this](auto &p) {
    return cv::pointPolygonTest(toInputArray(field_of_vision), cv::Point(p), false) > 0;
  });
}

double LookAt::fieldOfViewNearX() const {
  return static_cast<double>(field_of_vision.back().x);
}

double LookAt::fieldOfViewFarX() const {
  return static_cast<double>(field_of_vision.front().x);
}

bool LookAt::isInFrontOfCamera(const VehiclePoint &p) const {
  return p.x() > fieldOfViewNearX();
}

bool LookAt::isInFrontOfCamera(const VehicleScanLine &l) const {
  return isInFrontOfCamera(l.start) && isInFrontOfCamera(l.end);
}

bool LookAt::isinFrontOfCamera(const ScanLine &l) const {
  return isInFrontOfCamera(
      VehicleScanLine(camera_transformation_->transformImageToGround(l.start),
                      camera_transformation_->transformImageToGround(l.end)));
}


void LookAt::clipVehicleScanLineFOV(VehicleScanLine &line) const {
  const VehiclePoint start_to_end = line.end - line.start;

  // if not is in field of vision, trim until it is
  const auto signed_dist_start = cv::pointPolygonTest(
      toInputArray(field_of_vision), imagePointExactToCvPoint2f(to2D(line.start)), true);
  if (signed_dist_start <= 0) {
    line.start = line.start + std::fabs(signed_dist_start) * start_to_end.normalized();
  }

  const VehiclePoint end_to_start = line.start - line.end;

  const auto signed_dist_end = cv::pointPolygonTest(
      toInputArray(field_of_vision), imagePointExactToCvPoint2f(to2D(line.end)), true);
  if (signed_dist_end <= 0) {
    line.end = line.end + std::fabs(signed_dist_end) * end_to_start.normalized();
  }
}
}  // namespace look_at
