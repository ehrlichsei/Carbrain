#include "road_watcher.h"
#include <common/macros.h>

THIRD_PARTY_HEADERS_BEGIN
#include <geometry_msgs/TransformStamped.h>
#include <ros/console.h>
#include <tf2_eigen/tf2_eigen.h>
#include <boost/algorithm/cxx11/all_of.hpp>
#include <boost/algorithm/cxx11/any_of.hpp>
#include <boost/range/adaptor/filtered.hpp>
#include <boost/range/adaptor/transformed.hpp>
#include <boost/range/algorithm/max_element.hpp>
#include <boost/range/algorithm/partition.hpp>
#include <boost/range/algorithm_ext/erase.hpp>
#include <boost/range/algorithm_ext/push_back.hpp>
#include <boost/range/join.hpp>
#include <cmath>
#include <limits>
THIRD_PARTY_HEADERS_END

#include "../utils/foot_finder.h"
#include "common/adaptors.h"
#include "common/angle_conversions.h"
#include "common/basic_statistics_eigen.h"
#include "common/best_score.h"
#include "common/console_colors.h"
#include "common/eigen_functors.h"
#include "common/eigen_utils.h"
#include "common/make_vector.h"
#include "common/minmax_element.h"
#include "common/normal_shift.h"
#include "common/polynomialfit.h"
#include "common/unique_erase.h"
#include "opencv_eigen_conversions.h"
#include "opencv_utils.h"

const ParameterString<double> GRADIENT_DETECTION_THLD(
    "gradient_detection_thld");
const ParameterString<int> STEP_REFERENCE_FUNCTION_LENGTH(
    "step_reference_function_length");
const ParameterString<double> LANE_WIDTH("lane_width");
const ParameterString<double> LANE_TOLERANCE("lane_tolerance");
const ParameterString<double> SAMPLING_X_START("sampling_x_start");
const ParameterString<double> SAMPLING_X_END("sampling_x_end");
const ParameterString<double> SAMPLING_X_END_OFFSET("sampling_x_end_offset");
const ParameterString<double> SAMPLING_X_STEP("sampling_x_step");
const ParameterString<int> SAMPLING_OFFSET("sampling_offset");
const ParameterString<double> SAMPLING_OFFSET_RANGE("sampling_offset_range");
const ParameterString<double> MAX_CURVATURE_GUESS("max_curvature_guess");

const ParameterString<double> MIN_SAMPLING_DISTANCE("min_sampling_distance");
const ParameterString<double> MAX_SAMPLING_DISTANCE("max_sampling_distance");
const ParameterString<bool> DYNAMICALLY_RESTRICT_SEARCH_PATTERN(
    "dynamically_restrict_search_pattern");
const ParameterString<int> DEGREE_OF_POLYNOMIAL("degree_of_polynomial");
const ParameterString<double> TRIM_FACTOR("trim_factor");

const ParameterString<int> FIELD_OF_VISION_TOP("field_of_vision/top");
const ParameterString<int> FIELD_OF_VISION_BOTTOM("field_of_vision/bottom");
const ParameterString<int> FIELD_OF_VISION_LEFT("field_of_vision/left");
const ParameterString<int> FIELD_OF_VISION_RIGHT("field_of_vision/right");

// const ParameterString<double>
//    AREA_OF_INTEREST_STEP_SIZE("area_of_interest_step_size");
// const ParameterString<double> REDUCED_LANE_WIDTH("reduced_lane_width");
const ParameterString<int> MIN_SIZE_CLUSTERS("min_cluster_size");
const ParameterString<double> BIN_LENGTH_CLUSTERING("bin_length_clustering");

const ParameterString<double> MIN_LANE_WIDTH("min_lane_width");
const ParameterString<double> MAX_LANE_WIDTH("max_lane_width");
const ParameterString<int> MIN_SIZE_POINTS_WIDTH_ESTIMATION(
    "min_size_points_width_estimation");

RoadWatcher::RoadWatcher(ParameterInterface *parameters,
                         const common::CameraTransformation *const camera_transformation,
                         const EgoVehicle *const ego_vehicle)
    : cam_transform_(camera_transformation),
      parameters_ptr_(parameters),
      ego_vehicle(ego_vehicle),
      clusterer(parameters) {
  parameters->registerParam(GRADIENT_DETECTION_THLD);
  parameters->registerParam(STEP_REFERENCE_FUNCTION_LENGTH);
  parameters->registerParam(LANE_WIDTH);
  parameters->registerParam(LANE_TOLERANCE);
  parameters->registerParam(SAMPLING_X_START);
  parameters->registerParam(SAMPLING_X_END);
  parameters->registerParam(SAMPLING_X_STEP);
  parameters->registerParam(SAMPLING_OFFSET);
  parameters->registerParam(SAMPLING_X_END_OFFSET);
  parameters->registerParam(SAMPLING_OFFSET_RANGE);
  parameters->registerParam(MAX_CURVATURE_GUESS);

  parameters->registerParam(MIN_SAMPLING_DISTANCE);
  parameters->registerParam(MAX_SAMPLING_DISTANCE);

  parameters->registerParam(DYNAMICALLY_RESTRICT_SEARCH_PATTERN);
  parameters->registerParam(DEGREE_OF_POLYNOMIAL);
  parameters->registerParam(TRIM_FACTOR);

  parameters->registerParam(FIELD_OF_VISION_TOP);
  parameters->registerParam(FIELD_OF_VISION_BOTTOM);
  parameters->registerParam(FIELD_OF_VISION_LEFT);
  parameters->registerParam(FIELD_OF_VISION_RIGHT);

  //  parameters->registerParam(AREA_OF_INTEREST_STEP_SIZE);
  //  parameters->registerParam(REDUCED_LANE_WIDTH);
  parameters->registerParam(MIN_LANE_WIDTH);
  parameters->registerParam(MAX_LANE_WIDTH);
  parameters->registerParam(MIN_SIZE_POINTS_WIDTH_ESTIMATION);

  parameters->registerParam(MIN_SIZE_CLUSTERS);
  parameters->registerParam(BIN_LENGTH_CLUSTERING);

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
  cam_transform_->transformImageToGround(ground_in_image, &fov_on_ground);
  field_of_vision = {{toCV(to2D(fov_on_ground[0])),
                      toCV(to2D(fov_on_ground[1])),
                      toCV(to2D(fov_on_ground[2])),
                      toCV(to2D(fov_on_ground[3]))}};

  assert(boost::algorithm::all_of(field_of_vision,
                                  [](const auto &a) { return a.x > 0; }) &&
         "false field of vision chosen. Check parameters!");
}

void RoadWatcher::scanRoadAndFindClusters(const cv::Mat &img_gray,
                                          const LineVehiclePoints &points,
                                          std::vector<FeaturePointCluster> &clusters,
                                          common::DynamicPolynomial &middle_polynomial) const {

  const auto polynomial = createMiddlePolynomialByFusion(points);
  if (!polynomial) {
    return;
  }
  middle_polynomial = polynomial.get();

  ScanLines scan_lines = createScanLineGrid(points, middle_polynomial);

  common::Vector2iVector feature_points = apply1dGradientDetector(img_gray, scan_lines);
  boost::remove_erase_if(feature_points, [this](const Eigen::Vector2i &point) {
    return ego_vehicle->contains(toCV(point));
  });

  if (feature_points.empty()) {
    return;
  }

  FeaturePointCluster unassignedFeaturePoints(*cam_transform_, feature_points);

  //  verifyFeaturePointsDoNotLieOnLaneMarking(unassignedFeaturePoints_);
  const auto clusters_raw = clusterer.cluster(unassignedFeaturePoints);

  for (const auto &c : clusters_raw) {
    boost::push_back(clusters, clusterWithMiddlePolynom(c.feature_points_vehicle, middle_polynomial));
  }
}

VehiclePoints RoadWatcher::allPointToMiddle(LineVehiclePoints points) const {
  // too close points are make the shift instable, so remove them
  const double min_point_distance = 0.05;
  common::unique_erase(points[LINESPEC_LEFT], common::areClose(min_point_distance));
  common::unique_erase(points[LINESPEC_RIGHT], common::areClose(min_point_distance));
  VehiclePoints fused_points = std::move(points[LINESPEC_MIDDLE]);

  const double shift_by = parameters_ptr_->getParam(LANE_WIDTH);
  // only fuse points, if plausible
  if (arePointsPlausible(points[LINESPEC_LEFT], points[LINESPEC_MIDDLE])) {
    common::normalShift(points[LINESPEC_LEFT], -shift_by, &fused_points);
  }
  if (arePointsPlausible(points[LINESPEC_RIGHT], points[LINESPEC_MIDDLE])) {
    common::normalShift(points[LINESPEC_RIGHT], shift_by, &fused_points);
  }
  //  common::normalShift(points[LINESPEC_LEFT], -shift_by, &fused_points);
  //  common::normalShift(points[LINESPEC_RIGHT], shift_by, &fused_points);
  return fused_points;
}

boost::optional<common::DynamicPolynomial> RoadWatcher::createMiddlePolynomialByFusion(
    const LineVehiclePoints &points) const {
  VehiclePoints middle_points = allPointToMiddle(points);
  const int degree_of_polynomial = parameters_ptr_->getParam(DEGREE_OF_POLYNOMIAL);
  if (middle_points.size() <= static_cast<std::size_t>(degree_of_polynomial)) {
    return boost::none;
  }
  return common::fitToPoints(middle_points, degree_of_polynomial);
}

inline bool isBetween(const double t, const double low, const double hi) {
  return t > low && t <= hi;
}

inline auto isBetween(const double x1, const double x2) {
  return [x1, x2](const auto &p) { return isBetween(p.second, x1, x2); };
}

std::vector<FeaturePointCluster> RoadWatcher::clusterWithMiddlePolynom(
    const VehiclePoints &unassigend_feature_points,
    const common::DynamicPolynomial &middle_lane_polynom) const {
  const auto points = common::members(&std::pair<VehiclePoint, double>::first);

  const auto bin_length = parameters_ptr_->getParam(BIN_LENGTH_CLUSTERING);

  // projection onto lane polynomial
  const auto projectOntoPoly = [&middle_lane_polynom](const auto &p) {
    return std::make_pair(p, utils::findLotfusspunktX(middle_lane_polynom, to2D(p)));
  };
  auto points_and_projections = common::make_vector(
      unassigend_feature_points | boost::adaptors::transformed(projectOntoPoly));

  const auto min_max_x = common::minmax_score(
      points_and_projections, [](const auto &p) { return p.second; });

  // reserve bins for clustering
  const auto nr_bins = static_cast<std::size_t>(
      std::ceil((min_max_x.second->second - min_max_x.first->second) / bin_length));

  std::vector<FeaturePointCluster> refined_clusters;
  const auto start = min_max_x.first->second;

  bool is_bin_empty = true;
  VehiclePoints actual_cluster_points = {min_max_x.first->first};

  ROS_DEBUG("refine_clusters: min_x is %f; max_x is %f; nr_bins is %zu",
            min_max_x.first->second,
            min_max_x.second->second,
            nr_bins);

  for (std::size_t i = 0; i < nr_bins; i++) {

    const auto div = boost::partition(
        points_and_projections,
        isBetween(start + i * bin_length, start + (i + 1) * bin_length));
    is_bin_empty = (div == points_and_projections.begin());
    if (is_bin_empty && !actual_cluster_points.empty()) {
      // if actual bin is empty: push back collected feature points into the
      // refined clusterö
      refined_clusters.emplace_back(*cam_transform_, actual_cluster_points);
      actual_cluster_points.clear();
    } else if (!is_bin_empty) {
      // if last bin wasn't empty: collect feature points and erase the
      // associated points in the source container
      boost::push_back(
          actual_cluster_points,
          boost::make_iterator_range(points_and_projections.begin(), div) | points);
      boost::erase(points_and_projections,
                   boost::make_iterator_range(points_and_projections.begin(), div));
    }
  }

  if (!is_bin_empty) {
    //    actual_cluster_points.push_back(min_max_x.second->first);
    refined_clusters.emplace_back(*cam_transform_, actual_cluster_points);
  }

  // the minimum points required to form a valid crosswalk cluster should be
  // enough for one lane
  const auto min_cluster_size =
      static_cast<std::size_t>(parameters_ptr_->getParam(MIN_SIZE_CLUSTERS));
  boost::remove_erase_if(refined_clusters, [&min_cluster_size](const auto &c) {
    return c.feature_points_vehicle.size() < min_cluster_size;
  });

  return refined_clusters;
}

bool RoadWatcher::isInFieldOfView(const cv::Point2f &point) const {
  return cv::pointPolygonTest(toInputArray(field_of_vision), point, false) > 0;
}

double max_x_value(const VehiclePoints &points) {
  if (points.empty()) {
    return std::numeric_limits<double>::min();
  }

  return boost::max_element(points, common::less_x())->x();
}

common::DiscretizationParams RoadWatcher::initSamplingParameters(
    const LineVehiclePoints &points, const common::DynamicPolynomial &middle_polynomial) const {
  const double sampling_x_start =
      std::max(fieldOfViewNearX(), parameters_ptr_->getParam(SAMPLING_X_START));
  const double sampling_x_step = parameters_ptr_->getParam(SAMPLING_X_STEP);
  //  double sampling_x_end_offset =
  //  parameters_ptr_->getParam(SAMPLING_X_END_OFFSET);
  double sampling_x_end;
  if (parameters_ptr_->getParam(DYNAMICALLY_RESTRICT_SEARCH_PATTERN)) {
    double min_sampling_distance = parameters_ptr_->getParam(MIN_SAMPLING_DISTANCE);

    // Distance between sampling_x_start and sampling_x_end must be at least >=
    // min_sampling_distance.
    sampling_x_end = sampling_x_start + min_sampling_distance;
    // Distance of sampling x_end is limited

    // Check if there are lane detection points whose x value are bigger than
    // the current value of sampling_x_end.
    sampling_x_end = std::max(sampling_x_end + 0.02, max_x_value(points[LINESPEC_LEFT]));
    sampling_x_end =
        std::max(sampling_x_end + 0.02, max_x_value(points[LINESPEC_MIDDLE]));
    sampling_x_end =
        std::max(sampling_x_end + 0.02, max_x_value(points[LINESPEC_RIGHT]));
    sampling_x_end =
        std::min(sampling_x_end + 0.02,
                 sampling_x_start + parameters_ptr_->getParam(MAX_SAMPLING_DISTANCE));
    sampling_x_end = std::min(sampling_x_end + 0.02, fieldOfViewFarX());
  } else {
    // We do not dynamically restrict the search pattern so we just use the
    // value provided in the parameter file.
    sampling_x_end = parameters_ptr_->getParam(SAMPLING_X_END);
  }
  const double curvature_lookahead =
      sampling_x_end - parameters_ptr_->getParam(SAMPLING_X_END_OFFSET);
  auto curvature = std::fabs(middle_polynomial.calculateCurvature(curvature_lookahead));
  const double max_curvature = parameters_ptr_->getParam(MAX_CURVATURE_GUESS);
  curvature = std::min(curvature, max_curvature);
  ROS_DEBUG("road_watcher: curvature at lookahead is: %f", curvature);
  const double offset_range = parameters_ptr_->getParam(SAMPLING_OFFSET_RANGE);
  const double sampling_x_end_offset =
      offset_range * std::pow(1 - curvature / max_curvature, 2);
  ROS_DEBUG("road_watcher: curvature-adaptive offset is: %f", sampling_x_end_offset);
  sampling_x_end += sampling_x_end_offset;
  sampling_x_end = std::min(sampling_x_end, fieldOfViewFarX());

  return {sampling_x_start, sampling_x_end, sampling_x_step};
}

inline VehiclePoint minXPoint(const VehiclePoints &points) {
  return *common::minmax_score(points, [](const auto &p) { return p.x(); }).first;
}

std::vector<double> RoadWatcher::estimateLaneWidth(const common::DynamicPolynomial &lane_polynom,
                                                   const common::DiscretizationParams &params,
                                                   const VehiclePoints &outer_lane_points) const {

  if (outer_lane_points.size() < 2) {
    return {};
  }

  const auto poly_deg = parameters_ptr_->getParam(DEGREE_OF_POLYNOMIAL);

  const auto min_max_x = common::minmax_element(outer_lane_points | common::x_values);

  ROS_DEBUG("estimation: min x is %f, max x is %f", *min_max_x.first, *min_max_x.second);
  const auto outer_polynom = common::fitToPoints(outer_lane_points, poly_deg);

  const auto estimate = [&outer_polynom](const auto &p) {
    return (p - utils::findLotfusspunkt(outer_polynom, p)).norm();
  };

  const auto between = [&min_max_x, &outer_polynom](const auto &p) {
    return isBetween(utils::findLotfusspunktX(outer_polynom, to2D(p)),
                     *min_max_x.first,
                     *min_max_x.second);
  };

  VehiclePoints lane_poly_points;
  common::discretisize(lane_polynom, params, &lane_poly_points);

  ROS_DEBUG("estimation: size of lane poly points is %zu", lane_poly_points.size());

  return common::make_vector(lane_poly_points | boost::adaptors::filtered(between) |
                             boost::adaptors::transformed(estimate));
}

bool RoadWatcher::isLeftLinePlausibel(const std::vector<double> &left_distances) const {
  const auto min_size_points = static_cast<std::size_t>(
      parameters_ptr_->getParam(MIN_SIZE_POINTS_WIDTH_ESTIMATION));
  if (left_distances.size() < min_size_points) {
    return false;
  }

  const auto min_lane_width = parameters_ptr_->getParam(MIN_LANE_WIDTH);
  const auto max_lane_width = parameters_ptr_->getParam(MAX_LANE_WIDTH);

  const auto mean_distance = common::mean(left_distances);

  ROS_DEBUG("mean_distance of left lane is %f", mean_distance);

  return mean_distance < max_lane_width && mean_distance > min_lane_width;
}

bool RoadWatcher::arePointsPlausible(const VehiclePoints &side_points,
                                     const VehiclePoints &middle_points) const {
  // discretize
  const auto deg = parameters_ptr_->getParam(DEGREE_OF_POLYNOMIAL);

  // if not enough middle points, fuse
  if (middle_points.size() <= static_cast<std::size_t>(deg + 1)) {
    return true;
  }

  // if not enough side points, don't fuse
  if (side_points.size() <= static_cast<std::size_t>(deg + 1)) {
    return false;
  }

  const auto side_polynomial = common::fitToPoints(side_points, deg);
  const auto mid_polynomial = common::fitToPoints(middle_points, deg);

  const auto sampling_step = parameters_ptr_->getParam(SAMPLING_X_STEP);
  const auto minmax = common::minmax_element(side_points | common::x_values);
  const common::DiscretizationParams params = {
      *minmax.first, std::max(*minmax.second, *minmax.first + sampling_step), sampling_step};

  const auto width = estimateLaneWidth(mid_polynomial, params, side_points);

  const auto max_lane_width = parameters_ptr_->getParam(MAX_LANE_WIDTH);
  const auto min_lane_width = parameters_ptr_->getParam(MIN_LANE_WIDTH);

  const auto between = [&min_lane_width, &max_lane_width](const auto &v) {
    return v > min_lane_width && v < max_lane_width;
  };

  return boost::algorithm::all_of(width, between);
}

double RoadWatcher::fieldOfViewNearX() const {
  return static_cast<double>(field_of_vision.back().x);
}

double RoadWatcher::fieldOfViewFarX() const {
  return static_cast<double>(field_of_vision.front().x);
}

ScanLines RoadWatcher::createScanLineGrid(const LineVehiclePoints &points,
                                          const common::DynamicPolynomial &middle_polynomial) const {
  const common::DiscretizationParams discretization_params =
      initSamplingParameters(points, middle_polynomial);

  ScanLines scan_lines =
      points[LINESPEC_NO_PASSING].empty()
          ? createScanLineGridLane(discretization_params, middle_polynomial, points)
          : createScanLineGridNoPassingZone(discretization_params, middle_polynomial, points);

  return scan_lines;
}

ScanLines RoadWatcher::createScanLineGridNoPassingZone(
    const common::DiscretizationParams &discretization_params,
    const common::DynamicPolynomial &middle_polynomial,
    const LineVehiclePoints &lane_detection_points) const {

  const unsigned int offset =
      static_cast<unsigned int>(parameters_ptr_->getParam(SAMPLING_OFFSET));
  const double lane_width = parameters_ptr_->getParam(LANE_WIDTH);
  const double trim_factor = parameters_ptr_->getParam(TRIM_FACTOR);

  // calculate sample points, i.e. scan line endpoints

  const auto poly_degree = parameters_ptr_->getParam(DEGREE_OF_POLYNOMIAL);
  VehiclePoints middle_samples, right_samples;

  auto shift = -0.02;

  if (lane_detection_points[LINESPEC_MIDDLE].size() > 1 &&
      lane_detection_points[LINESPEC_MIDDLE].size() > static_cast<std::size_t>(poly_degree)) {
    const auto differenceToMiddleLanePolynom = [&middle_polynomial](const auto &p) {
      return VehiclePoint(p - utils::findLotfusspunkt(middle_polynomial, p));
    };

    const auto middle_right =
        common::mean(lane_detection_points[LINESPEC_MIDDLE]).y() <
                common::mean(lane_detection_points[LINESPEC_NO_PASSING]).y()
            ? lane_detection_points[LINESPEC_MIDDLE]
            : lane_detection_points[LINESPEC_NO_PASSING];

    const auto mean_difference = common::mean(
        middle_right | boost::adaptors::transformed(differenceToMiddleLanePolynom));
    shift = std::copysign(mean_difference.norm(), mean_difference.y());
  }

  const auto plusShift = [&shift](const auto &d) { return d + shift; };

  const auto distances =
      lane_detection_points[LINESPEC_RIGHT].size() <= static_cast<std::size_t>(poly_degree)
          ? std::vector<double>(middle_samples.size(), -lane_width)
          : common::make_vector(estimateLaneWidth(middle_polynomial,
                                                  discretization_params,
                                                  lane_detection_points[LINESPEC_RIGHT]) |
                                boost::adaptors::transformed(plusShift));

  common::normalShift(middle_polynomial, shift, discretization_params, &middle_samples);

  const auto wide_shift = distances.size() > 1 ? -common::mean(distances) : -lane_width;
  common::normalShift(middle_polynomial, wide_shift, discretization_params, &right_samples);

  ImagePoints middle_samples_image, right_samples_image;
  cam_transform_->transformVehicleToImage(middle_samples, &middle_samples_image);
  cam_transform_->transformVehicleToImage(right_samples, &right_samples_image);

  ScanLines scan_lines;
  scan_lines.reserve(middle_samples.size() * 2);

  for (size_t i = offset; i < middle_samples.size(); i++) {
    // bottom-right to top-middle
    scan_lines.push_back(
        ScanLine(middle_samples_image[i], right_samples_image[i - offset]).trim(trim_factor));

    // top-right to bottom-middle
    scan_lines.push_back(
        ScanLine(middle_samples_image[i - offset], right_samples_image[i]).trim(trim_factor));
  }

  return scan_lines;
}

ScanLines RoadWatcher::createScanLineGridPedestrianIsland(
    const common::DynamicPolynomial &middle_polynomial,
    const std::vector<double> &distances_right,
    const common::DiscretizationParams &params,
    const VehiclePoints &middle_samples) const {
  // FIXME make parameters
  const unsigned int offset =
      static_cast<unsigned int>(parameters_ptr_->getParam(SAMPLING_OFFSET));
  const double lane_width = parameters_ptr_->getParam(LANE_WIDTH);
  const double trim_factor = parameters_ptr_->getParam(TRIM_FACTOR);
  const auto min_points_required = static_cast<std::size_t>(
      parameters_ptr_->getParam(MIN_SIZE_POINTS_WIDTH_ESTIMATION));

  const double estimated_width = distances_right.size() > min_points_required
                                     ? common::mean(distances_right)
                                     : lane_width;

  VehiclePoints right_samples;
  common::normalShift(middle_polynomial, -estimated_width, params, &right_samples);

  ImagePoints middle_samples_image, right_samples_image;
  cam_transform_->transformVehicleToImage(middle_samples, &middle_samples_image);
  cam_transform_->transformVehicleToImage(right_samples, &right_samples_image);

  ScanLines scan_lines;
  scan_lines.reserve(middle_samples.size() * 2);

  for (size_t i = offset; i < middle_samples.size(); i++) {

    // bottom-right to top-middle
    scan_lines.push_back(
        ScanLine(middle_samples_image[i], right_samples_image[i - offset]).trim(trim_factor));

    // top-right to bottom-middle
    scan_lines.push_back(
        ScanLine(middle_samples_image[i - offset], right_samples_image[i]).trim(trim_factor));
  }

  return scan_lines;
}

ScanLines RoadWatcher::createScanLineGridLane(const common::DiscretizationParams &discretization_params,
                                              const common::DynamicPolynomial &middle_polynomial,
                                              const LineVehiclePoints &lane_detection_points) const {

  const unsigned int offset =
      static_cast<unsigned int>(parameters_ptr_->getParam(SAMPLING_OFFSET));
  const double lane_width = parameters_ptr_->getParam(LANE_WIDTH);
  const double trim_factor = parameters_ptr_->getParam(TRIM_FACTOR);
  // calculate sample points, i.e. scan line endpoints

  VehiclePoints middle_samples, left_samples, right_samples;
  common::discretisize(middle_polynomial, discretization_params, &middle_samples);

  const auto distances_left = estimateLaneWidth(
      middle_polynomial, discretization_params, lane_detection_points[LINESPEC_LEFT]);

  const auto distances_right = estimateLaneWidth(
      middle_polynomial, discretization_params, lane_detection_points[LINESPEC_RIGHT]);

  if (!isLeftLinePlausibel(distances_left)) {
    return createScanLineGridPedestrianIsland(
        middle_polynomial, distances_right, discretization_params, middle_samples);
  }

  const std::vector<double> aggregated =
      common::make_vector(common::join(distances_left, distances_right));
  const auto min_points_required = static_cast<std::size_t>(
      parameters_ptr_->getParam(MIN_SIZE_POINTS_WIDTH_ESTIMATION));
  const double estimated_width = distances_right.size() > min_points_required
                                     ? common::mean(distances_right)
                                     : lane_width;

  ROS_DEBUG("estimated width is %f", estimated_width);

  common::normalShift(middle_polynomial, estimated_width, discretization_params, &left_samples);
  common::normalShift(middle_polynomial, -estimated_width, discretization_params, &right_samples);

  ImagePoints middle_samples_image, left_samples_image, right_samples_image;
  cam_transform_->transformVehicleToImage(middle_samples, &middle_samples_image);
  cam_transform_->transformVehicleToImage(left_samples, &left_samples_image);
  cam_transform_->transformVehicleToImage(right_samples, &right_samples_image);

  ScanLines scan_lines;
  scan_lines.reserve(middle_samples.size() * 4);

  for (size_t i = offset; i < middle_samples.size(); i++) {
    // bottom-left to top-middle
    scan_lines.push_back(
        ScanLine(left_samples_image[i - offset], middle_samples_image[i]).trim(trim_factor));
    //    appendToScanlines(left_samples_image[i - offset],
    //    middle_samples_image[i],
    //                      trim_factor, scan_lines);

    // top-left to bottom-middle
    scan_lines.push_back(
        ScanLine(left_samples_image[i], middle_samples_image[i - offset]).trim(trim_factor));
    //    appendToScanlines(left_samples_image[i], middle_samples_image[i -
    //    offset],
    //                      trim_factor, scan_lines);

    // bottom-right to top-middle
    scan_lines.push_back(
        ScanLine(middle_samples_image[i], right_samples_image[i - offset]).trim(trim_factor));
    //    appendToScanlines(middle_samples_image[i], right_samples_image[i -
    //    offset],
    //                      trim_factor, scan_lines);

    // top-right to bottom-middle
    scan_lines.push_back(
        ScanLine(middle_samples_image[i - offset], right_samples_image[i]).trim(trim_factor));
    //    appendToScanlines(middle_samples_image[i - offset],
    //    right_samples_image[i],
    //                      trim_factor, scan_lines);
  }

  return scan_lines;
}

ImagePoints RoadWatcher::apply1dGradientDetector(const cv::Mat &img,
                                                 const ScanLines &scan_lines) const {
  const float threshold =
      static_cast<float>(parameters_ptr_->getParam(GRADIENT_DETECTION_THLD));

  const unsigned int range_length = static_cast<unsigned int>(
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
