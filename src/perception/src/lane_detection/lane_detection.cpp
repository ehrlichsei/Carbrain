#include "lane_detection.h"

THIRD_PARTY_HEADERS_BEGIN
#include <ros/console.h>
#include <boost/range/algorithm_ext/erase.hpp>
#include <boost/range/algorithm_ext/push_back.hpp>
#include <boost/range/algorithm/count_if.hpp>
#include <boost/range/adaptor/filtered.hpp>
#include <boost/range/adaptor/transformed.hpp>
#include <boost/algorithm/cxx11/any_of.hpp>
#include <cmath>
#include <ceres/ceres.h>
#include <algorithm>
#include <numeric>
THIRD_PARTY_HEADERS_END

#include "common/math.h"
#include "common/container.h"
#include "common/unique_erase.h"
#include "common/best_score.h"
#include "common/console_colors.h"
#include "common/pca_eigen.h"
#include "common/eigen_utils.h"
#include "common/eigen_functors.h"
#include "common/polynomialfit.h"
#include "common/normal_shift.h"
#include "common/polynomial.h"
#include "common/polynomial_utils.h"
#include "common/basic_statistics.h"
#include "common/make_vector.h"
#include "common/minmax_element.h"

#include "../utils/foot_finder.h"

#include "feature_extraction.h"
#include "../utils/linear_clustering.h"

const ParameterString<int> LaneDetection::FITTING_IMAGE_POINTS_TRESHOLD(
    "line_tracking/fitting/image_points_threshold");
const ParameterString<double> LaneDetection::FITTING_FITTING_ERROR_THRESHOLD(
    "line_tracking/fitting/fitting_error_threshold");
const ParameterString<int> LaneDetection::POLYNOMIAL_DEGREE(
    "line_tracking/polynomial/degree");
const ParameterString<int> LaneDetection::POLYNOMIAL_SWITCHDOWN(
    "line_tracking/polynomial/switchdown");
const ParameterString<double> LaneDetection::LANE_SEPARATION(
    "line_tracking/lane/sep");
const ParameterString<double> LaneDetection::LANE_WIDTH_DIFF(
    "line_tracking/lane/width_diff");
const ParameterString<double> LaneDetection::POLYNOMIAL_EQUALITY_EPS(
    "line_tracking/eps");
const ParameterString<bool> LaneDetection::DEGREE_ADAPTION_ENABLE(
    "line_tracking/degree_adaption/enable");
const ParameterString<double> LaneDetection::DEGREE_ADAPTION_MAX_A(
    "line_tracking/degree_adaption/max_a");
const ParameterString<double> LaneDetection::DEGREE_ADAPTION_MAX_ERROR(
    "line_tracking/degree_adaption/max_error");
const ParameterString<bool> LaneDetection::MIDLINE_RECOGNITION_ENABLED(
    "line_tracking/midline_recognition/enable");
const ParameterString<double> LaneDetection::MIDLINE_RECOGNITION_CLUSTERING_DISTANCE_THRESH(
    "line_tracking/midline_recognition/clustering_distance_thresh");
const ParameterString<double> LaneDetection::MIDLINE_RECOGNITION_INTRA_CLUSTER_DISTANCE_THRESH(
    "line_tracking/midline_recognition/intra_cluster_distance_thresh");
const ParameterString<double> LaneDetection::MIDLINE_RECOGNITION_REGION_OF_INTEREST(
    "line_tracking/midline_recognition/region_of_interest");
const ParameterString<int> LaneDetection::MIDLINE_RECOGNITION_MIN_NUMBER_OF_CLUSTERS(
    "line_tracking/midline_recognition/min_number_of_clusters");
const ParameterString<int> LaneDetection::MIDLINE_RECOGNITION_MIN_NUMBER_OF_POINTS_IN_CLUSTER(
    "line_tracking/midline_recognition/min_number_of_points_in_cluster");
const ParameterString<double> LaneDetection::START_BOX_HANDLING_MAX_A(
    "line_tracking/start_box_handling/max_A");
const ParameterString<double> LaneDetection::START_BOX_HANDLING_MAX_B(
    "line_tracking/start_box_handling/max_B");
const ParameterString<double> LaneDetection::START_BOX_HANDLING_MAX_ERROR(
    "line_tracking/start_box_handling/max_error");
const ParameterString<bool> LaneDetection::START_BOX_HANDLING_STARTS_WITH(
    "line_tracking/start_box_handling/starts_with");
const ParameterString<bool> LaneDetection::USE_ALWAYS_INIT(
    "line_tracking/use_alway_init");
const ParameterString<double> LaneDetection::VEHICLE_POINT_EQUALS_EPS(
    "line_tracking/vehicle_point_equals_eps");
const ParameterString<int> LaneDetection::NAV_POINTS_TRESHOLD(
    "line_tracking/nav_points_threshold");
const ParameterString<std::vector<double>> LaneDetection::CONSISTENCY_CHECK_SAMPLING_POINTS(
    "line_tracking/consistency_check_sampling_points");
const ParameterString<bool> LaneDetection::POINT_FILTERING_ENABLED(
    "line_tracking/points_filtering_enabled");
const ParameterString<double> LaneDetection::RELATIVE_CLUSTER_THRESHOLD(
    "line_tracking/double_middle_lane/cluster_threshold_relative");
const ParameterString<double> LaneDetection::CLUSTER_THRESHOLD_LEFT_RIGHT(
    "line_tracking/double_middle_lane/left_right_threshold");
const ParameterString<int> LaneDetection::CLUSTER_MIN_SIZE(
    "line_tracking/double_middle_lane/cluster_min_size");
const ParameterString<double> LaneDetection::OUTLINE_LOCALIZATION_THRESHOLD(
    "line_tracking/double_middle_lane/outline_localization_threshold");
const ParameterString<double> LaneDetection::DISCRETIZATION_STEP(
    "line_tracking/double_middle_lane/discretization_step");
const ParameterString<double> LaneDetection::MINIMAL_CLUSTER_LENGTH(
    "line_tracking/double_middle_lane/minimal_cluster_length");
const ParameterString<double> LaneDetection::POLYNOMIAL_MIN_X(
    "line_tracking/polynomial/min_x");
const ParameterString<double> LaneDetection::POLYNOMIAL_MAX_X(
    "line_tracking/polynomial/max_x");
const ParameterString<double> LaneDetection::POLYNOMIAL_STEP(
    "line_tracking/polynomial/step_x");
const ParameterString<double> LaneDetection::POLYNOMIAL_CAUCHY_LOSS(
    "line_tracking/polynomial/cauchy_loss");


inline void LaneDetection::registerLaneDetectionParameter(ParameterInterface* parameter_interface) {
  parameter_interface->registerParam(FITTING_IMAGE_POINTS_TRESHOLD);
  parameter_interface->registerParam(FITTING_FITTING_ERROR_THRESHOLD);
  parameter_interface->registerParam(POLYNOMIAL_DEGREE);
  parameter_interface->registerParam(POLYNOMIAL_SWITCHDOWN);
  parameter_interface->registerParam(LANE_SEPARATION);
  parameter_interface->registerParam(LANE_WIDTH_DIFF);
  parameter_interface->registerParam(POLYNOMIAL_EQUALITY_EPS);
  parameter_interface->registerParam(DEGREE_ADAPTION_ENABLE);
  parameter_interface->registerParam(DEGREE_ADAPTION_MAX_A);
  parameter_interface->registerParam(DEGREE_ADAPTION_MAX_ERROR);
  parameter_interface->registerParam(MIDLINE_RECOGNITION_ENABLED);
  parameter_interface->registerParam(MIDLINE_RECOGNITION_CLUSTERING_DISTANCE_THRESH);
  parameter_interface->registerParam(MIDLINE_RECOGNITION_INTRA_CLUSTER_DISTANCE_THRESH);
  parameter_interface->registerParam(MIDLINE_RECOGNITION_REGION_OF_INTEREST);
  parameter_interface->registerParam(MIDLINE_RECOGNITION_MIN_NUMBER_OF_CLUSTERS);
  parameter_interface->registerParam(MIDLINE_RECOGNITION_MIN_NUMBER_OF_POINTS_IN_CLUSTER);
  parameter_interface->registerParam(START_BOX_HANDLING_MAX_A);
  parameter_interface->registerParam(START_BOX_HANDLING_MAX_B);
  parameter_interface->registerParam(START_BOX_HANDLING_MAX_ERROR);
  parameter_interface->registerParam(START_BOX_HANDLING_STARTS_WITH);
  parameter_interface->registerParam(USE_ALWAYS_INIT);
  parameter_interface->registerParam(VEHICLE_POINT_EQUALS_EPS);
  parameter_interface->registerParam(NAV_POINTS_TRESHOLD);
  parameter_interface->registerParam(CONSISTENCY_CHECK_SAMPLING_POINTS);
  parameter_interface->registerParam(POINT_FILTERING_ENABLED);
  parameter_interface->registerParam(RELATIVE_CLUSTER_THRESHOLD);
  parameter_interface->registerParam(CLUSTER_THRESHOLD_LEFT_RIGHT);
  parameter_interface->registerParam(CLUSTER_MIN_SIZE);
  parameter_interface->registerParam(OUTLINE_LOCALIZATION_THRESHOLD);
  parameter_interface->registerParam(DISCRETIZATION_STEP);
  parameter_interface->registerParam(MINIMAL_CLUSTER_LENGTH);
  parameter_interface->registerParam(POLYNOMIAL_MIN_X);
  parameter_interface->registerParam(POLYNOMIAL_MAX_X);
  parameter_interface->registerParam(POLYNOMIAL_STEP);
  parameter_interface->registerParam(POLYNOMIAL_CAUCHY_LOSS);
}

LaneDetection::LaneDetection(ParameterInterface* parameters, FeatureExtraction* featureExtraction)
    : feature_extraction(featureExtraction),
      use_start_box_mode_(false),
      parameters_ptr_(parameters) {
  registerLaneDetectionParameter(parameters);
  setInitFlag();
  line_degree[LINESPEC_LEFT] = parameters_ptr_->getParam(POLYNOMIAL_DEGREE);
  line_degree[LINESPEC_MIDDLE] = line_degree[LINESPEC_LEFT];
  line_degree[LINESPEC_RIGHT] = line_degree[LINESPEC_LEFT];

  vehicle_point_filter[LINESPEC_LEFT] = std::make_shared<VehiclePointFilter>(
      parameters, "line_tracking/outer_lines_point_filtering");
  vehicle_point_filter[LINESPEC_MIDDLE] = std::make_shared<VehiclePointFilter>(
      parameters, "line_tracking/middle_line_point_filtering");
  vehicle_point_filter[LINESPEC_RIGHT] = vehicle_point_filter[LINESPEC_LEFT];
}

void LaneDetection::setInitFlag() { init = true; }

void LaneDetection::setStartBox(const bool use_start_box_mode) {
  if (use_start_box_mode != use_start_box_mode_) {
    use_start_box_mode_ = use_start_box_mode;
    ROS_INFO(use_start_box_mode ? "Lane_Detection entered Startbox mode."
                                : "Lane_Detection left Startbox mode.");
  }

  if (use_start_box_mode_) {
    setInitFlag();
    line_degree[LINESPEC_LEFT] = parameters_ptr_->getParam(POLYNOMIAL_DEGREE) -
                                 parameters_ptr_->getParam(POLYNOMIAL_SWITCHDOWN);
    line_degree[LINESPEC_MIDDLE] = line_degree[LINESPEC_LEFT];
    line_degree[LINESPEC_RIGHT] = line_degree[LINESPEC_LEFT];
  }
}

void LaneDetection::startModule() {
  setStartBox(parameters_ptr_->getParam(START_BOX_HANDLING_STARTS_WITH));
}

LineVehiclePoints LaneDetection::extractFeatures(const cv::Mat& image,
                                                 const LaneModel& lane_model) const {
  return empty(lane_model) ? feature_extraction->extractLineMarkings(image)
                           : feature_extraction->extractLineMarkings(image, lane_model);
}

LineVehiclePoints LaneDetection::processImage(const cv::Mat& image) {
  // Setting Upt Variables
  init |= parameters_ptr_->getParam(USE_ALWAYS_INIT);
  discretization_params = {parameters_ptr_->getParam(POLYNOMIAL_MIN_X),
                           parameters_ptr_->getParam(POLYNOMIAL_MAX_X),
                           parameters_ptr_->getParam(POLYNOMIAL_STEP)};

  if (init) {
    lane_model_[LINESPEC_LEFT] = boost::none;
    lane_model_[LINESPEC_MIDDLE] = boost::none;
    lane_model_[LINESPEC_RIGHT] = boost::none;
  }

  ROS_DEBUG_STREAM(COLOR_NORMAL << "First Run." << COLOR_DEBUG);
  LineVehiclePoints out_line_data;
  const bool needs_second_run = extractLines(image, false, &out_line_data);

  if (!needs_second_run) {
    ROS_DEBUG_STREAM(COLOR_NORMAL << "CHECK DOUBLE MIDDLE LINES" << COLOR_DEBUG);
    checkDoubleMiddleLines(image, &out_line_data);
    return out_line_data;
  }

  const bool first_run_was_init = init;
  init = false;
  clear(&out_line_data);
  ROS_DEBUG_STREAM(COLOR_NORMAL << "Second Run." << COLOR_DEBUG);
  const bool needs_third_run = extractLines(image, !first_run_was_init, &out_line_data);

  if (!first_run_was_init || !needs_third_run) {
    checkDoubleMiddleLines(image, &out_line_data);
    return out_line_data;
  }

  init = false;
  clear(&out_line_data);
  ROS_DEBUG_STREAM(COLOR_NORMAL << "Third Run." << COLOR_DEBUG);
  extractLines(image, true, &out_line_data);

  checkDoubleMiddleLines(image, &out_line_data);
  return out_line_data;
}

void LaneDetection::setMiddleLine(const VehiclePoints& points) {
  fitPolynom(LINESPEC_MIDDLE, points);

  const double LANE_SEP = parameters_ptr_->getParam(LANE_SEPARATION);

  const std::array<bool, LINESPEC_N> line_existed = {{false, true, false}};
  addVirtualLine(LINESPEC_LEFT, LINESPEC_MIDDLE, LINESPEC_RIGHT, LANE_SEP, 0, line_existed);
  addVirtualLine(LINESPEC_RIGHT, LINESPEC_MIDDLE, LINESPEC_LEFT, -LANE_SEP, 0, line_existed);
}

bool LaneDetection::extractLines(const cv::Mat& image,
                                 const bool second_run,
                                 LineVehiclePoints* out_line_data) {
  bool needs_second_run = init;
  LineVehiclePoints vehicle_points = extractFeatures(image, lane_model_);

  // Filter points if desired
  if (parameters_ptr_->getParam(POINT_FILTERING_ENABLED)) {
    filterVehiclePoints(
        vehicle_points[LINESPEC_LEFT], LINESPEC_LEFT, &(*out_line_data)[LINESPEC_LEFT]);
    filterVehiclePoints(
        vehicle_points[LINESPEC_RIGHT], LINESPEC_RIGHT, &(*out_line_data)[LINESPEC_RIGHT]);
    filterVehiclePoints(vehicle_points[LINESPEC_MIDDLE],
                        LINESPEC_MIDDLE,
                        &(*out_line_data)[LINESPEC_MIDDLE]);
  }

  // Perform polynomial fit
  fitPolynom(LINESPEC_LEFT, (*out_line_data)[LINESPEC_LEFT]);
  fitPolynom(LINESPEC_RIGHT, (*out_line_data)[LINESPEC_RIGHT]);
  fitPolynom(LINESPEC_MIDDLE, (*out_line_data)[LINESPEC_MIDDLE]);

  // Handle Start box if desired
  if (use_start_box_mode_) {
    handleStartBox(out_line_data);
  }

  // Perform consistency checks
  checkConsistency(out_line_data);

  const double LANE_SEP = parameters_ptr_->getParam(LANE_SEPARATION);
  if (number(lane_model_) == 1) {
    if (!second_run) {
      ROS_DEBUG_STREAM(COLOR_YELLOW
                       << "only single line, waiting for second run." << COLOR_DEBUG);
      needs_second_run = true;
    } else if (lane_model_[LINESPEC_LEFT]) {
      ROS_DEBUG_STREAM(COLOR_YELLOW << "only left line" << COLOR_DEBUG);
      tryCorrectingSingleLine(image, LINESPEC_LEFT, LANE_SEP, out_line_data);
    } else if (lane_model_[LINESPEC_RIGHT]) {
      ROS_DEBUG_STREAM(COLOR_YELLOW << "only right line" << COLOR_DEBUG);
      tryCorrectingSingleLine(image, LINESPEC_RIGHT, -LANE_SEP, out_line_data);
    } else {
      ROS_DEBUG_STREAM(COLOR_YELLOW
                       << "only middle line detected, kinda strange...." << COLOR_DEBUG);
    }
  }

  // Add virtual lines if lines are dropped but at least one line is still
  // there.
  const std::array<bool, LINESPEC_N> line_existed = {
      {lane_model_[LINESPEC_LEFT].is_initialized(),
       lane_model_[LINESPEC_MIDDLE].is_initialized(),
       lane_model_[LINESPEC_RIGHT].is_initialized()}};

  addVirtualLine(LINESPEC_LEFT, LINESPEC_MIDDLE, LINESPEC_RIGHT, LANE_SEP, 2 * LANE_SEP, line_existed);
  addVirtualLine(LINESPEC_MIDDLE, LINESPEC_RIGHT, LINESPEC_LEFT, LANE_SEP, -LANE_SEP, line_existed);
  addVirtualLine(LINESPEC_RIGHT, LINESPEC_MIDDLE, LINESPEC_LEFT, -LANE_SEP, -2 * LANE_SEP, line_existed);

  deleteLineWithFewPoints(LINESPEC_LEFT, out_line_data);
  deleteLineWithFewPoints(LINESPEC_MIDDLE, out_line_data);
  deleteLineWithFewPoints(LINESPEC_RIGHT, out_line_data);

  common::sortAlongPrincipalComponent(&(out_line_data->at(LINESPEC_LEFT)));
  common::sortAlongPrincipalComponent(&(out_line_data->at(LINESPEC_MIDDLE)));
  common::sortAlongPrincipalComponent(&(out_line_data->at(LINESPEC_RIGHT)));

  return needs_second_run;
}

void LaneDetection::tryCorrectingSingleLine(const cv::Mat& image,
                                            const LineSpec linespec,
                                            const double shift,
                                            LineVehiclePoints* out_line_data) {
  VehiclePoints proj_points;
  common::normalShift(*lane_model_[linespec], shift, discretization_params, &proj_points);
  const auto line =
      fitToPoints(proj_points, parameters_ptr_->getParam(POLYNOMIAL_DEGREE));

  const auto extracted_points = feature_extraction->extractLinePointsByLine(image, line);
  VehiclePoints filtered_vehicle_points;
  filterVehiclePoints(extracted_points, LINESPEC_MIDDLE, &filtered_vehicle_points);

  if (filtered_vehicle_points.size() >
          static_cast<std::size_t>(parameters_ptr_->getParam(NAV_POINTS_TRESHOLD)) &&
      mightBeMiddleLine(filtered_vehicle_points, true)) {
    ROS_DEBUG_STREAM(COLOR_YELLOW
                     << "Found middle line at the left right of right line." << COLOR_DEBUG);
    fitPolynom(LINESPEC_MIDDLE, filtered_vehicle_points);
    (*out_line_data)[LINESPEC_MIDDLE] = std::move(filtered_vehicle_points);
    (*out_line_data)[other(linespec)] = std::move((*out_line_data)[linespec]);
    lane_model_[other(linespec)] = std::move(lane_model_[linespec]);
    eraseCompletly(linespec, out_line_data);
  }
}

inline void LaneDetection::addVirtualLine(const LineSpec target,
                                          const LineSpec source1,
                                          const LineSpec source2,
                                          const double shift1,
                                          const double shift2,
                                          const std::array<bool, LINESPEC_N>& line_existed) {
  if (line_existed.at(target)) {
    ROS_DEBUG("Polynom %s exists.", toString(target).c_str());
    return;
  }

  ROS_DEBUG("Polynom %s does not exist; needs to be added virtually.",
            toString(target).c_str());

  VehiclePoints proj_points;
  if (line_existed.at(source1)) {
    common::normalShift(*lane_model_[source1], shift1, discretization_params, &proj_points);
  }

  if (line_existed.at(source2)) {
    common::normalShift(*lane_model_[source2], shift2, discretization_params, &proj_points);
  }

  if (proj_points.size() >
      std::size_t(parameters_ptr_->getParam(FITTING_IMAGE_POINTS_TRESHOLD))) {
    lane_model_[target] =
        fitToPoints(proj_points, parameters_ptr_->getParam(POLYNOMIAL_DEGREE));
  }

  showProjectionPoints(target, proj_points);
}

struct ResidualFunctor {
  ResidualFunctor(const common::polynomial::PolynomialDegree degree, double x, double y)
      : degree_(degree), x_(x), y_(y) {}

  template <typename T>
  bool operator()(const T a[LaneDetection::max_poly_degree_], T residual[1]) const {
    residual[0] = static_cast<T>(y_);

    for (std::size_t i = 0; i <= static_cast<std::size_t>(degree_); ++i) {
      residual[0] -= a[i] * pow(static_cast<T>(x_), i);
    }

    return true;
  }

 private:
  const common::polynomial::PolynomialDegree degree_;
  const double x_;
  const double y_;
};

inline void LaneDetection::fitPolynom(const LineSpec lineSpec, const VehiclePoints& line_data) {
  lane_model_[lineSpec] = boost::none;
  if (line_data.size() <=
      static_cast<std::size_t>(parameters_ptr_->getParam(FITTING_IMAGE_POINTS_TRESHOLD))) {
    ROS_DEBUG("Not enough points at %s!", toString(lineSpec).c_str());
    return;
  }

  ROS_DEBUG("Using %zu points for fit at %s", line_data.size(), toString(lineSpec).c_str());

  auto line = fitToPoints(line_data, line_degree[lineSpec]);

  double error = common::computeAbsoluteFittingError(line_data, line);
  if (parameters_ptr_->getParam(DEGREE_ADAPTION_ENABLE) && !use_start_box_mode_) {
    adaptPolyDegree(lineSpec, &line, line_data, &error);
  }

  if (error > parameters_ptr_->getParam(FITTING_FITTING_ERROR_THRESHOLD)) {
    ROS_DEBUG("No polynom at %s because of to big error %f.", toString(lineSpec).c_str(), error);
    return;
  }

  lane_model_[lineSpec] = std::move(line);
}

inline common::polynomial::DynamicPolynomial LaneDetection::fitToPoints(
    const VehiclePoints& line_data, const common::polynomial::PolynomialDegree degree) const {
  const auto cauchy_loss = parameters_ptr_->getParam(POLYNOMIAL_CAUCHY_LOSS);
  constexpr auto max_degree = LaneDetection::max_poly_degree_;
  // std::vector<double> coeffs(max_degree, 0.0);
  std::array<double, max_degree> coeffs = {};

  ceres::Problem problem;

  for (const auto& point : line_data) {
    ceres::CostFunction* cost_function =
        new ceres::AutoDiffCostFunction<ResidualFunctor, 1, max_degree>(
            new ResidualFunctor(degree, point.x(), point.y()));

    ceres::LossFunction* loss_function = new ceres::CauchyLoss(cauchy_loss);

    problem.AddResidualBlock(cost_function, loss_function, &(coeffs[0]));
  }

  ceres::Solver::Options options;
  options.max_num_iterations = 10;
  options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
  options.minimizer_progress_to_stdout = false;

  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);

  return common::DynamicPolynomial(
      coeffs.begin(), coeffs.begin() + static_cast<int>(degree) + 1);
}

void LaneDetection::adaptPolyDegree(const LineSpec lineSpec,
                                    common::DynamicPolynomial* line,
                                    const VehiclePoints& points,
                                    double* error) {
  const int DEGREE = parameters_ptr_->getParam(POLYNOMIAL_DEGREE);
  const int SWITCHDOWN = parameters_ptr_->getParam(POLYNOMIAL_SWITCHDOWN);
  if (lineSpec != LINESPEC_MIDDLE) {
    const auto reduction_error =
        line->calculateReductionError(common::PolynomialDegrees::Linear);

    if (line_degree[lineSpec] == DEGREE &&
        reduction_error < parameters_ptr_->getParam(DEGREE_ADAPTION_MAX_A)) {
      switchDegree(-SWITCHDOWN, &line_degree[lineSpec], line, points, error);
    } else if (line_degree[lineSpec] == DEGREE - SWITCHDOWN &&
               *error > parameters_ptr_->getParam(DEGREE_ADAPTION_MAX_ERROR)) {
      switchDegree(0, &line_degree[lineSpec], line, points, error);
    } else {
      ROS_DEBUG("NO_SWITCH: error: %f a: %f", *error, reduction_error);
    }
    return;
  }

  if (line_degree[LINESPEC_RIGHT] == DEGREE - SWITCHDOWN &&
      line_degree[LINESPEC_LEFT] == DEGREE - SWITCHDOWN &&
      line_degree[LINESPEC_MIDDLE] == DEGREE) {
    switchDegree(-SWITCHDOWN, &line_degree[lineSpec], line, points, error);
  } else if (line_degree[LINESPEC_MIDDLE] == DEGREE - SWITCHDOWN &&
             (line_degree[LINESPEC_RIGHT] == DEGREE || line_degree[LINESPEC_LEFT] == DEGREE)) {
    switchDegree(0, &line_degree[lineSpec], line, points, error);
  }
}

void LaneDetection::switchDegree(const int sw,
                                 int* line_degree,
                                 common::DynamicPolynomial* line,
                                 const VehiclePoints& points,
                                 double* error) {
  const int new_degree = parameters_ptr_->getParam(POLYNOMIAL_DEGREE) + sw;
  const auto newLine = fitToPoints(points, new_degree);
  const double newError = common::computeAbsoluteFittingError(points, newLine);

  if (newError > parameters_ptr_->getParam(DEGREE_ADAPTION_MAX_ERROR)) {
    ROS_DEBUG("No switch because of too big fitting error at refit.");
    return;
  }

  *error = newError;
  *line = newLine;
  *line_degree = new_degree;
  ROS_DEBUG("Switched with refit to %d", *line_degree);
}

bool LaneDetection::inPedestrianIsland(const LineVehiclePoints& out_line_data) const {
  const auto max_width_diff = parameters_ptr_->getParam(LANE_WIDTH_DIFF);
  const auto lane_width_left = estimateLaneWidth(LINESPEC_LEFT, out_line_data);
  const auto lane_width_right = estimateLaneWidth(LINESPEC_RIGHT, out_line_data);

  ROS_DEBUG_STREAM("width left: " << lane_width_left << " width right: " << lane_width_right);
  ROS_DEBUG_STREAM("in island: " << (lane_width_left > lane_width_right + max_width_diff));

  if (lane_width_left > 0.0 && lane_width_right > 0.0) {
    return (lane_width_left > lane_width_right + max_width_diff);
  } else {
    return false;
  }
}

double LaneDetection::estimateLaneWidth(const LineSpec& outer,
                                        const LineVehiclePoints& out_line_data) const {
  const auto& outer_lane_points = out_line_data[outer];

  if (outer_lane_points.size() < 2 || !lane_model_[LINESPEC_MIDDLE].is_initialized()) {
    return 0.0;
  }

  const auto& lane_polynom = *lane_model_[LINESPEC_MIDDLE];

  const auto poly_deg = static_cast<int>(lane_polynom.getCoefficients().size() - 1);

  const auto min_max_x = common::minmax_element(outer_lane_points | common::x_values);

  ROS_DEBUG("estimation: min x is %f, max x is %f", *min_max_x.first, *min_max_x.second);
  const auto outer_polynom = fitToPoints(outer_lane_points, poly_deg);

  const auto estimate = [&outer_polynom](const auto& p) {
    return (p - utils::findLotfusspunkt(outer_polynom, p)).norm();
  };

  const auto isBetween = [](const double t, const double low, const double hi) {
    return t > low && t <= hi;
  };

  const auto between = [&min_max_x, &outer_polynom, &isBetween](const auto& p) {
    return isBetween(utils::findLotfusspunktX(outer_polynom, to2D(p)),
                     *min_max_x.first,
                     *min_max_x.second);
  };

  VehiclePoints lane_poly_points;
  common::discretisize(lane_polynom, discretization_params, &lane_poly_points);

  ROS_DEBUG("estimation: size of lane poly points is %zu", lane_poly_points.size());

  auto points = common::make_vector(lane_poly_points | boost::adaptors::filtered(between) |
                                    boost::adaptors::transformed(estimate));

  if (points.size() > 10) {
    std::partial_sort(points.begin(), points.begin() + 10, points.end());
    return common::mean(points.begin(), points.begin() + 10);
  } else {
    return 0.0;
  }
}

inline void LaneDetection::checkConsistency(LineVehiclePoints* out_line_data) {
  checkPairWise(LINESPEC_LEFT, LINESPEC_RIGHT, out_line_data);
  checkPairWise(LINESPEC_LEFT, LINESPEC_MIDDLE, out_line_data);
  checkPairWise(LINESPEC_MIDDLE, LINESPEC_RIGHT, out_line_data);

  int curvature_left = 0;
  int curvature_middle = 0;
  int curvature_right = 0;

  const double x0 = 0.0;  // might replaced by parameter

  // Curvature check
  if (lane_model_[LINESPEC_LEFT]) {
    curvature_left = common::sgn(lane_model_[LINESPEC_LEFT]->calculateCurvature(x0));
    ROS_DEBUG("l: %f", lane_model_[LINESPEC_LEFT]->calculateCurvature(x0));
  }

  if (lane_model_[LINESPEC_MIDDLE]) {
    curvature_middle = common::sgn(lane_model_[LINESPEC_MIDDLE]->calculateCurvature(x0));
    ROS_DEBUG("m: %f", lane_model_[LINESPEC_MIDDLE]->calculateCurvature(x0));
  }

  if (lane_model_[LINESPEC_RIGHT]) {
    curvature_right = common::sgn(lane_model_[LINESPEC_RIGHT]->calculateCurvature(x0));
    ROS_DEBUG("r: %f", lane_model_[LINESPEC_RIGHT]->calculateCurvature(x0));
  }

  ROS_DEBUG_STREAM("l: " << curvature_left << " m: " << curvature_middle
                         << " r: " << curvature_right);

  if (curvature_left != curvature_right && curvature_right != 0 &&
      curvature_left != 0 && curvature_middle != curvature_right) {
    eraseCompletly(smallerLine(*out_line_data, LINESPEC_LEFT, LINESPEC_RIGHT), out_line_data);
  }

  // if (lane_model_[LINESPEC_MIDDLE]) {
  //   const bool is_linear = isLinear(*lane_model_[LINESPEC_MIDDLE]);

  //   if (is_linear) {
  //     const auto tangent =
  //         common::polynomial::tangentAngle(*lane_model_[LINESPEC_MIDDLE],
  //         1.0);
  //     const auto tangentTooHigh = std::abs(tangent) > M_PI / 3.0;

  //     if (tangentTooHigh) {
  //       eraseCompletly(LINESPEC_MIDDLE, out_line_data);
  //     }
  //   }
  // }

  // is the Middle line the middle line and if not, is there any other middle
  // line
  if (parameters_ptr_->getParam(MIDLINE_RECOGNITION_ENABLED) &&
      !inPedestrianIsland(*out_line_data))
  // if (parameters_ptr_->getParam(MIDLINE_RECOGNITION_ENABLED))
  {
    const bool left_data_exists =
        lane_model_[LINESPEC_LEFT] && !out_line_data->at(LINESPEC_LEFT).empty();
    const bool middle_data_exists = lane_model_[LINESPEC_MIDDLE] &&
                                    !out_line_data->at(LINESPEC_MIDDLE).empty();
    const bool right_data_exists =
        lane_model_[LINESPEC_RIGHT] && !out_line_data->at(LINESPEC_RIGHT).empty();

    bool left_data_middle = false;
    bool middle_data_middle = false;
    bool right_data_middle = false;

    if (left_data_exists) {
      ROS_DEBUG("check left line if middle line");
      left_data_middle = mightBeMiddleLine((*out_line_data)[LINESPEC_LEFT], true);
    }
    if (middle_data_exists) {
      ROS_DEBUG("check middle line if middle line");
      middle_data_middle = mightBeMiddleLine((*out_line_data)[LINESPEC_MIDDLE], false);
    }
    if (right_data_exists) {
      ROS_DEBUG("check right line if middle line");
      right_data_middle = mightBeMiddleLine((*out_line_data)[LINESPEC_RIGHT], true);
    }

    if (right_data_exists && right_data_middle) {
      ROS_DEBUG("Right line classified as middle line!");
    }
    if (left_data_exists && left_data_middle) {
      ROS_DEBUG("Left line classified as middle line!");
    }

    if (/*middle_data_exists && */ !middle_data_middle) {
      ROS_DEBUG("Middle classified as non-middle line");
      if (left_data_exists && left_data_middle && !right_data_middle) {
        moveToMiddle(LINESPEC_LEFT, out_line_data);
      } else if (right_data_exists && right_data_middle && !left_data_middle) {
        moveToMiddle(LINESPEC_RIGHT, out_line_data);
      } else {
        const auto lane_sep = parameters_ptr_->getParam(LANE_SEPARATION) +
                              parameters_ptr_->getParam(LANE_WIDTH_DIFF);
        const auto lane_width_left = estimateLaneWidth(LINESPEC_LEFT, *out_line_data);
        const auto lane_width_right = estimateLaneWidth(LINESPEC_RIGHT, *out_line_data);

        if (lane_width_left > lane_sep || lane_width_right > lane_sep) {
          eraseCompletly(LINESPEC_MIDDLE, out_line_data);
        }
      }
    }
  }

  const std::array<bool, LINESPEC_N> line_exists = {
      {lane_model_[LINESPEC_LEFT].is_initialized(),
       lane_model_[LINESPEC_MIDDLE].is_initialized(),
       lane_model_[LINESPEC_RIGHT].is_initialized()}};

  if (line_exists[LINESPEC_LEFT] && !line_exists[LINESPEC_MIDDLE] &&
      !line_exists[LINESPEC_RIGHT]) {
    if (lane_model_[LINESPEC_LEFT]->getCoefficients()[0] < 0.0) {
      eraseCompletly(LINESPEC_LEFT, out_line_data);
    }
  }

  if (!line_exists[LINESPEC_LEFT] && !line_exists[LINESPEC_MIDDLE] &&
      line_exists[LINESPEC_RIGHT]) {
    if (lane_model_[LINESPEC_RIGHT]->getCoefficients()[0] > 0.0) {
      eraseCompletly(LINESPEC_RIGHT, out_line_data);
    }
  }
}

void LaneDetection::checkPairWise(const LineSpec& lefter,
                                  const LineSpec& righter,
                                  LineVehiclePoints* out_line_data) {
  if (!lane_model_[righter] || !lane_model_[lefter]) {
    return;
  }

  const double eps = parameters_ptr_->getParam(POLYNOMIAL_EQUALITY_EPS);
  const std::vector<double> sampling_points =
      parameters_ptr_->getParam(CONSISTENCY_CHECK_SAMPLING_POINTS);
  std::vector<double> cdiffs;
  cdiffs.reserve(sampling_points.size());

  bool are_consistent = true;
  for (double sampling_point : sampling_points) {
    double cdiff = lane_model_[righter]->evaluate(sampling_point) -
                   lane_model_[lefter]->evaluate(sampling_point);
    are_consistent &= (cdiff < 0.0 && std::abs(cdiff) > eps);
    cdiffs.push_back(cdiff);
  }
  // Right line is left of middle line or together
  if (are_consistent) {
    return;
  }

  using namespace common;  // toString for containers
  const auto smaller_line = smallerLine(*out_line_data, lefter, righter);
  const auto other = (smaller_line == lefter) ? righter : lefter;
  eraseCompletly(smaller_line, out_line_data);
  ROS_DEBUG_STREAM("Erased " << toString(smaller_line)
                             << " line (collision with " << toString(other)
                             << ") cdiffs: " << toString(cdiffs));
}

void LaneDetection::moveToMiddle(const LineSpec line_spec, LineVehiclePoints* out_line_data) {
  ROS_WARN("Move %s to new middle line", toString(line_spec).c_str());
  (*out_line_data)[other(line_spec)] = std::move((*out_line_data)[LINESPEC_MIDDLE]);
  (*out_line_data)[LINESPEC_MIDDLE] = std::move((*out_line_data)[line_spec]);
  lane_model_[other(line_spec)] = std::move(lane_model_[LINESPEC_MIDDLE]);
  lane_model_[LINESPEC_MIDDLE] = std::move(lane_model_[line_spec]);
  eraseCompletly(line_spec, out_line_data);
}

void LaneDetection::eraseCompletly(const LineSpec line_sec, LineVehiclePoints* out_line_data) {
  out_line_data->at(line_sec).clear();
  lane_model_[line_sec] = boost::none;
}

void LaneDetection::handleStartBox(LineVehiclePoints* out_line_data) {
  // Start Box mode is over if every line is straight
  setStartBox(!checkLineLinear(LINESPEC_LEFT, out_line_data) ||
              !checkLineLinear(LINESPEC_MIDDLE, out_line_data) ||
              !checkLineLinear(LINESPEC_RIGHT, out_line_data));
}

bool LaneDetection::checkLineLinear(const LineSpec lineSpec, LineVehiclePoints* out_line_data) {
  if (!lane_model_[lineSpec]) {
    return false;
  }

  const auto& polynomial = lane_model_[lineSpec].get();
  auto& points = out_line_data->at(lineSpec);

  if (!isLinear(polynomial)) {
    ROS_DEBUG("StartBox Mode: %s lane was erased because it was not linear",
              toString(lineSpec).c_str());
    eraseCompletly(lineSpec, out_line_data);
    return false;
  }

  // Remove points with big error to get a nearly staright line
  const double MAX_ERROR = parameters_ptr_->getParam(START_BOX_HANDLING_MAX_ERROR);
  const std::size_t IMAGE_POINT_TRHESH =
      static_cast<size_t>(parameters_ptr_->getParam(FITTING_IMAGE_POINTS_TRESHOLD));
  boost::remove_erase_if(
      points,
      [&](const auto& p) { return common::absError(p, polynomial) > MAX_ERROR; });

  if (points.size() < IMAGE_POINT_TRHESH) {
    eraseCompletly(lineSpec, out_line_data);
  }

  return true;
}

bool LaneDetection::isLinear(const common::DynamicPolynomial& line) const {
  const auto& coeffs = line.getCoefficients();
  const double a = (coeffs.size() > 2) ? coeffs[2] : 0.0;
  const double b = (coeffs.size() > 1) ? coeffs[1] : 0.0;
  return (std::abs(a) < parameters_ptr_->getParam(START_BOX_HANDLING_MAX_A)) &&
         (std::abs(b) < parameters_ptr_->getParam(START_BOX_HANDLING_MAX_B));
}

void inline LaneDetection::filterVehiclePoints(VehiclePoints in_points,
                                               const LineSpec linespec,
                                               VehiclePoints* out_line_points) {
  ROS_DEBUG("Point_filtering %s", toString(linespec).c_str());
  ROS_DEBUG("All points: %zu", in_points.size());
  deleteDoublePoints(&in_points);
  vehicle_point_filter.at(linespec)->filterPoints(in_points, out_line_points);
}

bool LaneDetection::mightBeMiddleLine(const VehiclePoints& points, const bool strict) const {
  const double clustering_dist_thresh =
      parameters_ptr_->getParam(MIDLINE_RECOGNITION_CLUSTERING_DISTANCE_THRESH);
  const double intra_cluster_dist_tresh =
      parameters_ptr_->getParam(MIDLINE_RECOGNITION_INTRA_CLUSTER_DISTANCE_THRESH);
  const double roi = parameters_ptr_->getParam(MIDLINE_RECOGNITION_REGION_OF_INTEREST);

  const std::size_t min_number_of_clusters = static_cast<std::size_t>(
      parameters_ptr_->getParam(MIDLINE_RECOGNITION_MIN_NUMBER_OF_CLUSTERS));
  const std::size_t min_number_of_points_in_cluster = static_cast<std::size_t>(
      parameters_ptr_->getParam(MIDLINE_RECOGNITION_MIN_NUMBER_OF_POINTS_IN_CLUSTER));

  std::vector<VehiclePoints> clusters;
  EuclideanLinearClustering<VehiclePoint>(clustering_dist_thresh).cluster(points, &clusters);

  if (strict && clusters.size() <= min_number_of_clusters) {
    ROS_DEBUG_STREAM(COLOR_RED << "no midline. clusters size: " << clusters.size()
                               << COLOR_DEBUG);
    return false;
  }

  ROS_DEBUG_STREAM(COLOR_RED << "clusters size: " << clusters.size() << COLOR_DEBUG);

  bool one_cluster_in_roi_with_enough_points = false;
  for (const VehiclePoints& current_cluster : clusters) {
    if (current_cluster.size() > min_number_of_points_in_cluster) {
      const bool is_in_roi = current_cluster.front()(0) < roi;
      if (is_in_roi) {
        ROS_DEBUG_STREAM(
            COLOR_RED << "one cluster with enough points pro cluster in roi: "
                      << current_cluster.size() << COLOR_DEBUG);
        one_cluster_in_roi_with_enough_points = true;
      }
      if (is_in_roi || strict) {
        const double in_dist = (current_cluster.front() - current_cluster.back()).norm();
        if (in_dist > intra_cluster_dist_tresh) {
          ROS_DEBUG_STREAM(COLOR_RED << "no midline. in_dist: " << in_dist << COLOR_DEBUG);
          return false;
        }
      }
    } else if (strict && current_cluster.front()(0) < roi) {
      ROS_DEBUG_STREAM(COLOR_RED
                       << "one cluster with to view points pro cluster in roi: "
                       << current_cluster.size() << COLOR_DEBUG);
      one_cluster_in_roi_with_enough_points |= false;
    }
  }
  return !strict || one_cluster_in_roi_with_enough_points;
}

void LaneDetection::deleteLineWithFewPoints(const LineSpec& line,
                                            LineVehiclePoints* line_points) const {
  if (!line_points->empty() &&
      line_points->at(line).size() <
          static_cast<std::size_t>(parameters_ptr_->getParam(NAV_POINTS_TRESHOLD))) {
    ROS_DEBUG("Deleted %s line because of too few points", toString(line).c_str());
    line_points->at(line).clear();
  }
}

bool LaneDetection::tryNoPassingLine(const cv::Mat& image,
                                     const VehiclePoints* out_line_middle,
                                     VehiclePoints* no_passing_line) {
  const ScanLines sc =
      feature_extraction->createScanLines(lane_model_[LINESPEC_MIDDLE].get());

  VehiclePoints left_middle_line_vec, right_middle_line;
  if (!feature_extraction->isDoubleLine(image, sc, &left_middle_line_vec, &right_middle_line)) {
    ROS_DEBUG_STREAM(COLOR_MAGENTA
                     << "no double middle line detected in current image frame."
                     << COLOR_DEBUG);
    return false;
  }

  const VehiclePoints right_middle_line_vec = filterTransformMiddlePoints(right_middle_line);

  const auto min_size =
      static_cast<std::size_t>(parameters_ptr_->getParam(CLUSTER_MIN_SIZE));

  if (left_middle_line_vec.size() < min_size) {
    ROS_DEBUG_STREAM(COLOR_MAGENTA << "left_middle_line contains only "
                                   << left_middle_line_vec.size() << " points."
                                   << COLOR_DEBUG);
    return false;
  }

  ROS_DEBUG_STREAM("left_middle_line contains: " << left_middle_line_vec.size());

  if (right_middle_line_vec.empty()) {
    return false;
  }

  auto npl_begin = common::min_score(
      right_middle_line_vec, common::distanceTo(left_middle_line_vec.front()));
  auto npl_end = common::min_score(
      right_middle_line_vec, common::distanceTo(left_middle_line_vec.back()));
  if (npl_begin > npl_end) {
    ROS_ERROR(
        "The strange case npl_begin > npl_ed did REALLY occure!!! Swaped "
        "them");
    std::swap(npl_begin, npl_end);
  }
  no_passing_line->insert(no_passing_line->begin(), npl_begin, npl_end);

  // Fall: 2 durchgezogene Linien
  if (!isDashed(left_middle_line_vec)) {
    ROS_DEBUG_STREAM(COLOR_CYAN << " left part of middle line is solid." << COLOR_DEBUG);
    return true;
  }
  ROS_DEBUG_STREAM(COLOR_MAGENTA << " left part of middle line is dashed." << COLOR_DEBUG);

  // Fall: eine durchgezogene und eine gestrichelte Linie
  if (!isDashed(right_middle_line_vec)) {
    ROS_DEBUG_STREAM(COLOR_CYAN << "Right part of middle line is solid." << COLOR_DEBUG);
    return true;
  }

  ROS_DEBUG_STREAM(COLOR_MAGENTA << " Right part of middle line is dashed." << COLOR_DEBUG);
  // localizeOutlinePoints(no_passing_line, out_line_middle);
  no_passing_line->clear();
  const double cluster_threshold = parameters_ptr_->getParam(CLUSTER_THRESHOLD_LEFT_RIGHT);
  std::vector<VehiclePoints> clusters =
      EuclideanLinearClustering<VehiclePoint>(cluster_threshold).cluster(*out_line_middle);
  deleteTooSmallClusters(&clusters);
  return false;
}

VehiclePoints LaneDetection::filterTransformMiddlePoints(const VehiclePoints& double_line_points) {
  const double cluster_threshold = parameters_ptr_->getParam(POLYNOMIAL_STEP) +
                                   parameters_ptr_->getParam(RELATIVE_CLUSTER_THRESHOLD);
  const std::vector<VehiclePoints> clusters =
      EuclideanLinearClustering<VehiclePoint>(cluster_threshold).cluster(double_line_points);

  ROS_DEBUG(COLOR_RED "LinearCLustering found %lu clusters", clusters.size());
  const unsigned long min_cluster_size = static_cast<unsigned long>(
      parameters_ptr_->getParam(MIDLINE_RECOGNITION_MIN_NUMBER_OF_POINTS_IN_CLUSTER));
  const auto is_big_enough =
      [min_cluster_size](const auto& c) { return c.size() > min_cluster_size; };

  VehiclePoints filtered_points;
  filtered_points.reserve(double_line_points.size());
  for (const VehiclePoints& cluster : clusters) {
    if (is_big_enough(cluster)) {
      boost::push_back(filtered_points, cluster);
    }
  }
  ROS_DEBUG(COLOR_RED "After filtering: %lu clusters",
            boost::count_if(clusters, is_big_enough));
  return filtered_points;
}

void LaneDetection::checkDoubleMiddleLines(const cv::Mat& image,
                                           LineVehiclePoints* out_line_data) {
  if (!lane_model_[LINESPEC_MIDDLE]) {
    return;
  }

  VehiclePoints& no_passing_line = (*out_line_data)[LINESPEC_NO_PASSING];
  if (!tryNoPassingLine(image, &(*out_line_data)[LINESPEC_MIDDLE], &no_passing_line) &&
      !no_passing_line.empty()) {
    boost::push_back((*out_line_data)[LINESPEC_MIDDLE], no_passing_line);
    common::sortAlongPrincipalComponent(&(*out_line_data)[LINESPEC_MIDDLE]);
    deleteDoublePoints(&(*out_line_data)[LINESPEC_MIDDLE]);
    no_passing_line.clear();
    return;
  }
  deleteDoublePoints(&no_passing_line);
}

bool LaneDetection::isDashed(const VehiclePoints& points) {
  const double cluster_threshold = parameters_ptr_->getParam(CLUSTER_THRESHOLD_LEFT_RIGHT);
  std::vector<VehiclePoints> clusters =
      EuclideanLinearClustering<VehiclePoint>(cluster_threshold).cluster(points);
  ROS_DEBUG_STREAM(COLOR_CYAN
                   << "cluster size before function deleteTooSmallClusters: "
                   << clusters.size() << COLOR_DEBUG);
  deleteTooSmallClusters(&clusters);
  ROS_DEBUG_STREAM(COLOR_YELLOW
                   << "cluster size after function deleteTooSmallClusters: "
                   << clusters.size() << COLOR_DEBUG);
  return !largeClusters(clusters);
}

void LaneDetection::deleteTooSmallClusters(std::vector<VehiclePoints>* clusters) {
  unsigned long min_size =
      static_cast<unsigned long>(parameters_ptr_->getParam(CLUSTER_MIN_SIZE));
  boost::remove_erase_if(*clusters, [&](auto& c) { return c.size() < min_size; });
}

void LaneDetection::localizeOutlinePoints(VehiclePoints* right_line_middle,
                                          VehiclePoints* out_line_middle) {
  const double step = parameters_ptr_->getParam(DISCRETIZATION_STEP);
  common::unique_erase(*right_line_middle, common::areClose(step));
  common::unique_erase(*out_line_middle, common::areClose(step));
  const double filter_threshold = parameters_ptr_->getParam(OUTLINE_LOCALIZATION_THRESHOLD);
  for (const VehiclePoint& out_point : *out_line_middle) {
    if (right_line_middle->empty()) {
      break;
    }
    const auto nearest =
        *common::min_score(*right_line_middle, common::distanceTo(out_point));
    if ((out_point - nearest).norm() < filter_threshold) {
      boost::remove_erase(*right_line_middle, nearest);
      boost::remove_erase(*out_line_middle, out_point);
    }
  }
  boost::push_back(*out_line_middle, *right_line_middle);
  common::sortAlongPrincipalComponent(out_line_middle);
}

void LaneDetection::deleteDoublePoints(VehiclePoints* line_points) {
  const double eps = parameters_ptr_->getParam(VEHICLE_POINT_EQUALS_EPS);
  common::unique_erase(*line_points, common::areClose(eps));
}

bool LaneDetection::largeClusters(std::vector<VehiclePoints>& clusters) {
  const double cluster_min_length = parameters_ptr_->getParam(MINIMAL_CLUSTER_LENGTH);
  for (auto& cluster : clusters) {
    common::sortAlongPrincipalComponent(&cluster);
    ROS_DEBUG(COLOR_BLUE "cluster length = %e", (cluster.back() - cluster.front()).norm());
  }
  return std::any_of(clusters.begin(),
                     clusters.end(),
                     [&cluster_min_length](const auto& c) {
                       return (c.back() - c.front()).norm() > cluster_min_length;
                     });
}
