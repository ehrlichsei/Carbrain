#include "no_passing_zone_classifier.h"
#include <ros/console.h>
#include <boost/range/algorithm_ext.hpp>
#include "../../utils/step_detection.h"
#include "../src/utils/foot_finder.h"
#include "common/console_colors.h"
#include "common/polynomialfit.h"
#include "common/types.h"
#include "opencv_eigen_conversions.h"

namespace road_object_detection {

const std::string NoPassingZoneClassifier::NAMESPACE(
    "no_passing_zone_classifier");

// lane strip + tolerance
//    SCAN_LEFT  SCAN_RIGHT
// |_____________|_____|
//  ---|---|---|-x-|--
//  ---|---|---|-x-|--
//               |
//             NP Point

const ParameterString<double> NoPassingZoneClassifier::SCAN_RIGHT(
    NAMESPACE + "/scan_right");
const ParameterString<double> NoPassingZoneClassifier::SCAN_LEFT(NAMESPACE +
                                                                 "/scan_left");
const ParameterString<int> NoPassingZoneClassifier::STEP_THRESHOLD(
    NAMESPACE + "/step_threshold");
const ParameterString<int> NoPassingZoneClassifier::STEP_LENGTH(NAMESPACE +
                                                                "/step_length");
const ParameterString<double> NoPassingZoneClassifier::MAX_LINE_LINE_DEV(
    NAMESPACE + "/max_line_line_dev");
const ParameterString<double> NoPassingZoneClassifier::MAX_LINE_SPACE_DEV(
    NAMESPACE + "/max_line_space_dev");
const ParameterString<double> NoPassingZoneClassifier::LINE_WIDTH(
    NAMESPACE + "/line_width");
const ParameterString<double> NoPassingZoneClassifier::MAX_LINE_WIDTH_DEV(
    NAMESPACE + "/max_line_width_dev");
const ParameterString<double> NoPassingZoneClassifier::MIN_LENGTH_NP_ZONE(
    NAMESPACE + "/min_length_np_zone");
const ParameterString<int> NoPassingZoneClassifier::TWO_SOLID_LINES_QUAD_DOUBLE_RELATION(
    NAMESPACE + "/two_solid_lines_quad_double_relation");

NoPassingZoneClassifier::NoPassingZoneClassifier(
    const common::CameraTransformation *const camera_transformation,
    common::node_base::ParameterInterface *const parameter_interface)
    : camera_transformation_(camera_transformation),
      parameters_ptr_(parameter_interface) {

  parameter_interface->registerParam(SCAN_RIGHT);
  parameter_interface->registerParam(SCAN_LEFT);
  parameter_interface->registerParam(STEP_THRESHOLD);
  parameter_interface->registerParam(STEP_LENGTH);
  parameter_interface->registerParam(MAX_LINE_LINE_DEV);
  parameter_interface->registerParam(MAX_LINE_SPACE_DEV);
  parameter_interface->registerParam(LINE_WIDTH);
  parameter_interface->registerParam(MAX_LINE_WIDTH_DEV);
  parameter_interface->registerParam(MIN_LENGTH_NP_ZONE);
  parameter_interface->registerParam(TWO_SOLID_LINES_QUAD_DOUBLE_RELATION);
}

NoPassingZoneClassifier::Params NoPassingZoneClassifier::readParameters() const {
  Params params{};

  params.scan_right = parameters_ptr_->getParam(SCAN_RIGHT);
  params.scan_left = parameters_ptr_->getParam(SCAN_LEFT);
  params.step_threshold = parameters_ptr_->getParam(STEP_THRESHOLD);
  params.step_length = static_cast<std::size_t>(parameters_ptr_->getParam(STEP_LENGTH));
  params.max_line_line_dev = parameters_ptr_->getParam(MAX_LINE_LINE_DEV);
  params.max_line_space_dev = parameters_ptr_->getParam(MAX_LINE_SPACE_DEV);
  params.line_width = parameters_ptr_->getParam(LINE_WIDTH);
  params.max_line_width_dev = parameters_ptr_->getParam(MAX_LINE_WIDTH_DEV);
  params.min_length_np_zone = parameters_ptr_->getParam(MIN_LENGTH_NP_ZONE);
  params.two_solid_lines_quad_double_relation =
      parameters_ptr_->getParam(TWO_SOLID_LINES_QUAD_DOUBLE_RELATION);

  return params;
}

RoadObjects NoPassingZoneClassifier::classify(const Features &features) {
  if (features.no_passing_points.empty()) {
    return {};
  }
  const auto params = readParameters();

  // getSteps(features);
  RoadObjects no_passing_zones;

  const auto all_step_clstrs_raw = getSteps(features, params);

  const auto double_step_clstrs = selectMultiSteps(all_step_clstrs_raw, 2);
  const auto quad_step_clstrs = selectMultiSteps(all_step_clstrs_raw, 4);

  const auto double_step_clstrs_filtered =
      filterLineWidth(double_step_clstrs,
                      parameters_ptr_->getParam(LINE_WIDTH),
                      parameters_ptr_->getParam(MAX_LINE_WIDTH_DEV));
  const auto quad_step_clstrs_filtered = filterRelativeDeviation(
      filterLineWidth(quad_step_clstrs, params.line_width, params.max_line_width_dev),
      params.max_line_line_dev,
      params.max_line_space_dev);

  if (quad_step_clstrs_filtered.size() > 2) {  // if at least 2 quads

    const VehicleScanLine np_line = {quad_step_clstrs_filtered.front()[3],
                                     quad_step_clstrs_filtered.back()[3]};

    if ((np_line.start - np_line.end).norm() > params.min_length_np_zone) {  // and long enough

      if (quad_step_clstrs_filtered.size() > static_cast<std::size_t>(params.two_solid_lines_quad_double_relation) *
                                                 double_step_clstrs_filtered.size() ||  // if we have 2 solid lines
          solidLineIsRight(
              double_step_clstrs_filtered, quad_step_clstrs_filtered, features, params)) {

        VehiclePose pose_in_vehicle = Eigen::Translation3d{np_line.start} *
                                      Eigen::AngleAxisd{features.cluster_center_lane_orientation,
                                                        VehiclePoint::UnitZ()};

        using NoPassingZonePoint = Eigen::Vector3d;

        //        auto startpoint_leftshift = np_line.start;
        //        startpoint_leftshift.y() += 0.10;
        //        auto startpoint_rightshift = np_line.start;
        //        startpoint_rightshift.y() -= 0.10;
        //        auto endpoint_leftshift = np_line.end;
        //        endpoint_leftshift.y() += 0.10;
        //        auto endpoint_rightshift = np_line.end;
        //        endpoint_rightshift.y() -= 0.10;

        const NoPassingZonePoint diff = np_line.end - np_line.start;
        const VehiclePoints base_hull_polygon_in_vehicle = {
            pose_in_vehicle * (0.1 * NoPassingZonePoint::UnitY()),
            pose_in_vehicle * (-0.1 * NoPassingZonePoint::UnitY()),
            pose_in_vehicle * (diff + 0.1 * NoPassingZonePoint::UnitY()),
            pose_in_vehicle * (diff - 0.1 * NoPassingZonePoint::UnitY())};

        //        base_hull_polygon_in_vehicle.push_back(startpoint_leftshift);
        //        base_hull_polygon_in_vehicle.push_back(startpoint_rightshift);

        //        base_hull_polygon_in_vehicle.push_back(endpoint_leftshift);
        //        base_hull_polygon_in_vehicle.push_back(endpoint_rightshift);

        no_passing_zones.push_back(std::make_unique<NoPassingZone>(
            features.timestamp, 1.0, pose_in_vehicle, base_hull_polygon_in_vehicle, np_line.start, np_line.end));

        return no_passing_zones;
      }
    }
    return no_passing_zones;
  }
  return no_passing_zones;
}

VehiclePoints NoPassingZoneClassifier::selectNthSteps(const std::vector<VehiclePoints> &step_clstrs,
                                                      const std::size_t stepnumber) const {

  VehiclePoints selected_steps;
  selected_steps.reserve(step_clstrs.size());

  for (const auto &scl : step_clstrs) {
    selected_steps.push_back(scl[stepnumber]);
  }

  return selected_steps;
}

bool NoPassingZoneClassifier::solidLineIsRight(const std::vector<VehiclePoints> &double_step_clstrs,
                                               const std::vector<VehiclePoints> &quad_step_clstrs,
                                               const Features &features,
                                               const Params &params) const {

  if (double_step_clstrs.empty() || quad_step_clstrs.empty()) {
    return false;
  }

  const VehiclePoints right_steps_double = selectNthSteps(double_step_clstrs, 1);
  const VehiclePoints right_steps_quad = selectNthSteps(quad_step_clstrs, 3);

  const auto poly_deg = static_cast<common::PolynomialDegree>(
      features.middle_lane_polynomial.getCoefficients().size() - 1);


  if (right_steps_quad.size() <= static_cast<std::size_t>(poly_deg) ||
      common::numberOfDistinctXValues(right_steps_quad) <= static_cast<std::size_t>(poly_deg)) {
    return false;
  }

  const auto polynomial = common::fitToPoints(right_steps_quad, poly_deg);

  const auto avg_dist_double =
      common::computeAbsoluteFittingError(right_steps_double, polynomial);
  return avg_dist_double < params.line_width;
}

std::vector<VehiclePoints> NoPassingZoneClassifier::getSteps(const Features &features,
                                                             const Params &params) const {

  std::vector<VehiclePoints> all_step_clstrs_raw;  // all_steps_raw cotains
  // singlesteps, doublesteps,
  all_step_clstrs_raw.reserve((features.no_passing_points.size()));
  for (const auto &p : features.no_passing_points) {  // for each NP point

    const auto footpoint = utils::findLotfusspunkt(features.middle_lane_polynomial, p);

    const Eigen::Vector3d diff_to_polynomial = (footpoint - p).normalized().cwiseAbs();
    //    diff_to_polynomial.normalize();
    //    diff_to_polynomial = diff_to_polynomial.cwiseAbs();
    // create a copy and shift left and shift right
    const auto scnpnt_l = p + diff_to_polynomial * params.scan_left;
    const auto scnpnt_r = p - diff_to_polynomial * params.scan_right;

    const auto ref_func = step_detection::createReferenceFunction(params.step_length);
    ImagePoints step_clstr_raw_img = step_detection::detectStep(
        features.image_complete,
        ScanLine(camera_transformation_->transformVehicleToImage(scnpnt_l),
                 camera_transformation_->transformVehicleToImage(scnpnt_r)),
        ref_func,
        params.step_threshold,
        true);

    VehiclePoints step_clstr_raw;
    camera_transformation_->transformImageToGround(step_clstr_raw_img, &step_clstr_raw);
    all_step_clstrs_raw.push_back(step_clstr_raw);
  }
  return all_step_clstrs_raw;
}

std::vector<VehiclePoints> NoPassingZoneClassifier::filterLineWidth(
    const std::vector<VehiclePoints> &step_clstrs_gnd,
    const double line_width,
    const double max_deviation) const {

  std::vector<VehiclePoints> valid_step_clstrs;
  valid_step_clstrs.reserve(step_clstrs_gnd.size());

  for (const auto &step_clstr_gnd : step_clstrs_gnd) {
    const auto first_line_width = step_clstr_gnd[0].y() - step_clstr_gnd[1].y();

    if (std::fabs(first_line_width - line_width) / line_width < max_deviation) {
      valid_step_clstrs.push_back(step_clstr_gnd);
    }
  }
  return valid_step_clstrs;
}

std::vector<VehiclePoints> NoPassingZoneClassifier::filterRelativeDeviation(
    const std::vector<VehiclePoints> &step_clstrs_gnd,
    const double max_line2line_deviation,
    const double max_line2space_deviation) const {

  std::vector<VehiclePoints> valid_step_clstrs;
  valid_step_clstrs.reserve(step_clstrs_gnd.size());

  for (const auto &step_clstr_gnd : step_clstrs_gnd) {
    const auto first_line_width = step_clstr_gnd[0].y() - step_clstr_gnd[1].y();
    const auto space_between_lines = step_clstr_gnd[1].y() - step_clstr_gnd[2].y();
    const auto second_line_width = step_clstr_gnd[2].y() - step_clstr_gnd[3].y();

    if (std::fabs(first_line_width - second_line_width) / first_line_width < max_line2line_deviation &&
        std::fabs(first_line_width - space_between_lines) / first_line_width <
            max_line2space_deviation) {

      valid_step_clstrs.push_back(step_clstr_gnd);
    }
  }
  return valid_step_clstrs;
}

std::vector<VehiclePoints> NoPassingZoneClassifier::selectMultiSteps(
    const std::vector<VehiclePoints> &step_clstrs_gnd, const unsigned int steps) const {

  std::vector<VehiclePoints> multi_step_clstrs;
  multi_step_clstrs.reserve(step_clstrs_gnd.size());

  for (const auto &step_clstr_gnd : step_clstrs_gnd) {
    if (step_clstr_gnd.size() == steps) {
      multi_step_clstrs.push_back(step_clstr_gnd);
    }
  }
  return multi_step_clstrs;
}

size_t NoPassingZoneClassifier::getClassifierId() const {
  return typeid(NoPassingZoneClassifier).hash_code();
}

}  // namespace road_object_detection
