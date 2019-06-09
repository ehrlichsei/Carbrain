#include "parking_end_classifier.h"

THIRD_PARTY_HEADERS_BEGIN
#include <ros/console.h>
#include <boost/algorithm/cxx11/copy_if.hpp>
#include <boost/range/adaptors.hpp>
#include <boost/range/algorithm.hpp>
#include <boost/range/algorithm_ext/erase.hpp>
#include <boost/range/algorithm_ext/push_back.hpp>
THIRD_PARTY_HEADERS_END
#include "../utils/step_detection.h"
#include "common/angle_conversions.h"
#include "common/best_score.h"
#include "common/eigen_functors.h"
#include "common/eigen_utils.h"
#include "common/make_vector.h"
#include "common/normal_shift.h"
#include "common/polynomialfit.h"

namespace perpendicular_parking {

using boost::adaptors::filtered;
using boost::adaptors::transformed;

const std::string ParkingEndClassifier::NAMESPACE("parking_end_classifier");

const ParameterString<double> ParkingEndClassifier::LEFT_LANE_POLY_STEP(
    NAMESPACE + "/left_lane_poly_step");
const ParameterString<double> ParkingEndClassifier::SCAN_LINE_LENGTH(
    NAMESPACE + "/scan_line_length");
const ParameterString<double> ParkingEndClassifier::STEP_DETECTION_THLD(
    NAMESPACE + "/step_detection_threshold");
const ParameterString<int> ParkingEndClassifier::STEP_DETECTION_REF_FUNC_SIZE(
    NAMESPACE + "/step_detection_ref_func_size");
const ParameterString<double> ParkingEndClassifier::PARKING_SPOT_DEPTH(
    "parking_spot_depth");
const ParameterString<double> ParkingEndClassifier::LEFT_LANE_SCANLINE_PADDING(
    NAMESPACE + "/left_lane_scanline_padding");
const ParameterString<double> ParkingEndClassifier::ADDITIONAL_PADDING(
    NAMESPACE + "/additional_padding");
const ParameterString<double> ParkingEndClassifier::SCANLINE_X_OFFSET(
    NAMESPACE + "/scanline_x_offset");
const ParameterString<double> ParkingEndClassifier::MINIMAL_HYPOTHESE_BELIEF(
    NAMESPACE + "/minimal_hypothese_belief");
const ParameterString<double> ParkingEndClassifier::MARKING_SCANLINE_SHIFT(
    NAMESPACE + "/marking_scanline_shift");
const ParameterString<int> ParkingEndClassifier::NO_MARKING_SCANLINES(
    NAMESPACE + "/no_marking_scanlines");
const ParameterString<int> ParkingEndClassifier::MIN_CLUSTER_SIZE(
    NAMESPACE + "/min_cluster_size");
const ParameterString<double> ParkingEndClassifier::RANSAC_MIN_DIST(
    NAMESPACE + "/ransac/minimal_model_distance");
const ParameterString<int> ParkingEndClassifier::RANSAC_MIN_CLUSTER_SIZE(
    NAMESPACE + "/ransac/minimal_consensus_set_size");
const ParameterString<int> ParkingEndClassifier::RANSAC_NR_LINES(
    NAMESPACE + "/ransac/expected_nr_of_lines");
const ParameterString<double> ParkingEndClassifier::MINIMUM_ENDLINE_ANGLE(
    NAMESPACE + "/minimum_endline_angle");
const ParameterString<double> ParkingEndClassifier::MAXIMUM_ENDLINE_ANGLE(
    NAMESPACE + "/maximum_endline_angle");

ParkingEndClassifier::ParkingEndClassifier(
    ParameterInterface *const parameters_ptr,
    const common::CameraTransformation *const camera_transform,
    const tf_helper::TFHelperInterface<double> *const world_coordinates_helper)
    : parameters_ptr_(parameters_ptr),
      cam_transform_(camera_transform),
      world_coordinates_helper_(world_coordinates_helper),
      db_clusterer_(parameters_ptr, NAMESPACE + "/dbscan") {
  parameters_ptr->registerParam(LEFT_LANE_POLY_STEP);
  parameters_ptr->registerParam(SCAN_LINE_LENGTH);
  parameters_ptr->registerParam(STEP_DETECTION_THLD);
  parameters_ptr->registerParam(STEP_DETECTION_REF_FUNC_SIZE);
  parameters_ptr->registerParam(LEFT_LANE_SCANLINE_PADDING);
  parameters_ptr->registerParam(ADDITIONAL_PADDING);
  parameters_ptr->registerParam(SCANLINE_X_OFFSET);
  parameters_ptr->registerParam(MINIMAL_HYPOTHESE_BELIEF);
  parameters_ptr->registerParam(MARKING_SCANLINE_SHIFT);
  parameters_ptr->registerParam(NO_MARKING_SCANLINES);
  parameters_ptr->registerParam(MIN_CLUSTER_SIZE);
  parameters_ptr->registerParam(RANSAC_MIN_DIST);
  parameters_ptr->registerParam(RANSAC_MIN_CLUSTER_SIZE);
  parameters_ptr->registerParam(RANSAC_NR_LINES);
  parameters_ptr->registerParam(MINIMUM_ENDLINE_ANGLE);
  parameters_ptr->registerParam(MAXIMUM_ENDLINE_ANGLE);

  // register params of Endline and along with that of ParkingLine
  Endline::registerParams(parameters_ptr);
}

boost::optional<WorldPose> ParkingEndClassifier::detectEnd(
    const ParkingSpotsConstRef &all_parking_spots,
    const cv::Mat &img,
    const ros::Time &timestamp,
    const common::DynamicPolynomial &left_lane_polynom,
    const LineVehiclePoints &lanes,
    const MapPose &world_T_map) {

  // scan parking spots for end line
  if (all_parking_spots.empty() || lanes[LINESPEC_LEFT].empty()) {
    return boost::none;
  }

  // compute discretization_params
  const auto disc_params = generateDiscParams(lanes, all_parking_spots, world_T_map);

  // extract feature_points for endlines
  const auto endline_feature_points =
      getEndlineFeaturePoints(img, left_lane_polynom, disc_params);

  // cluster detected points and return the cluster that's located the farest
  // away
  const auto clusters = extractClusters(endline_feature_points);

  if (clusters.empty()) {
    return boost::none;
  }

  for (const auto &cluster : clusters) {
    // compute point in cluster with maximum x-coordinate
    const auto farest_cluster_point =
        *common::max_score(cluster.feature_points_vehicle, common::x_value{});

    // update endlines with cluster and free_depth
    updateEndlines(cluster,
                   left_lane_polynom,
                   timestamp,
                   computeFreeDepth(img, left_lane_polynom, farest_cluster_point));
  }

  if (parking_endlines_.empty()) {
    return boost::none;
  }

  const auto min_certainity = parameters_ptr_->getParam(MINIMAL_HYPOTHESE_BELIEF);
  const auto end_line = common::max_score(
      parking_endlines_, [](const auto &el) { return el.belief(); });

  return boost::make_optional(end_line->belief() >= min_certainity, end_line->pose());
}

void ParkingEndClassifier::reset() {
  parking_endlines_.clear();
  id_count_ = 0;
}

ImagePoints ParkingEndClassifier::apply1DGradientDetector(const cv::Mat &img,
                                                          const ScanLines &scan_lines,
                                                          const bool all) const {
  ImagePoints image_points;

  const float threshold =
      static_cast<float>(parameters_ptr_->getParam(STEP_DETECTION_THLD));
  const unsigned int range_length =
      static_cast<unsigned int>(parameters_ptr_->getParam(STEP_DETECTION_REF_FUNC_SIZE));

  const std::vector<float> reference_function =
      step_detection::createReferenceFunction(range_length);

  for (const ScanLine &scan_line : scan_lines) {
    auto feature_points = step_detection::detectStep(
        img, scan_line, reference_function, threshold, true);

    // assume that feature_points are sorted

    if (!all && !feature_points.empty()) {
      image_points.push_back(feature_points[0]);
    } else {
      boost::push_back(image_points, feature_points);
    }
  }
  return image_points;
}

template <typename Point>
inline Point shiftPerpendicularToPoly(const common::DynamicPolynomial &poly,
                                      const Point &on_poly,
                                      const double dist) {
  const Point normal_vector = common::to<Point>(common::normal(poly, on_poly.x()));
  return on_poly + dist * normal_vector;
}

ImagePoints ParkingEndClassifier::getEndlineFeaturePoints(
    const cv::Mat &img,
    const common::DynamicPolynomial &left_lane_polynom,
    const common::DiscretizationParams &disc_params) const {

  // create Scanlines for feature point detection
  const double scanline_length = parameters_ptr_->getParam(PARKING_SPOT_DEPTH);
  const double padding = parameters_ptr_->getParam(LEFT_LANE_SCANLINE_PADDING);

  VehiclePoints start_points, end_points;
  common::normalShift(left_lane_polynom, padding, disc_params, &start_points);
  common::normalShift(left_lane_polynom, scanline_length, disc_params, &end_points);

  ImagePoints start_points_img, end_points_img;
  cam_transform_->transformGroundToImage(start_points, &start_points_img);
  cam_transform_->transformGroundToImage(end_points, &end_points_img);

  auto endline_scanlines = createScanLines(start_points_img, end_points_img);

  const auto scanlines_clipped = clipScanLines(endline_scanlines, img.size());

  return apply1DGradientDetector(img, scanlines_clipped, false);
}

boost::optional<double> ParkingEndClassifier::computeFreeDepth(
    const cv::Mat &img,
    const common::DynamicPolynomial &left_lane_polynom,
    const VehiclePoint &farest_cluster_point) const {

  const double x_offset = parameters_ptr_->getParam(MARKING_SCANLINE_SHIFT);

  // create Scanlines start and end points
  const VehiclePoint start_point{
      farest_cluster_point.x(), left_lane_polynom.evaluate(farest_cluster_point.x()), 0.0};
  const double end_x = farest_cluster_point.x() + x_offset;
  const VehiclePoint end_point{end_x, left_lane_polynom.evaluate(end_x), 0.0};

  // extract endlines
  auto marking_scanlines = getParallelScanlines(left_lane_polynom, start_point, end_point);

  const auto clipped_lines = clipScanLines(marking_scanlines, img.size());
  // get feature_points
  const auto fp = apply1DGradientDetector(img, clipped_lines, true);

  // search for clusters, that have enough members
  const auto output_clusters = extractClusters(fp);

  // return cluster the farest away from vehicle
  const auto valid_clusters = boost::make_optional(
      !output_clusters.empty(), *common::min_score(output_clusters, [](const auto &cl) {
        return common::min_score(cl.feature_points_vehicle,
                                 [](const auto &vp) { return vp.norm(); })
            ->norm();
      }));

  // if no such clusters found, free depth is inifinity --> none returned
  if (!valid_clusters) {
    return boost::none;
  }

  // compute point with minimal x-coordinate in cluster
  const auto nearest_in_cluster =
      common::min_score(valid_clusters->feature_points_vehicle, common::x_value{});

  // FIXME should this value be clamped?
  const auto value = nearest_in_cluster->x() - farest_cluster_point.x();

  return boost::make_optional(value);
}

ScanLines ParkingEndClassifier::getParallelScanlines(const common::DynamicPolynomial &polynom,
                                                     const VehiclePoint &start_point,
                                                     const VehiclePoint &end_point) const {
  // get dist_step and number of scanlines within
  const std::size_t no_scanlines =
      static_cast<std::size_t>(parameters_ptr_->getParam(NO_MARKING_SCANLINES));
  const double padding = parameters_ptr_->getParam(LEFT_LANE_SCANLINE_PADDING);
  const double step = parameters_ptr_->getParam(LEFT_LANE_POLY_STEP);

  // create ScanLines
  ScanLines scanlines;
  scanlines.reserve(no_scanlines);
  for (std::size_t no = 0; no < no_scanlines; no++) {
    const ImagePoint start = cam_transform_->transformGroundToImage(
        shiftPerpendicularToPoly(polynom, start_point, padding + no * step));
    const ImagePoint end = cam_transform_->transformGroundToImage(
        shiftPerpendicularToPoly(polynom, end_point, padding + no * step));
    scanlines.emplace_back(start, end);
  }

  return scanlines;
}

ScanLines ParkingEndClassifier::createScanLines(const ImagePoints &start_points,
                                                const ImagePoints &end_points) const {
  ScanLines scan_lines;
  boost::transform(start_points,
                   end_points,
                   std::back_inserter(scan_lines),
                   [](const auto &s, const auto &e) { return ScanLine(s, e); });
  return scan_lines;
}

ScanLines ParkingEndClassifier::clipScanLines(const ScanLines &unclipped,
                                              const cv::Size &img_size) const {
  ScanLines clipped_lines;
  clipped_lines.reserve(unclipped.size());
  for (auto sc : unclipped) {
    if (!ScanLine::clip(img_size, sc)) {
      continue;
    }
    clipped_lines.push_back(sc);
  }
  return clipped_lines;
}


FeaturePointClusters ParkingEndClassifier::extractClusters(const ImagePoints &feature_points) const {
  // cluster detected endline feature points
  const FeaturePointCluster input{*cam_transform_, feature_points};
  auto output_clusters = db_clusterer_.cluster(input);

  //  assert(!output_clusters.empty() && "output_clusters must never be
  //  empty!");
  if (output_clusters.empty()) {
    return output_clusters;
  }

  // remove clusters, that are too small
  const auto min_cluster_size =
      static_cast<std::size_t>(parameters_ptr_->getParam(MIN_CLUSTER_SIZE));
  boost::remove_erase_if(output_clusters, [&min_cluster_size](const auto &c) {
    return c.feature_points_vehicle.size() <= min_cluster_size;
  });

  return output_clusters;
}

std::size_t ParkingEndClassifier::generateID() {
  id_count_++;
  return id_count_;
}

void ParkingEndClassifier::updateEndlines(const FeaturePointCluster &cluster,
                                          const common::DynamicPolynomial &left_lane_polynom,
                                          const ros::Time &stamp,
                                          const boost::optional<double> &free_space) {
  // coefficients of polynom
  const auto &coeffs = left_lane_polynom.getCoefficients();
  const auto ll_c = coeffs[0];
  const auto ll_m = coeffs[1];

  // extraction of state (intersection of two lines for coordinates)
  const auto toVehicleState = [&ll_m, &ll_c](const auto &line) {
    const auto line_coeffs = line.getCoefficients();
    const auto intersection_x = (line_coeffs[0] - ll_c) / (ll_m - line_coeffs[1]);
    return Eigen::Vector4d{intersection_x,
                           line.evaluate(intersection_x),
                           std::atan(line_coeffs[1]),
                           std::atan(line_coeffs[1]) - std::atan(ll_m)};
  };
  // check if angle is in valid range
  const auto MIN_ENDLINE_ANGLE = parameters_ptr_->getParam(MINIMUM_ENDLINE_ANGLE);
  const auto MAX_ENDLINE_ANGLE = parameters_ptr_->getParam(MAXIMUM_ENDLINE_ANGLE);
  ;

  const auto isValid = [&MIN_ENDLINE_ANGLE, &MAX_ENDLINE_ANGLE](const auto &state) {
    ROS_DEBUG("endline detection, difference angle is %f", state.w());
    return state.w() > MIN_ENDLINE_ANGLE && state.w() < MAX_ENDLINE_ANGLE;
  };

  const std::size_t nr_lines =
      static_cast<std::size_t>(parameters_ptr_->getParam(RANSAC_NR_LINES));
  // get lines in cluster
  const auto lines = fitLinesR(nr_lines, cluster.feature_points_vehicle);

  // prepare transformation of valid states to world_frame
  const auto toState = [](const auto &vs) {
    return State{vs.template head<3>()};
  };
  const auto toEndline = [this, &stamp](const auto &s) {
    return Endline{stamp, s, 0UL, parameters_ptr_};
  };

  const Eigen::Affine3d world_T_vehicle = world_coordinates_helper_->getTransform();

  const auto translationToWorldFrame = [&world_T_vehicle](const auto &s) {
    const Eigen::Vector2d world_translation = to2D(world_T_vehicle) * to2D(s);
    return State{world_translation.x(),
                 world_translation.y(),
                 common::toYaw(world_T_vehicle.linear() *
                               Eigen::AngleAxisd(s.z(), Eigen::Vector3d::UnitZ()))};
  };

  const auto actual_endlines = common::make_vector(
      lines | transformed(toVehicleState) | filtered(isValid) | transformed(toState) |
      transformed(translationToWorldFrame) | transformed(toEndline));

  for (const auto &endline : actual_endlines) {
    // search for lines that have been already seen
    const auto matched_line = boost::find(parking_endlines_, endline);
    if (matched_line == parking_endlines_.end()) {
      parking_endlines_.emplace_back(stamp, endline.state(), generateID(), parameters_ptr_);
    } else {
      matched_line->update(stamp, endline.state(), free_space);
    }
  }
}


inline auto modelSupport(const Line &model, const double min_dist) {
  return [&model, min_dist](const auto &p) {
    return std::fabs(model.distance(to2D(p))) < min_dist;
  };
};

std::vector<common::DynamicPolynomial> ParkingEndClassifier::fitLinesR(
    const std::size_t nr_lines, const VehiclePoints &points) const {
  std::vector<common::DynamicPolynomial> lines;
  lines.reserve(nr_lines);
  // use observations
  auto detected_points = points;
  const std::size_t max_it = detected_points.size();
  const double min_dist = parameters_ptr_->getParam(RANSAC_MIN_DIST);
  const std::size_t min_set_size =
      static_cast<std::size_t>(parameters_ptr_->getParam(RANSAC_MIN_CLUSTER_SIZE));
  VehiclePoints data_next_it;

  // loop in order to detect the specified number of lines
  for (std::size_t i = 0; i < nr_lines; i++) {

    std::size_t best_set_size = 0;
    boost::optional<common::DynamicPolynomial> best_model = boost::none;

    // ransac loop
    for (std::size_t it = 0; it < max_it; it++) {
      boost::random_shuffle(detected_points);
      const Line model =
          Line::Through(to2D(detected_points.front()), to2D(detected_points.back()));
      const auto divider =
          boost::partition(detected_points, modelSupport(model, min_dist));
      const VehiclePoints consensus_set(detected_points.begin(), divider);
      const auto model_size = consensus_set.size();
      if (model_size > best_set_size && model_size > min_set_size) {
        //  least squares fit for best model
        best_model = common::fitToPoints(consensus_set, 1);
        best_set_size = model_size;
        data_next_it.clear();
        data_next_it.insert(data_next_it.begin(), divider, detected_points.end());
      }
    }

    // if no model is found, break
    if (!best_model) {
      break;
    }

    // push back best line and remove points supporting the model
    lines.push_back(best_model.get());
    detected_points = data_next_it;
    if (boost::size(detected_points) < min_set_size) {
      break;
    }
  }

  return lines;
}

common::DiscretizationParams ParkingEndClassifier::generateDiscParams(
    const LineVehiclePoints &lane_points,
    const ParkingSpotsConstRef &all_parking_spots,
    const MapPose &world_T_map) const {

  // transformation from world to vehicle frame
  const Eigen::Affine3d vehicle_T_world =
      world_coordinates_helper_->getTransform().inverse();

  // determine start_point for scanlines as right entrance of first parking spot
  const VehiclePoint scanlines_start_point =
      vehicle_T_world * world_T_map *
      VehiclePoint{all_parking_spots.front().get().rightEntrance().translation()};

  // determine endline candidate with maximum balief, if there are any
  const boost::optional<Endline> valid_endline = boost::make_optional<Endline>(
      !parking_endlines_.empty(),
      *common::max_score(parking_endlines_,
                         [](const auto &el) { return el.belief(); }));

  // parameters needed for computation of end point
  const double minimal_belief = parameters_ptr_->getParam(MINIMAL_HYPOTHESE_BELIEF);
  const double x_offset = parameters_ptr_->getParam(SCANLINE_X_OFFSET);

  // compute end point: if there is an endline_ with high belief, take its pose
  // as outmost basis for endpoint and add offset; if no such endline present,
  // take lane detection point with maximum x-coordinate and add offset
  const VehiclePoint scanlines_end_point =
      valid_endline && valid_endline.get().belief() > minimal_belief
          ? vehicle_T_world * WorldPoint{valid_endline.get().pose().translation()} +
                x_offset * VehiclePoint::UnitX()
          : *common::max_score(join(lane_points), common::x_value{}) +
                x_offset * VehiclePoint::UnitX();

  const double dis_step = parameters_ptr_->getParam(LEFT_LANE_POLY_STEP);
  return {scanlines_start_point.x(), scanlines_end_point.x(), dis_step};
}


}  // namespace perpendicular_parking
