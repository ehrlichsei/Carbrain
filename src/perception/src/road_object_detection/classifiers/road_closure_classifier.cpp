#include "road_closure_classifier.h"

THIRD_PARTY_HEADERS_BEGIN
#include <boost/algorithm/clamp.hpp>
#include <boost/algorithm/cxx11/all_of.hpp>
#include <boost/geometry/algorithms/intersection.hpp>
#include <boost/range/adaptor/filtered.hpp>
#include <boost/range/adaptor/transformed.hpp>
#include <boost/range/algorithm/max_element.hpp>
#include <boost/range/algorithm/min_element.hpp>
#include <boost/range/algorithm/nth_element.hpp>
#include <boost/range/algorithm/partition.hpp>
#include <boost/range/algorithm/random_shuffle.hpp>
#include <boost/range/algorithm_ext/erase.hpp>
#include <boost/range/algorithm_ext/push_back.hpp>

#include <ros/console.h>
#include <opencv2/imgproc.hpp>
THIRD_PARTY_HEADERS_END

#include "../../utils/foot_finder.h"
#include "../../utils/step_detection.h"
#include "common/angle_conversions.h"
#include "common/basic_statistics.h"
#include "common/basic_statistics_eigen.h"
#include "common/best_score.h"
#include "common/discretisize.h"
#include "common/eigen_adaptors.h"
#include "common/eigen_functors.h"
#include "common/make_vector.h"
#include "common/minmax_element.h"
#include "common/pca_eigen.h"
#include "common/polynomial_utils.h"
#include "common/polynomialfit.h"
#include "opencv_eigen_conversions.h"

namespace road_object_detection {

using boost::adaptors::filtered;
using boost::adaptors::transformed;
using common::make_eigen_vector;
using common::make_vector;

const std::string RoadClosureClassifier::NAMESPACE("road_closure_classifier");

const ParameterString<double> RoadClosureClassifier::RANSAC_MIN_DIST_BOUNDARY(
    NAMESPACE + "/ransac/minimal_model_distance_boundary");
const ParameterString<double> RoadClosureClassifier::RANSAC_ALLOWED_ANGLE_DEVIATION(
    NAMESPACE + "/ransac/allowed_angle_deviation");
const ParameterString<double> RoadClosureClassifier::RANSAC_MAX_DIST_INNER(
    NAMESPACE + "/ransac/minimal_model_distance");
const ParameterString<int> RoadClosureClassifier::RANSAC_MIN_SET_SIZE(
    NAMESPACE + "/ransac/minimal_consensus_set_size");
const ParameterString<int> RoadClosureClassifier::RANSAC_NR_LINES(
    NAMESPACE + "/ransac/expected_nr_of_lines");
const ParameterString<double> RoadClosureClassifier::FRONTLINE_ANGLE(
    NAMESPACE + "/model/frontline_angle");
const ParameterString<double> RoadClosureClassifier::INNER_LINE_ANGLE(
    NAMESPACE + "/model/inner_line_angle");
const ParameterString<double> RoadClosureClassifier::MINIMAL_AREA_SIZE_X(
    NAMESPACE + "/model/minimal_area_size_x");
const ParameterString<int> RoadClosureClassifier::MIN_SIZE_LINES(
    NAMESPACE + "/model/min_size_lines");
const ParameterString<double> RoadClosureClassifier::ADDITIONAL_SCAN_LINE_WINDOW_WIDTH(
    NAMESPACE + "/additional_scan_lines/window_width");
const ParameterString<double> RoadClosureClassifier::ADDITIONAL_SCAN_LINE_STEP_Y(
    NAMESPACE + "/additional_scan_lines/step_y");
const ParameterString<int> RoadClosureClassifier::STEP_DETECTION_REF_FUNC_LENGTH(
    "step_reference_function_length");
const ParameterString<double> RoadClosureClassifier::FEATURE_POINT_DETECTION_THLD(
    "gradient_detection_thld");
const ParameterString<double> RoadClosureClassifier::FILTER_DISTANCE_MIDDLE_LANE(
    NAMESPACE + "/preprocessing/filter_distance_middle_lane");
const ParameterString<double> RoadClosureClassifier::MIN_LATERAL_DISTANCE_TO_MP(
    NAMESPACE + "/model/min_lateral_distance_to_mp");
const ParameterString<double> RoadClosureClassifier::FILTER_DISTANCE_RIGHT_LANE(
    NAMESPACE + "/preprocessing/filter_distance_right_lane");
const ParameterString<double> RoadClosureClassifier::HORIZON(
    NAMESPACE + "/preprocessing/horizon");
const ParameterString<int> RoadClosureClassifier::MIN_SIZE_ADDITIONAL_OUTSIDE(
    NAMESPACE + "/model/min_size_additional_points");

RoadClosureClassifier::RoadClosureClassifier(const common::CameraTransformation *cam_transform,
                                             ParameterInterface *parameter_interface)
    : camera_transform(cam_transform),
      parameter_interface_(parameter_interface),
      line_clusterer_(parameter_interface, NAMESPACE + "/db_scan/inner_lines"),
      refinement_clusterer_(parameter_interface,
                            NAMESPACE + "/db_scan/refinement") {
  parameter_interface->registerParam(RANSAC_MIN_DIST_BOUNDARY);
  parameter_interface->registerParam(RANSAC_ALLOWED_ANGLE_DEVIATION);
  parameter_interface->registerParam(RANSAC_MAX_DIST_INNER);
  parameter_interface->registerParam(RANSAC_MIN_SET_SIZE);
  parameter_interface->registerParam(RANSAC_NR_LINES);
  parameter_interface->registerParam(FRONTLINE_ANGLE);
  parameter_interface->registerParam(INNER_LINE_ANGLE);
  parameter_interface->registerParam(MINIMAL_AREA_SIZE_X);
  parameter_interface->registerParam(MIN_SIZE_LINES);
  parameter_interface->registerParam(ADDITIONAL_SCAN_LINE_WINDOW_WIDTH);
  parameter_interface->registerParam(ADDITIONAL_SCAN_LINE_STEP_Y);
  parameter_interface->registerParam(FILTER_DISTANCE_MIDDLE_LANE);
  parameter_interface->registerParam(MIN_LATERAL_DISTANCE_TO_MP);
  parameter_interface->registerParam(FILTER_DISTANCE_RIGHT_LANE);
  parameter_interface->registerParam(MIN_SIZE_ADDITIONAL_OUTSIDE);
  parameter_interface->registerParam(HORIZON);
}

inline double angleTo(const RCLine &first, const RCLine &second) {
  return common::getAngle(first.line_.direction(),
                          common::ensureSameOrientation(
                              second.line_.direction(), first.line_.direction()));
}

inline bool lanesAroundCluster(const Features &features, const VehiclePoints &boundary) {
  const auto projectionOnX = [](const auto &p) {
    return p.dot(VehiclePoint::UnitX());
  };

  if (features.points_left.empty() && features.points_right.empty()) {
    return false;
  }

  const auto minmax_x_point =
      common::minmax_element(common::join(features.points_left, features.points_right) |
                             transformed(std::cref(projectionOnX)));

  const VehiclePoint boundary_center = common::mean(boundary);

  return *minmax_x_point.first < boundary_center.dot(VehiclePoint::UnitX());
}

inline auto toPolygonPoint() {
  return [](const auto &p) { return PolygonPoint(p.x, p.y); };
}

inline PolygonPoints contourToPolygonPoints(const std::vector<cv::Point2f> &contour) {
  if (contour.empty()) {
    return {};
  }
  PolygonPoints polygon_points;
  polygon_points.reserve(contour.size());
  boost::transform(contour, std::back_inserter(polygon_points), toPolygonPoint());
  //  polygon_points.emplace_back(contour.front().x, contour.front().y);
  return polygon_points;
}

inline Polygon createPolygon(const std::vector<cv::Point2f> &contour) {

  Polygon polygon;
  assign_points(polygon, contourToPolygonPoints(contour));
  correct(polygon);

  return polygon;
}

RoadObjects RoadClosureClassifier::classify(const Features &features) {
  const auto arc_length = computeArcLength(features);
  const auto horizon = parameter_interface_->getParam(HORIZON);

  if (arc_length > horizon) {
    ROS_DEBUG(
        "road_closure_classifier: min_distance to cluster is %f; too far away!", arc_length);
    return returnRoadClosure(features, boost::none, 0.0);
  }

  // filter feature_points: only take the ones on right lane
  const auto isOnRightPart = [&features](const auto &p) {
    return (utils::findLotfusspunkt(features.middle_lane_polynomial, p) - p).y() > 0;
  };

  // remove lane detection points that are present in the cluster
  //  const auto feature_points_wo_ldp =
  //  removeMiddleLanePointsInCluster(features);

  //  if (feature_points_wo_ldp.empty()) {
  //    ROS_DEBUG(
  //        "road_closure_classifier: no feature points left after filtering "
  //        "of lane detection points!");
  //    return returnRoadClosure(features, boost::none, 0);
  //  }

  const FeaturePointCluster right_lane_cluster(
      *camera_transform,
      common::make_eigen_vector(features.cluster.feature_points_vehicle |
                                filtered(isOnRightPart)));

  // refine cluster and take the ones withthe most elements
  const auto refined_clusters = refinement_clusterer_.cluster(right_lane_cluster);

  if (refined_clusters.empty()) {
    ROS_DEBUG(
        "road_closure_classifier: no clusters found on right part of lane!");
    return returnRoadClosure(features, boost::none, 0);
  }

  const auto dominant_cluster = *common::max_score(refined_clusters, [](const auto &c) {
    return c.feature_points_vehicle.size();
  });

  // get feature points in vehicle frame
  const auto feature_points_vehicle =
      searchAdditionalFeaturePoints(dominant_cluster, features);

  if (feature_points_vehicle.empty()) {
    ROS_DEBUG(
        "road_closure_classifier returning zero because additional cluster is "
        "empty!");
    return returnRoadClosure(features, boost::none, 0);
  }

  const auto fused = common::make_eigen_vector(common::join(
      feature_points_vehicle, dominant_cluster.feature_points_vehicle));

  const FeaturePointCluster complete_cluster{*camera_transform, fused};

  // split cluster into boundary and inner points
  const auto cluster_split = splitCluster(complete_cluster);

  if (!lanesAroundCluster(features, cluster_split.first) || cluster_split.first.empty()) {
    ROS_DEBUG("road_closure_classifier: lanes are to far away!");

    return returnRoadClosure(features, boost::none, 0);
  }

  if (!isRoadClosurePlausible(cluster_split.first, features)) {
    ROS_DEBUG(
        "road_closure_classifier: Cluster structure is not plausible to be "
        "road_closure, returning zero.");

    return returnRoadClosure(features, boost::none, 0);
  }

  // extract convex hull around cluster
  const auto boundary_line = searchBoundary(cluster_split.first, features);

  // create additional scan lines in order to be able to obtain more feature
  // points and to check if there's something in between the cluster and the
  // right lane

  if (!boundary_line) {
    ROS_DEBUG(
        "road_closure_classifier: no boundary line detcted in current "
        "cluster!");
    return returnRoadClosure(features, boost::none, 0);
  }

  boundary_line->align(Eigen::Vector2d::UnitX());
  // extract cluster in the middle of the road closure
  const auto inner_lines =
      extractInnerLines(line_clusterer_.cluster(FeaturePointCluster(
                            *camera_transform, cluster_split.second)),
                        boundary_line.get());

  const auto min_size_lines =
      static_cast<std::size_t>(parameter_interface_->getParam(MIN_SIZE_LINES));

  ROS_DEBUG("road_closure_classifier found %zu lines that fit for the model.",
            inner_lines.size());

  if (inner_lines.size() <= min_size_lines) {
    ROS_DEBUG("road_closure_classifier: size of lines is not large enough!");
    return returnRoadClosure(features, boundary_line, 0);
  }

  // compute score
  const auto expected_angle = parameter_interface_->getParam(INNER_LINE_ANGLE);

  const auto getDifferenceAngle = [&boundary_line, &expected_angle](const auto &l) {
    return std::fabs(
        common::getAngle(common::ensureSameOrientation(
                             l.line_.direction(), boundary_line->line_.direction()),
                         boundary_line->line_.direction()) -
        expected_angle);
  };

  const auto angle_dev = common::mean(inner_lines | transformed(getDifferenceAngle));

  ROS_DEBUG("road_closure_classifier: angle_dev is %f!", angle_dev);

  // check consistency of inner line and compute score
  const auto minimal_length_inner_line = parameter_interface_->getParam(MINIMAL_AREA_SIZE_X);

  const double score_multiplier = boost::algorithm::clamp(
      (boundary_line->start_ - boundary_line->end_).norm() / minimal_length_inner_line, 0.0, 1.0);

  const auto score =
      score_multiplier *
      std::pow(1 - boost::algorithm::clamp(angle_dev, 0, expected_angle) / expected_angle, 2);

  ROS_DEBUG("road_closure_classifier: score is %f!", score);

  return returnRoadClosure(features, boundary_line, score);
}

size_t RoadClosureClassifier::getClassifierId() const {
  return typeid(RoadClosureClassifier).hash_code();
}

inline auto modelSupport(const LineEquation &model, const double min_dist) {
  return [&model, min_dist](const auto &p) {
    return std::fabs(model.distance(to2D(p))) < min_dist;
  };
};

std::pair<VehiclePoints, VehiclePoints> RoadClosureClassifier::splitCluster(
    const FeaturePointCluster &refined_cluster) const {

  const auto cvToGround = [this](const auto &p) {
    return camera_transform->transformImageToGround(cvPointToImagePoint(p));
  };

  std::vector<cv::Point> boundary;
  cv::convexHull(toCV(refined_cluster.feature_points_img), boundary);

  const auto hull_vec = common::make_eigen_vector(boundary | transformed(cvToGround));
  auto fp_vec = refined_cluster.feature_points_vehicle;

  const auto max_dev = parameter_interface_->getParam(RANSAC_MIN_DIST_BOUNDARY);

  VehiclePoints boundary_points, tmp;
  for (auto it = hull_vec.begin(); it != std::prev(hull_vec.end()); it++) {
    // fit line through
    const LineEquation l = LineEquation::Through(to2D(*it), to2D(*std::next(it)));

    // find all points supporting the model
    const auto div = boost::partition(fp_vec, modelSupport(l, max_dev));
    VehiclePoints supporters(fp_vec.begin(), div);

    // remove points after pushback to avoid duplicates
    boost::push_back(boundary_points, supporters);
    tmp.clear();
    tmp.insert(tmp.begin(), div, fp_vec.end());

    if (tmp.empty()) {
      break;
    }
    fp_vec = tmp;
  }

  // return the boundary as first part of pair and the complete cluster as the
  // second part
  return make_pair(boundary_points,
                   common::make_eigen_vector(common::join(boundary_points, fp_vec)));
}

inline RCLine leastSquaresFit(const VehiclePoints &points) {
  Eigen::Vector2d direction =
      common::getPrincipalComponent(common::toMatrix2D(points)).normalized();

  if (direction.dot(Eigen::Vector2d::UnitX()) < 1.0) {
    direction = common::ensureSameOrientation(direction, Eigen::Vector2d::UnitX());
  }

  const Eigen::Vector2d origin = to2D(common::mean<VehiclePoint>(points));

  const auto projectOntoDirection = [&direction](const auto &p) {
    return to2D(p).dot(direction);
  };

  const auto start_end = common::minmax_score(points, projectOntoDirection);

  return {
      LineEquation{origin, direction}, to2D(*start_end.first), to2D(*start_end.second)};
}

RCLines RoadClosureClassifier::fitLinesR(const VehiclePoints &points,
                                         const Eigen::Vector2d &prior_direction,
                                         const double min_dist,
                                         const double angle_eps,
                                         const std::size_t min_set_size,
                                         const std::size_t nr_lines) const {
  RCLines lines;
  lines.reserve(nr_lines);
  // use observations
  auto detected_points = points;
  const std::size_t max_it = detected_points.size() / 2;
  VehiclePoints data_next_it;

  // loop in order to detect the specified number of lines
  for (std::size_t i = 0; i < nr_lines; i++) {

    std::size_t best_set_size = 0;
    boost::optional<RCLine> best_model = boost::none;

    // ransac loop
    for (std::size_t it = 0; it < max_it; it++) {
      boost::random_shuffle(detected_points);
      const LineEquation model = LineEquation::Through(
          to2D(detected_points.front()), to2D(detected_points.back()));
      // additional angle criterium
      const auto angle_dev = common::getAngle(
          common::ensureSameOrientation(model.direction(), Eigen::Vector2d::UnitX()),
          common::ensureSameOrientation(prior_direction, Eigen::Vector2d::UnitX()));
      const auto divider =
          boost::partition(detected_points, modelSupport(model, min_dist));
      VehiclePoints consensus_set(detected_points.begin(), divider);
      const auto model_size = consensus_set.size();
      if (model_size > best_set_size && model_size > min_set_size &&
          std::fabs(angle_dev) < angle_eps) {
        //  least squares fit for best model
        best_model = leastSquaresFit(consensus_set);
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

boost::optional<RCLine> RoadClosureClassifier::searchBoundary(const VehiclePoints &road_closure_boundary,
                                                              const Features &features) const {

  const auto max_dist = parameter_interface_->getParam(RANSAC_MIN_DIST_BOUNDARY);
  const auto angle_eps = parameter_interface_->getParam(RANSAC_ALLOWED_ANGLE_DEVIATION);

  const VehiclePoint search_direction =
      (Eigen::AngleAxisd{features.cluster_center_lane_orientation, VehiclePoint::UnitZ()} *
       VehiclePoint::UnitX())
          .normalized();

  // in the boundary of the road_closure there are two lines in road_closure
  // direction expected

  const auto outer_lines = fitLinesR(
      road_closure_boundary, to2D(search_direction), max_dist, angle_eps, 6, 2);

  const auto projectionYCoordinate = [&features](const auto &l) {
    return l.line_.projection(to2D(features.cluster_center3d)).y();
  };

  // the line with the maximum y-coordinate of the projection of its foot point
  // of the cluster center is the inner boundary line
  const auto boundary_line = *common::max_score(outer_lines, projectionYCoordinate);

  return boost::make_optional(!outer_lines.empty(), boundary_line);
}

RCLines RoadClosureClassifier::extractInnerLines(const std::vector<FeaturePointCluster> &inner_clusters,
                                                 const RCLine &bounding_line) const {
  RCLines inner_lines;

  const auto min_set_size =
      static_cast<std::size_t>(parameter_interface_->getParam(RANSAC_MIN_SET_SIZE));
  const auto distance_thld = parameter_interface_->getParam(RANSAC_MAX_DIST_INNER);
  const auto expected_nr_lines =
      static_cast<std::size_t>(parameter_interface_->getParam(RANSAC_NR_LINES));

  for (const auto &cluster : inner_clusters) {
    boost::push_back(inner_lines,
                     fitLinesR(cluster.feature_points_vehicle,
                               Eigen::Vector2d::UnitX(),
                               distance_thld,
                               M_PI,
                               min_set_size,
                               expected_nr_lines));
  }

  const auto angle_dev = parameter_interface_->getParam(RANSAC_ALLOWED_ANGLE_DEVIATION);
  const auto bound_angle = parameter_interface_->getParam(FRONTLINE_ANGLE);

  const auto isAngleCloseToBoundary =
      [&bounding_line, &angle_dev, &bound_angle](const auto &rcl) {
        return angleTo(bounding_line, rcl) < 0 &&
               std::fabs(angleTo(bounding_line, rcl)) > angle_dev &&
               std::fabs(angleTo(bounding_line, rcl)) < (bound_angle - angle_dev);
      };

  return common::make_vector(inner_lines | filtered(isAngleCloseToBoundary));
}

VehiclePoints RoadClosureClassifier::localizeRoadClosure(
    const Features &features, const boost::optional<RCLine> &inner_boundary) const {
  // no bounding line detected --> create dummy
  if (!inner_boundary) {
    const auto changeToCV = [](const auto &p) { return toCV(to2D(p)); };
    const auto changeToEigen = [](const auto &p) { return to3D(toEigen(p)); };

    std::vector<cv::Point2f> cv_points;
    cv::convexHull(common::make_vector(features.cluster.feature_points_vehicle |
                                       transformed(changeToCV)),
                   cv_points);

    return common::make_eigen_vector(cv_points | transformed(changeToEigen));
  }

  const IntersectionLine inner =
      IntersectionLine::Through(inner_boundary->start_, inner_boundary->end_);

  const auto angle_from_boundary = parameter_interface_->getParam(FRONTLINE_ANGLE);

  const auto angle_x_to_inner = common::getAngle(
      common::ensureSameOrientation(inner_boundary->line_.direction(),
                                    Eigen::Vector2d::UnitX()),
      Eigen::Vector2d::UnitX());

  const auto angle_x_to_front = angle_x_to_inner + angle_from_boundary;
  const auto angle_x_to_rear = angle_x_to_front - M_PI_2;

  // both start end end points constructed by heuristic

  const auto guessed_width = 0.45;

  const Eigen::Vector2d center = (inner_boundary->start_ + inner_boundary->end_) / 2;

  const auto normal_distance =
      guessed_width -
      (utils::findLotfusspunkt(features.middle_lane_polynomial, center) - center)
          .norm();

  const auto line_length = normal_distance / std::sin(angle_from_boundary);

  const VehiclePoint start_front =
      to3D(inner_boundary->start_) -
      line_length * (Eigen::AngleAxisd{angle_x_to_front, VehiclePoint::UnitZ()} *
                     VehiclePoint::UnitX());
  const VehiclePoint end_rear =
      to3D(inner_boundary->end_) +
      line_length * (Eigen::AngleAxisd{angle_x_to_rear, VehiclePoint::UnitZ()} *
                     VehiclePoint::UnitX());

  return {{start_front, to3D(inner_boundary->start_), to3D(inner_boundary->end_), end_rear}};
}

RoadObjects RoadClosureClassifier::returnRoadClosure(const Features &features,
                                                     const boost::optional<RCLine> &inner_boundary,
                                                     const double score) const {

  RoadObjects road_closure;

  const auto base_area = localizeRoadClosure(features, inner_boundary);

  const VehiclePose pose = Eigen::Translation3d{common::mean(base_area)} *
                           Eigen::AngleAxisd{features.cluster_center_lane_orientation,
                                             VehiclePoint::UnitZ()};

  road_closure.push_back(
      std::make_unique<RoadClosure>(features.timestamp, score, pose, base_area));
  return road_closure;
}

VehiclePoints RoadClosureClassifier::searchAdditionalFeaturePoints(
    const FeaturePointCluster &dominant_cluster, const Features &features) const {

  // create ScanLines
  const auto additional_scan_lines = createAdditionalScanlines(dominant_cluster, features);

  const auto gradient_threshold =
      static_cast<float>(parameter_interface_->getParam(FEATURE_POINT_DETECTION_THLD));
  const auto ref_func_length = static_cast<std::size_t>(
      parameter_interface_->getParam(STEP_DETECTION_REF_FUNC_LENGTH));
  const auto ref_func = step_detection::createReferenceFunction(ref_func_length);

  const auto toGround = [this](const auto &p) {
    return camera_transform->transformImageToGround(p);
  };

  VehiclePoints additional_feature_points;
  for (const auto &scan_line : additional_scan_lines) {
    boost::push_back(
        additional_feature_points,
        step_detection::detectStep(
            features.image_complete, scan_line, ref_func, gradient_threshold, true) |
            transformed(toGround));
  }

  const auto poly_deg = static_cast<common::PolynomialDegree>(
      features.middle_lane_polynomial.getCoefficients().size() - 1);

  const double thresh = parameter_interface_->getParam(FILTER_DISTANCE_RIGHT_LANE);

  const VehiclePoints additional_wo_right_points =
      features.points_right.size() > static_cast<std::size_t>(poly_deg)
          ? removeClosePointsPolynomial(
                features.points_right, additional_feature_points, thresh, poly_deg)
          : removeClosePoints(features.points_right, additional_feature_points, thresh);

  //  const VehiclePoints further_reduced =
  //      removePointsLeftOfCenter(additional_wo_right_points, features);

  if (enoughPointsOutside(additional_wo_right_points, features.cluster.feature_points_vehicle)) {
    return additional_wo_right_points;
  } else {
    ROS_DEBUG(
        "road_closure_classifier: Newly detected feature points are not "
        "large enough");
    return {};
  }
}
ScanLines RoadClosureClassifier::createAdditionalScanlines(const FeaturePointCluster &dominant_cluster,
                                                           const Features &features) const {

  const Eigen::Vector3d direction_longitudinal = common::ensureSameOrientation(
      (Eigen::AngleAxisd{features.cluster_center_lane_orientation, Eigen::Vector3d::UnitZ()} *
       VehiclePoint::UnitX()),
      VehiclePoint::UnitX());

  const Eigen::Vector3d direction_lateral = common::ensureSameOrientation(
      direction_longitudinal.unitOrthogonal(), -VehiclePoint::UnitY());

  // get points with minimum and maximum x-coordinate in cluster
  const auto minmax_points_x = common::minmax_score(
      dominant_cluster.feature_points_vehicle, common::x_value());

  // get mean of y coordinates
  const auto cluster_mean =
      common::mean<VehiclePoint>(dominant_cluster.feature_points_vehicle);

  // get cluster point with minimum y-coordinate
  const auto scan_line_window_offset =
      parameter_interface_->getParam(ADDITIONAL_SCAN_LINE_WINDOW_WIDTH);
  const VehiclePoint cluster_outer_bound_point =
      *common::min_score(dominant_cluster.feature_points_vehicle, common::y_value()) +
      scan_line_window_offset * direction_lateral;

  // create boundary lines
  const IntersectionLine innerbound{to2D(direction_lateral), to2D(cluster_mean)};
  const IntersectionLine backbound{to2D(direction_longitudinal),
                                   to2D(*minmax_points_x.second)};
  const IntersectionLine outerbound{to2D(direction_lateral), to2D(cluster_outer_bound_point)};
  const IntersectionLine frontbound{to2D(direction_longitudinal),
                                    to2D(*minmax_points_x.first)};

  // intersection points of boundary lines are start and end points for
  // additional scan lines
  const VehiclePoint bottom_left = to3D(innerbound.intersection(frontbound));
  const VehiclePoint top_left = to3D(innerbound.intersection(backbound));
  const VehiclePoint top_right = to3D(outerbound.intersection(backbound));
  const VehiclePoint bottom_right = to3D(outerbound.intersection(frontbound));

  // define scan line step and calculate number of scan lines that is
  // resulting
  const auto scan_line_step_ref =
      parameter_interface_->getParam(ADDITIONAL_SCAN_LINE_STEP_Y);

  // define parameters for scan line grid
  const auto n_scan_lines_front = static_cast<std::size_t>(std::floor(
      std::fabs((bottom_right - bottom_left).dot(direction_lateral)) / scan_line_step_ref));
  const auto n_scan_lines_inner = static_cast<std::size_t>(std::floor(
      std::fabs((top_left - bottom_left).dot(direction_longitudinal)) / scan_line_step_ref));

  const double scan_line_step_back =
      std::fabs((top_right - top_left).dot(direction_lateral)) / n_scan_lines_front;
  const double scan_line_step_outer =
      std::fabs((top_right - bottom_right).dot(direction_longitudinal)) / n_scan_lines_inner;

  // create scan lines
  ScanLines additional_scan_lines;
  additional_scan_lines.reserve(n_scan_lines_front + n_scan_lines_inner);

  // longitudinal scan lines
  for (std::size_t i = 0; i < n_scan_lines_front; i++) {
    // create points
    additional_scan_lines.push_back(transformGroundToImage(
        camera_transform,
        VehicleScanLine{bottom_left + i * scan_line_step_ref * direction_lateral,
                        top_left + i * scan_line_step_back * direction_lateral}));
  }

  // lateral scan_lines
  for (std::size_t i = 0; i < n_scan_lines_inner; i++) {
    // create scan line until lane polynomial an then trim it in order not to
    // touch the right lane
    additional_scan_lines.push_back(transformGroundToImage(
        camera_transform,
        VehicleScanLine{bottom_left + i * scan_line_step_ref * direction_longitudinal,
                        bottom_right + i * scan_line_step_outer * direction_longitudinal}));
  }

  return additional_scan_lines;
}

VehiclePoints RoadClosureClassifier::removeMiddleLanePointsInCluster(const Features &features) const {

  const auto filter_threshold_distance =
      parameter_interface_->getParam(FILTER_DISTANCE_MIDDLE_LANE);

  // filter function returns false if the observed point is too close to the
  // middle lane polynomial
  const auto isTooCloseOnMiddleLane = [&features, &filter_threshold_distance](
                                          const VehiclePoint &p) {
    const auto distance =
        (utils::findLotfusspunkt(features.middle_lane_polynomial, p) - p).norm();
    ROS_DEBUG("distance to middle_lane_polynom is %f", distance);
    return distance > filter_threshold_distance;
  };

  return common::make_eigen_vector(features.cluster.feature_points_vehicle |
                                   filtered(isTooCloseOnMiddleLane));
}

VehiclePoints RoadClosureClassifier::removeClosePoints(const VehiclePoints &ref_points,
                                                       const VehiclePoints &cluster_points,
                                                       const double thresh) const {

  // crazy filter function but effecive =)

  const auto filterLanePoints = [&ref_points, thresh](const auto &p) {
    return boost::algorithm::all_of(
        ref_points | transformed(common::distanceTo(p)),
        [thresh](const auto &d) { return d > thresh; });
  };

  return common::make_eigen_vector(cluster_points | filtered(filterLanePoints));
}

VehiclePoints RoadClosureClassifier::removeClosePointsPolynomial(
    const VehiclePoints &ref_points,
    const VehiclePoints &cluster_points,
    const double thresh,
    const common::PolynomialDegree poly_deg) const {
  const auto polynom = common::fitToPoints(ref_points, poly_deg);

  const auto isTooNear = [thresh, &polynom](const auto &p) {
    return (p - utils::findLotfusspunkt(polynom, p)).norm() > thresh;
  };

  return common::make_eigen_vector(cluster_points | filtered(isTooNear));
}

bool RoadClosureClassifier::isRoadClosurePlausible(const VehiclePoints &complete_points,
                                                   const Features &features) const {

  if (features.points_right.empty()) {
    return true;
  }

  const auto distanceToFittedMiddleLane = [&features](const auto &p) {
    return (utils::findLotfusspunkt(features.middle_lane_polynomial, p) - p).norm();
  };

  const auto minimal_lane_width = 0.34;
  const auto current_min_lane_width = *boost::min_element(
      features.points_right | transformed(std::cref(distanceToFittedMiddleLane)));

  const auto minimal_lateral_cluster_width =
      current_min_lane_width >= minimal_lane_width
          ? parameter_interface_->getParam(MIN_LATERAL_DISTANCE_TO_MP)
          : parameter_interface_->getParam(MIN_LATERAL_DISTANCE_TO_MP) / 2;

  const auto footPointDistance = [&features](const auto &p) {
    return (utils::findLotfusspunkt(features.middle_lane_polynomial, p) - p).norm();
  };

  // trimmed maximum in order to filter out outlayers
  auto distances =
      common::make_vector(complete_points | transformed(std::cref(footPointDistance)));

  // sort
  boost::nth_element(distances, distances.begin() + 5, std::greater<>());
  ROS_DEBUG(
      "road_closure_classifier: fifth biggest distance in footpointdistances "
      "is %f",
      distances[5]);

  return distances[5] > minimal_lateral_cluster_width;
}

bool RoadClosureClassifier::enoughPointsOutside(const VehiclePoints &targets,
                                                const VehiclePoints &reference) const {

  // there must be at least 3 points in order to be able to define a polygon
  if (targets.size() < 3) {
    ROS_DEBUG(
        "road_closure_classifier: not enough additional feature points after "
        "removal of ldp !");
    return {};
  }

  // toCV points
  const auto vehicleToCv2f = [](const auto &p) {
    return imagePointExactToCvPoint2f(to2D(p));
  };

  const auto boostGeoToCv2f = [](const auto &p) {
    return cv::Point2f(p.x(), p.y());
  };

  // create 2D opencv representation of target vehicle points
  auto targets_cv = common::make_vector(targets | transformed(vehicleToCv2f));

  // compute convex hulls
  std::vector<cv::Point2f> target_hull, reference_hull;
  cv::convexHull(targets_cv, target_hull);
  cv::convexHull(common::make_vector(reference | transformed(vehicleToCv2f)), reference_hull);

  // create intersection of hulls
  const auto target_polygon = createPolygon(target_hull);
  const auto ref_polygon = createPolygon(reference_hull);

  if (!intersects(target_polygon, ref_polygon)) {
    ROS_DEBUG(
        "road_closure_classifier: no intersection for polygons-->STRANGE!");
    return {};
  }

  // compute intersection if there any
  std::vector<Polygon> intersection_polygon;

  try {
    intersection(target_polygon, ref_polygon, intersection_polygon);
  } catch (const overlay_invalid_input_exception &ex) {
    ROS_WARN_STREAM_THROTTLE(1.0, ex.what());
    return {};
  }

  // intersection is not intended to consist of more than one polygon
  if (intersection_polygon.size() != 1) {
    ROS_DEBUG(
        "road_closure_classifier: more than one nested contour of "
        "polygons or "
        "no contours--> STRANGE!");
    return {};
  }

  // back to opencv
  const auto cv_intersection = common::make_vector(
      intersection_polygon.front().outer() | transformed(boostGeoToCv2f));

  const auto checkIfIsOutside = [&cv_intersection](const auto &p) {
    return cv::pointPolygonTest(cv_intersection, p, true) < -0.02;
  };

  const auto part_pt = boost::partition(targets_cv, checkIfIsOutside);
  const auto min_size_outside = parameter_interface_->getParam(MIN_SIZE_ADDITIONAL_OUTSIDE);

  return std::distance(targets_cv.begin(), part_pt) >= min_size_outside;
}

double RoadClosureClassifier::computeArcLength(const Features &features) const {
  // compute nearest footpoint
  const auto toFoot = [&features](const auto &p) {
    return utils::findLotfusspunkt(features.middle_lane_polynomial, p);
  };
  const auto minmax_foot = common::min_score(
      features.cluster.feature_points_vehicle | transformed(std::cref(toFoot)),
      [](const auto &p) { return p.norm(); });

  // discretisize polynomial in order to approximate arc length
  const common::DiscretizationParams params{0.0, std::max(0.0, minmax_foot->x()), 0.02};
  VehiclePoints disc_points;
  common::discretisize(features.middle_lane_polynomial, params, &disc_points);

  if (disc_points.size() < 2) {
    return std::numeric_limits<double>::max();
  }

  auto arc_length = 0.0;
  for (auto it = std::next(disc_points.begin()); it != disc_points.end(); it++) {
    arc_length += (*it - *std::prev(it)).norm();
  }

  return arc_length;
}

// VehiclePoints RoadClosureClassifier::removePointsLeftOfCenter(
//    const VehiclePoints &points, const Features &features) const {
//  // remove points left of cluster center with aid of the normal to the middle
//  // lane polynom which serves as projecion direction
//  const Eigen::Vector3d projection_direction = common::ensureSameOrientation(
//      (Eigen::AngleAxisd{features.cluster_center_lane_orientation,
//                         VehiclePoint::UnitZ()} *
//       VehiclePoint::UnitX())
//          .unitOrthogonal(),
//      Eigen::Vector3d::UnitY());

//  const auto isRightOfCenter = [&features,
//                                &projection_direction](const auto &p) {
//    return (features.cluster_center3d - p).dot(projection_direction) >= 0;
//  };

//  return common::make_eigen_vector(points | filtered(isRightOfCenter));
//}

}  // namespace road_object_detection
