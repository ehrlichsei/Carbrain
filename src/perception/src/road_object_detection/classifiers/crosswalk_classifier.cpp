#include "crosswalk_classifier.h"

THIRD_PARTY_HEADERS_BEGIN
#include <ros/console.h>
#include <boost/algorithm/clamp.hpp>
#include <boost/algorithm/cxx11/all_of.hpp>
#include <boost/algorithm/cxx11/any_of.hpp>
#include <boost/range/adaptor/filtered.hpp>
#include <boost/range/adaptor/transformed.hpp>
#include <boost/range/algorithm.hpp>
#include <boost/range/algorithm/min_element.hpp>
#include <boost/range/algorithm_ext/erase.hpp>
#include <cmath>
#include <memory>
#include <opencv2/imgproc.hpp>
THIRD_PARTY_HEADERS_END

#include "../../utils/foot_finder.h"
#include "common/angle_conversions.h"
#include "common/basic_statistics_eigen.h"
#include "common/best_score.h"
#include "common/console_colors.h"
#include "common/discretisize.h"
#include "common/eigen_adaptors.h"
#include "common/eigen_functors.h"
#include "common/eigen_utils.h"
#include "common/make_vector.h"
#include "common/minmax_element.h"
#include "common/pca_eigen.h"
#include "common/polynomialfit.h"

#include "../road_objects/road_object.h"
#include "opencv_eigen_conversions.h"

namespace road_object_detection {

const std::string CrosswalkClassifier::NAMESPACE("crosswalk_classifier");
const ParameterString<int> CrosswalkClassifier::WHITE_PIXEL(NAMESPACE +
                                                            "/white_pixel");
const ParameterString<int> CrosswalkClassifier::BLACK_PIXEL(NAMESPACE +
                                                            "/black_pixel");
const ParameterString<int> CrosswalkClassifier::LINE_OFFSET(NAMESPACE +
                                                            "/line_offset");
const ParameterString<int> CrosswalkClassifier::NR_LINES(NAMESPACE +
                                                         "/nr_lines");
const ParameterString<int> CrosswalkClassifier::BOTTOM_LINE_ROW(
    NAMESPACE + "/bottom_line_row");
const ParameterString<int> CrosswalkClassifier::CANNY_THRESH_LOW(
    NAMESPACE + "/canny_thresh_low");
const ParameterString<int> CrosswalkClassifier::CANNY_THRESH_HIGH(
    NAMESPACE + "/canny_thresh_high");
const ParameterString<int> CrosswalkClassifier::CANNY_MASK_SIZE(
    NAMESPACE + "/canny_mask_size");
const ParameterString<int> CrosswalkClassifier::MIN_SIZE_POINT_VECTORS(
    NAMESPACE + "/min_size_point_vectors");
const ParameterString<double> CrosswalkClassifier::GUESSED_LANE_WIDTH(
    NAMESPACE + "/guessed_lane_width");
const ParameterString<double> CrosswalkClassifier::STEP_LOC_POINT(
    NAMESPACE + "/step_loc_point");
const ParameterString<int> CrosswalkClassifier::OFFSET_RIGHT_LEFT_BOUND(
    NAMESPACE + "/offset_right_left_bound");
const ParameterString<double> CrosswalkClassifier::MAX_VARIANCE(
    NAMESPACE + "/max_variance");
const ParameterString<double> CrosswalkClassifier::MIN_SCORE(NAMESPACE +
                                                             "/min_score");
const ParameterString<double> CrosswalkClassifier::MIN_PROJECTION_ON_LANE(
    NAMESPACE + "/min_projection_on_lane");
const ParameterString<double> CrosswalkClassifier::CROSSWALK_LENGTH(
    NAMESPACE + "/crosswalk_length");

const ParameterString<double> CrosswalkClassifier::DISC_X_STEP(
    NAMESPACE + "/sampling_x_step");
const ParameterString<double> CrosswalkClassifier::MAX_LANE_WIDTH(
    NAMESPACE + "/max_lane_width");
const ParameterString<double> CrosswalkClassifier::MIN_LANE_WIDTH(
    NAMESPACE + "/min_lane_width");
const ParameterString<double> CrosswalkClassifier::MIN_CLUSTER_SPREAD_NORMAL(
    NAMESPACE + "/min_cluster_spread_normal");

using boost::adaptors::filtered;
using boost::adaptors::transformed;

CrosswalkClassifier::CrosswalkClassifier(const common::CameraTransformation *cam_transform,
                                         ParameterInterface *parameter_interface)
    : camera_transform(cam_transform), parameter_interface_(parameter_interface) {
  parameter_interface->registerParam(WHITE_PIXEL);
  parameter_interface->registerParam(BLACK_PIXEL);
  parameter_interface->registerParam(LINE_OFFSET);
  parameter_interface->registerParam(BOTTOM_LINE_ROW);
  parameter_interface->registerParam(NR_LINES);
  parameter_interface->registerParam(CANNY_THRESH_LOW);
  parameter_interface->registerParam(CANNY_THRESH_HIGH);
  parameter_interface->registerParam(CANNY_MASK_SIZE);
  parameter_interface->registerParam(MIN_SIZE_POINT_VECTORS);
  parameter_interface->registerParam(GUESSED_LANE_WIDTH);
  parameter_interface->registerParam(STEP_LOC_POINT);
  parameter_interface->registerParam(OFFSET_RIGHT_LEFT_BOUND);
  parameter_interface->registerParam(MAX_VARIANCE);
  parameter_interface->registerParam(MIN_SCORE);
  parameter_interface->registerParam(MIN_PROJECTION_ON_LANE);
  parameter_interface->registerParam(CROSSWALK_LENGTH);

  parameter_interface->registerParam(DISC_X_STEP);
  parameter_interface->registerParam(MAX_LANE_WIDTH);
  parameter_interface->registerParam(MIN_LANE_WIDTH);
  parameter_interface->registerParam(MIN_CLUSTER_SPREAD_NORMAL);
}

inline double getUVariance(const CVPoints &points) {
  std::vector<double> distances;
  distances.reserve(points.size());
  for (auto it = points.begin() + 1; it != points.end(); it++) {
    distances.push_back(std::fabs<double>(it->x - (it - 1)->x));
  }
  if (distances.size() < 2) {
    return std::numeric_limits<double>::max();
  }
  return common::variance(distances, common::mean(distances));
}

RoadObjects CrosswalkClassifier::classify(const Features &features) {
  const auto minmax = common::minmax_score(features.cluster.feature_points_vehicle,
                                           [](const auto &a) { return a.x(); });
  closest_point = *minmax.first;
  farest_point = *minmax.second;

  const auto bottom_row = parameter_interface_->getParam(BOTTOM_LINE_ROW);
  const auto line_offset = parameter_interface_->getParam(LINE_OFFSET);
  const auto nr_lines =
      static_cast<std::size_t>(parameter_interface_->getParam(NR_LINES));

  std::vector<CVPoints> crosswalk_lines;
  crosswalk_lines.reserve(nr_lines);

  for (std::size_t i = 0; i < nr_lines; i++) {
    crosswalk_lines.push_back(
        linePoints(features, bottom_row + static_cast<int>(i) * line_offset));
  }

  const auto min_size =
      assumptPedestrianIsland(features)
          ? static_cast<std::size_t>(parameter_interface_->getParam(MIN_SIZE_POINT_VECTORS)) / 2
          : static_cast<std::size_t>(parameter_interface_->getParam(MIN_SIZE_POINT_VECTORS));
  assert(min_size > 4 &&
         "min_size_point_vectors-parameter of crosswalk "
         "classifier is too small. Check parameter settings!");
  const auto getSize = [](const auto &v) { return v.size(); };

  RoadObjects crosswalk;

  if ((*boost::min_element(crosswalk_lines | transformed(std::cref(getSize))) <= min_size) ||
      isTooSmall(features)) {
    ROS_DEBUG(
        "crosswalk_classifier returning zero because size of one at "
        "least one element of crosswalk lines is smaller "
        "than minimal value.");
    createCrosswalk(crosswalk, features, crosswalk_lines, 0.0);
    return crosswalk;
  }

  const double max_variance = parameter_interface_->getParam(MAX_VARIANCE);

  const auto smallerThanMaxVar = [&max_variance](const auto &v) {
    return getUVariance(v) < max_variance;
  };

  if (!assumptPedestrianIsland(features) &&
      !boost::algorithm::all_of(crosswalk_lines, smallerThanMaxVar)) {
    ROS_DEBUG(
        "crosswalk_classifier returning zero because variance bigger than "
        "max_variance");
    createCrosswalk(crosswalk, features, crosswalk_lines, 0.0);
    return crosswalk;
  }

  const auto points_inside_lane = onlyPointsOnLane(crosswalk_lines, features);

  if (points_inside_lane.size() < crosswalk_lines.size()) {
    ROS_DEBUG(
        "crosswalk_classifier returning zero because at least one line "
        "is not plausible!");
    createCrosswalk(crosswalk, features, crosswalk_lines, 0.01);
    return crosswalk;
  }

  const auto len =
      *boost::min_element(points_inside_lane | transformed(std::cref(getSize)));

  const auto horizontal =
      points_inside_lane.front().front() - points_inside_lane.front().back();
  const auto hor_norm = cv::norm(horizontal);

  std::vector<double> dots;
  for (auto it = std::next(points_inside_lane.begin()); it != points_inside_lane.end(); it++) {

    for (std::size_t i = 0; i < len; i++) {
      cv::Point2i vertical = (*it)[i] - (*std::prev(it))[i];
      //          cv::Point2i horizontal = (*it)[i] - ref_point;
      const double vert_norm = cv::norm(vertical);

      if (vert_norm * hor_norm <= std::numeric_limits<double>::epsilon()) {
        continue;
      }
      dots.push_back(std::fabs(vertical.ddot(horizontal)) / (vert_norm * hor_norm));
    }
  }

  const auto score = dots.size() > 1 ? std::pow(1.0 - common::mean(dots), 2.0) : 0.0;

  createCrosswalk(crosswalk, features, points_inside_lane, score);

  return crosswalk;
}

size_t CrosswalkClassifier::getClassifierId() const {
  return typeid(CrosswalkClassifier).hash_code();
}

CVPoints CrosswalkClassifier::linePoints(const Features &features, const int row_idx) const {
  const uchar white_pixel =
      static_cast<uchar>(parameter_interface_->getParam(WHITE_PIXEL));
  const uchar black_pixel =
      static_cast<uchar>(parameter_interface_->getParam(BLACK_PIXEL));

  const int thresh_low = parameter_interface_->getParam(CANNY_THRESH_LOW);
  const int thresh_high = parameter_interface_->getParam(CANNY_THRESH_HIGH);
  const int mask_size = parameter_interface_->getParam(CANNY_MASK_SIZE);
  const double guessed_width = parameter_interface_->getParam(GUESSED_LANE_WIDTH);
  cv::Mat birdsview_patch = *features.birdsview_patch.getImage();

  VehiclePoint right_bound = computeBound(features, closest_point, -guessed_width);
  VehiclePoint left_bound = computeBound(features, closest_point, guessed_width);

  ImagePoint right_bound_image = features.birdsview_patch.vehicleToImage(right_bound);
  ImagePoint left_bound_image = features.birdsview_patch.vehicleToImage(left_bound);

  const int offset_right_left = parameter_interface_->getParam(OFFSET_RIGHT_LEFT_BOUND);
  left_bound_image[0] += offset_right_left;
  right_bound_image[0] -= offset_right_left;
  Eigen::Vector2i closest_point_image =
      features.birdsview_patch.vehicleToImage(closest_point);

  cv::Mat point_line;
  std::vector<cv::Point2i> point_coordinates;

  left_bound_image[0] =
      boost::algorithm::clamp(left_bound_image[0], 10, birdsview_patch.cols - 11);
  right_bound_image[0] = boost::algorithm::clamp(
      right_bound_image[0], left_bound_image[0], birdsview_patch.cols - 10);
  closest_point_image[1] = boost::algorithm::clamp(
      closest_point_image[1], 4 + row_idx, birdsview_patch.rows - 3 + row_idx);

  cv::Canny(birdsview_patch(cv::Range(closest_point_image[1] - row_idx - 3,
                                      closest_point_image[1] - row_idx + 2),
                            cv::Range(left_bound_image[0], right_bound_image[0])),
            point_line,
            thresh_low,
            thresh_high,
            mask_size,
            true);
  for (int i = 1; i < point_line.cols - 1; i++) {
    if (point_line.at<uchar>(2, i) >= white_pixel &&
        point_line.at<uchar>(2, i + 1) <= black_pixel) {
      point_coordinates.emplace_back(left_bound_image.x() + i,
                                     closest_point_image.x() - row_idx);
    }
  }
  return point_coordinates;
}

std::pair<common::DynamicPolynomial, common::DynamicPolynomial> CrosswalkClassifier::extractBoundingPolynomials(
    const Features &features) const {
  const auto left_poly =
      assumptPedestrianIsland(features)
          ? features.middle_lane_polynomial
          : (features.points_left.size() >
                     features.middle_lane_polynomial.getCoefficients().size()
                 ? common::fitToPoints(
                       features.points_left,
                       static_cast<common::PolynomialDegree>(
                           features.middle_lane_polynomial.getCoefficients().size() - 1))
                 : common::Polynomial<1, double>(std::vector<double>{0.0, 0.5}));

  const auto right_poly =
      features.points_right.size() >
              features.middle_lane_polynomial.getCoefficients().size()
          ? common::fitToPoints(
                features.points_right,
                static_cast<common::PolynomialDegree>(
                    features.middle_lane_polynomial.getCoefficients().size() - 1))
          : common::Polynomial<1, double>(std::vector<double>{0.0, -0.2});

  return std::make_pair(left_poly, right_poly);
}

VehiclePoint CrosswalkClassifier::computeLanePoint(const VehiclePoints &lane_points,
                                                   const double angle,
                                                   const VehiclePoint &ref_point,
                                                   const VehiclePoint &offset) const {
  const auto trafo = Eigen::AngleAxisd(angle, Eigen::Vector3d::UnitZ()) *
                     Eigen::Translation3d(-offset);
  if (lane_points.empty()) {
    return {0.0, (trafo * ref_point).y(), 0.0};
  }

  return *common::min_score(lane_points | common::eigen_transformed(trafo),
                            [](const auto &a) { return a.norm(); });
}

VehiclePoint CrosswalkClassifier::computeBound(const Features &features,
                                               const VehiclePoint &ref_point,
                                               const double guessed_width) const {
  const VehiclePoints &boundary_points =
      (guessed_width < 0) ? features.points_right : features.points_left;

  if (!boundary_points.empty()) {
    return *common::min_score(boundary_points, common::distanceTo(ref_point));
  }
  // if boundary points are empty, transform to cluster center and add guess
  // for lane width in y-direction.
  const Eigen::Affine3d to_cluster_center{
      Eigen::Translation3d{features.cluster_center_middle_lane_foot_point} *
      Eigen::AngleAxisd(features.cluster_center_lane_orientation, Eigen::Vector3d::UnitZ())};

  return to_cluster_center.inverse() * (guessed_width * VehiclePoint::UnitY());
}

inline auto toVehicle(const BirdsviewPatch &bv_patch) {
  return [&bv_patch](const auto &p) { return bv_patch.imageToVehicle(p); };
}

auto asVehiclePoints(const CVPoints &cv_points, const BirdsviewPatch &bv_patch) {
  return boost::adaptors::transform(cv_points, [&bv_patch](const auto &p) {
    return bv_patch.imageToVehicle(toEigen(p));
  });
}

Direction CrosswalkClassifier::getCrosswalkDirection(const Features &features,
                                                     const std::vector<CVPoints> &crosswalk_lines) const {
  const auto tooSmall = [](const auto &v) { return v.size() < 2; };
  if (boost::algorithm::any_of(crosswalk_lines, tooSmall)) {
    return Direction::UnitX();
  }

  VehiclePoints all_directions;

  for (auto it = std::next(crosswalk_lines.begin()); it != crosswalk_lines.end(); it++) {
    VehiclePoints current_directions;
    current_directions.reserve(std::min(it->size(), std::prev(it)->size()));
    boost::transform(asVehiclePoints(*it, features.birdsview_patch),
                     asVehiclePoints(*std::prev(it), features.birdsview_patch),
                     std::back_inserter(current_directions),
                     [](const auto &a, const auto &b) {
                       return common::ensureSameOrientation(
                           (b - a).normalized(), VehiclePoint::UnitX());
                     });

    boost::push_back(all_directions, current_directions);
  }

  return common::mean(all_directions);
}

VehiclePoints CrosswalkClassifier::convexHull(const VehiclePose &cw_pose) const {
  const double lane_width = parameter_interface_->getParam(GUESSED_LANE_WIDTH);
  // get left and right points, assuming that lane width is fitting
  // unit vector in direction of crosswalk
  const VehiclePoint unit_dir = (cw_pose.linear() * VehiclePoint::UnitX()).normalized();
  const VehiclePoint normal =
      common::ensureSameOrientation(
          VehiclePoint{-unit_dir.y(), unit_dir.x(), 0}, VehiclePoint::UnitX())
          .normalized();
  const VehiclePoint right = cw_pose.translation() - lane_width * normal;
  const VehiclePoint left = cw_pose.translation() + lane_width * normal;

  // length of crosswalk from cup regulations
  const double crosswalk_length = parameter_interface_->getParam(CROSSWALK_LENGTH);
  // return all points
  return {{right, left, left + crosswalk_length * unit_dir, right + crosswalk_length * unit_dir}};
}

bool CrosswalkClassifier::isTooSmall(const Features &features) const {
  if (features.cluster.feature_points_vehicle.size() < 2) {
    return true;
  }
  // basic idea: take the two points with maximum distance to each other in
  // cluster
  // and project them onto unit vector pointing in direction of cluster center
  // lane orientation
  std::vector<std::pair<VehiclePoint, VehiclePoint>> point_pairs;
  point_pairs.reserve(features.cluster.feature_points_vehicle.size());
  for (const auto &fp : features.cluster.feature_points_vehicle) {
    const auto point_with_max_dist = *common::max_score(
        features.cluster.feature_points_vehicle, common::distanceTo(fp));
    point_pairs.emplace_back(fp, point_with_max_dist);
  }
  const auto max_pair = *common::max_score(point_pairs, [](const auto &pair) {
    return (pair.first - pair.second).norm();
  });
  Eigen::Vector3d direction = max_pair.first - max_pair.second;
  // generate vector with which is oriented parallel to cluster center lane
  // orientation, but since ccl is not exact if car is far away from cluster,
  // try PCA and use second component
  //  Eigen::MatrixXd comps;
  //  Eigen::VectorXd scores;
  //  common::pca_svd(common::toMatrix2D(features.cluster.feature_points_vehicle),
  //  &comps, &scores);

  //  const Eigen::Vector2d lane_orientation_2d{comps.block<2, 1>(0, 1)};
  //  const VehiclePoint lane_orientation = to3D(lane_orientation_2d);
  const Eigen::Vector3d lane_orientation =
      (Eigen::AngleAxisd(features.cluster_center_lane_orientation, Eigen::Vector3d::UnitZ()) *
       Eigen::Vector3d::UnitX())
          .normalized();
  // compute dot_product
  const double dot = std::fabs(direction.dot(lane_orientation));
  ROS_DEBUG(
      "isTooSmall: feature_point_cluster projected on "
      "cluster_center_lane_orientation: %f",
      dot);
  const double min_dot = parameter_interface_->getParam(MIN_PROJECTION_ON_LANE);
  return dot < min_dot;
}

bool CrosswalkClassifier::notWideEnough(const Features &features) const {

  if (features.cluster.feature_points_vehicle.size() < 2) {
    return true;
  }
  const auto shift = [&features](const auto &p) {
    return p - features.cluster_center3d;
  };

  const VehiclePoint cluster_direction_normal =
      (Eigen::AngleAxisd{features.cluster_center_lane_orientation, VehiclePoint::UnitZ()} *
       VehiclePoint::UnitX())
          .unitOrthogonal();

  const auto projectOntoLaneNormal = [&cluster_direction_normal](const auto &p) {
    return p.dot(cluster_direction_normal);
  };

  const auto min_max_projection = common::minmax_element(
      features.cluster.feature_points_vehicle | transformed(std::cref(shift)) |
      transformed(std::cref(projectOntoLaneNormal)));

  const auto min_size_normal = parameter_interface_->getParam(MIN_CLUSTER_SPREAD_NORMAL);

  return *min_max_projection.second - *min_max_projection.first <= min_size_normal;
}

bool CrosswalkClassifier::assumptPedestrianIsland(const Features &features) const {
  // sampling parameters
  const auto minmax_p = common::minmax_element(
      common::join(features.points_left, features.points_middle, features.points_right) |
      common::x_values);
  const auto disc_step = parameter_interface_->getParam(DISC_X_STEP);
  const auto x_start = *minmax_p.first;
  const auto x_end = (*minmax_p.second - *minmax_p.first) < disc_step
                         ? *minmax_p.first + disc_step
                         : *minmax_p.second;

  const common::DiscretizationParams params{x_start, x_end, disc_step};

  const auto left_point_distances =
      estimateLaneWidth(features.middle_lane_polynomial, params, features.points_left);

  if (left_point_distances.size() < 2) {
    return false;
  }

  const auto left_pd_mean = common::mean(left_point_distances);

  const auto max_lane_width = parameter_interface_->getParam(MAX_LANE_WIDTH);
  const auto min_lane_width = parameter_interface_->getParam(MIN_LANE_WIDTH);

  ROS_DEBUG("crosswalk_classifier: left_pd_mean is %f", left_pd_mean);

  ROS_DEBUG("crosswalk_classifier: assumptPedestrianIsland returns %d",
            left_pd_mean > max_lane_width || left_pd_mean < min_lane_width);

  return left_pd_mean > max_lane_width || left_pd_mean < min_lane_width;
}

std::vector<CVPoints> CrosswalkClassifier::onlyPointsOnLane(
    const std::vector<CVPoints> &crosswalk_lines, const Features &features) const {
  // select polynomials
  const auto polynomials = extractBoundingPolynomials(features);

  // transform points in into birdsview patch
  const auto toVehicle = [&features](const auto &p) {
    return features.birdsview_patch.imageToVehicle(cvPointToImagePoint(p));
  };

  const VehiclePoint projection_direction = common::ensureSameOrientation(
      (Eigen::AngleAxisd{features.cluster_center_lane_orientation, VehiclePoint::UnitZ()} *
       VehiclePoint::UnitX())
          .unitOrthogonal(),
      VehiclePoint::UnitY());

  const auto righterThanLeft = [&polynomials, &projection_direction](const auto &p) {
    return (utils::findLotfusspunkt(polynomials.first, p) - p).dot(projection_direction) > 0.0;
  };

  const auto lefterThanRight = [&polynomials, &projection_direction](const auto &p) {
    return (utils::findLotfusspunkt(polynomials.second, p) - p).dot(projection_direction) < 0.0;
  };

  const auto toBirdsview = [&features](const auto &p) {
    return imagePointToCvPoint(features.birdsview_patch.vehicleToImage(p));
  };

  std::vector<CVPoints> filtered_points;
  filtered_points.reserve(crosswalk_lines.size());

  for (const auto &points : crosswalk_lines) {
    filtered_points.push_back(common::make_vector(
        points | transformed(toVehicle) | filtered(lefterThanRight) |
        filtered(righterThanLeft) | transformed(toBirdsview)));
  }

  // remove empty points
  boost::remove_erase_if(filtered_points, [](const auto &c) { return c.empty(); });

  return filtered_points;
}

inline bool isBetween(const double t, const double low, const double hi) {
  return t > low && t <= hi;
}

std::vector<double> CrosswalkClassifier::estimateLaneWidth(
    const common::polynomial::DynamicPolynomial &lane_polynom,
    const common::DiscretizationParams &params,
    const VehiclePoints &outer_lane_points) const {
  if (outer_lane_points.size() < 2) {
    return {};
  }

  const auto poly_deg = static_cast<int>(lane_polynom.getCoefficients().size() - 1);

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

  return common::make_vector(lane_poly_points | filtered(between) | transformed(estimate));
}

void CrosswalkClassifier::createCrosswalk(RoadObjects &crosswalk,
                                          const Features &features,
                                          const std::vector<CVPoints> &crosswalk_lines,
                                          const double score) const {
  const auto loc_point =
      computeLocalizationPoint(features.middle_lane_polynomial,
                               features.points_left,
                               features.cluster_center_lane_orientation,
                               features.cluster_center_middle_lane_foot_point);
  const auto cw_dir = getCrosswalkDirection(features, crosswalk_lines);
  const double angle = common::getAngle(to2D(cw_dir), Eigen::Vector2d::UnitX());
  const VehiclePose actual_vehicle_pose{Eigen::Translation3d{loc_point} *
                                        Eigen::AngleAxisd{angle, VehiclePoint::UnitZ()}};

  ROS_DEBUG_STREAM(COLOR_CYAN << "score of crosswalk_classifier: " << score << COLOR_DEBUG);

  const auto min_score = parameter_interface_->getParam(MIN_SCORE);

  const auto output_score = score < min_score ? 0.0 : score;
  const VehiclePoints base_hull_polygon = convexHull(actual_vehicle_pose);
  crosswalk.push_back(std::make_unique<Crosswalk>(
      features.timestamp, output_score, actual_vehicle_pose, base_hull_polygon));
}

VehiclePoint CrosswalkClassifier::computeLocalizationPoint(
    const common::DynamicPolynomial &middle_lane_polynomial,
    const VehiclePoints & /*left_points*/,
    const double angle,
    const VehiclePoint &middle_point) const {

  const double step = parameter_interface_->getParam(STEP_LOC_POINT);
  common::DiscretizationParams params{closest_point.x(), farest_point.x(), step};

  VehiclePoints middle_lane_points;
  common::discretisize(middle_lane_polynomial, params, &middle_lane_points);

  const VehiclePoint loc_point =
      computeLanePoint(middle_lane_points, angle, middle_point, closest_point);

  return Eigen::AngleAxisd(angle, Eigen::Vector3d::UnitZ()).inverse() * loc_point + closest_point;
}

}  // namespace road_object_detection
