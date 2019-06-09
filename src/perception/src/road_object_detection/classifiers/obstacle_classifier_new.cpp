#include "obstacle_classifier_new.h"
#include <common/angle_conversions.h>
#include <common/discretisize.h>
#include <common/eigen_utils.h>
#include <common/pca_eigen.h>
#include <common/polynomial.h>
#include <common/polynomial_utils.h>
#include <common/polynomialfit.h>
#include <opencv_eigen_conversions.h>
#include <opencv_utils.h>
#include <perception_types.h>
#include <numeric>
#include "../../utils/ego_vehicle.h"
#include "common/make_vector.h"

#include "../../utils/foot_finder.h"
#include "../../utils/linear_clustering.h"
#include "../../utils/step_detection.h"

THIRD_PARTY_HEADERS_BEGIN
#include <cv.h>
#include <Eigen/Dense>
#include <boost/algorithm/clamp.hpp>
#include <boost/range/adaptor/transformed.hpp>
#include <boost/range/algorithm_ext.hpp>
#include <cmath>
#include "boost/range/algorithm.hpp"

THIRD_PARTY_HEADERS_END

namespace road_object_detection {

using boost::adaptors::transformed;

/******************************************************************************
 *Line2D***********************************************************************
 *****************************************************************************/
Line2d::Line2d(const Eigen::Vector2d &start, const Eigen::Vector2d &end)
    : Line<Eigen::Vector2d>(start, end) {}

boost::optional<Eigen::Vector2d> Line2d::intersectInfiniteRange(const Line2d &other) const {
  const auto other_direction_normalized = other.directionNormalized();
  const Eigen::Vector2d other_normal(other_direction_normalized(1),
                                     -other_direction_normalized(0));
  const double other_d = other_normal.dot(other.start);
  const double t = (other_d - start.dot(other_normal)) / diff().dot(other_normal);
  if (std::isfinite(t)) {
    return boost::make_optional<Eigen::Vector2d>(start + diff() * t);
  }
  return boost::none;
}

boost::optional<Eigen::Vector2d> Line2d::intersectInBounds(const Line2d &other) const {
  const boost::optional<Eigen::Vector2d> intersect = intersectInfiniteRange(other);
  if (!intersect) {
    return boost::none;
  }
  // check bounds
  const auto d1 = this->signedDistanceAlongFromStart(*intersect);
  const auto d2 = other.signedDistanceAlongFromStart(*intersect);
  if (d1 > 0 && d1 < this->length() && d2 > 0 && d2 < other.length()) {
    return intersect;
  }
  return boost::none;
}

ImagePointExact Line2d::normalVectorLeftUsingLeftCoord() const {
  const auto direction = directionNormalized();
  return ImagePointExact(direction[1], -direction[0]);
}

ImagePointExact Line2d::normalVectorRightUsingLeftCoord() const {
  const auto direction = directionNormalized();
  return ImagePointExact(-direction[1], direction[0]);
}

bool Line2d::isPartiallyInsidePolygon(const ImagePointsExact &polygon_points) const {

  const auto toCVConversion = [](const auto &p) {
    return imagePointExactToCvPoint(p);
  };

  const auto cv_polygon =
      common::make_vector(polygon_points | transformed(toCVConversion));

  if (cv::pointPolygonTest(toInputArray(cv_polygon), toCV(start), false) > 0) {
    return true;
  }
  if (cv::pointPolygonTest(toInputArray(cv_polygon), toCV(end), false) > 0) {
    return true;
  }
  // if neither start nor end point included => then there must be an
  // intersection point between the line and the polygon

  for (auto it = std::next(polygon_points.begin()); it != polygon_points.end(); it++) {
    const Line2d line(*std::prev(it), *it);
    if (intersectInBounds(line)) {
      return true;
    }
  }
  const Line2d line(polygon_points.back(), polygon_points.front());
  return static_cast<bool>(intersectInBounds(line));
}

void Line2d::calcConsensusSetInfLine(const ImagePointsExact &pts,
                                     const double pixel_distance_thld,
                                     ImagePointsExact &comsesnus,
                                     ImagePointsExact &non_comsesnus) const {

  for (const ImagePointExact &p : pts) {
    if (distanceInf(p) < pixel_distance_thld) {
      comsesnus.emplace_back(p);
    } else {
      non_comsesnus.emplace_back(p);
    }
  }
}

Line2d Line2d::estimate(const ImagePointsExact &points) {
  const Eigen::MatrixX2d data_points = common::toMatrix2D<Eigen::MatrixX2d>(points);
  const Eigen::Vector2d mean = data_points.colwise().mean().transpose();
  const Eigen::MatrixX2d centered = data_points.rowwise() - mean.transpose();
  const Eigen::Matrix2d cov = centered.adjoint() * centered / data_points.rows();
  const Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> eigen_solver(cov);

  // a line shall start on the left and end at the right
  const Eigen::Vector2d plane_vector = common::ensureSameOrientation(
      eigen_solver.eigenvectors().col(1), Eigen::Vector2d::UnitX());

  const auto distanceAlongLine = [&](const ImagePointExact &p1) {
    return plane_vector.dot(p1 - mean);
  };

  // boost adaptive range
  const auto minmax =
      common::minmax_element(points | transformed(std::cref(distanceAlongLine)));

  return Line2d(*minmax.first * plane_vector + mean, *minmax.second * plane_vector + mean);
}

Line3d Line2d::asGroundLineInVehicle(const common::CameraTransformation *const camera_transformation) const {
  return Line3d(
      camera_transformation->transformImageToGround(common::eigen_utils::round(start)),
      camera_transformation->transformImageToGround(common::eigen_utils::round(end)));
}

/**************************************************************************
 *Line3D*******************************************************************
 *************************************************************************/
Line3d::Line3d(const Eigen::Vector3d &start, const Eigen::Vector3d &end)
    : Line<Eigen::Vector3d>(start, end) {}

Line2d Line3d::asImageLine(const common::CameraTransformation *const camera_transformation) const {
  return Line2d(camera_transformation->transformVehicleToImageExact(start),
                camera_transformation->transformVehicleToImageExact(end));
}

/******************************************************************************
 ***** ObstacleFrontModel*****************************************************
 *****************************************************************************/

void ObstacleFrontModel::reset() {
  resetCnts(0);
  resetCnts(1);
  penalty_factor = 1;
  total_score = 0;
  score[0] = 0;
  score[1] = 0;
  model_type = ModelType::NONE;
  // resetDebugData();
}

// void ObstacleFrontModel::resetDebugData() {
//  debug_edge_score_b.fill(0.0);
//  debug_edge_score_v.fill(0.0);
//  debug_peneltizde_score.fill(0.0);
//  debug_gray_mean.fill(0.0);
//  debug_gray_mean_score.fill(0.0);
//  debug_gray_std_dev.fill(0.0);
//  debug_gray_std_dev_score.fill(0.0);
//  debug_b_entropie_score.fill(0.0);
//  debug_v_entropie_score.fill(0.0);
//  debug_vertical_angle_score.fill(0.0);
//  debug_ground_angle_score.fill(0.0);
//}

void ObstacleFrontModel::resetCnts(const std::size_t index) {
  b_set_sizes.at(index) = 0;
  v_set_sizes.at(index) = 0;
  nr_penaltized.at(index) = 0;

  std::fill(b_hists.at(index).begin(), b_hists.at(index).end(), 0);
  std::fill(v_hists.at(index).begin(), v_hists.at(index).end(), 0);
}

void ObstacleFrontModel::setModelState(const ModelType &model_state) {
  this->model_type = model_state;
}

bool ObstacleFrontModel::generateModelL(const Line2d &bottom0,
                                        const Line2d &vertical0,
                                        const cv::Size &img_size,
                                        const ModelParams &model_params,
                                        const common::CameraTransformation *const camera_transformation,
                                        const bool vertical_from_left_dataset) {
  // TODO param
  const double min_pixel_area = 150;
  const double min_unit_cos = 0.7;

  b.front() = bottom0;
  b.front().sortForDimension(0);

  // calculate intersection of vertical and horizontal bottom
  const boost::optional<ImagePointExact> intersection =
      b.front().intersectInfiniteRange(vertical0);
  if (!intersection || (intersection.get().x() < 0 || intersection.get().y() < 0 ||
                        intersection.get().x() >= img_size.width ||
                        intersection.get().y() >= img_size.height)) {
    // intersection point not valid(e.g out of image frame)
    return false;
  }

  // check if vertical is left (model is L or L mirrored)
  ImagePointExact extend_direction;
  if (b.front().squaredDistanceToStart(intersection.get()) <
      b.front().squaredDistanceToEnd(intersection.get())) {
    vertical_is_left.front() = true;
    extend_direction = b.front().directionNormalized();
  } else {
    vertical_is_left.front() = false;
    extend_direction = -b.front().directionNormalized();
  }

  // if model orientation does not match the expected data set => no valid model
  if (vertical_is_left.front() != vertical_from_left_dataset) {
    return false;
  }

  // extend horizontal bottom line to minimum line length
  Line3d hb3 = b.front().asGroundLineInVehicle(camera_transformation);
  const double hb_2d_length =
      std::max(b.front().length(),
               b.front().length() * model_params.min_model_size_in_m / hb3.length());
  b.front().start = intersection.get();
  b.front().end = intersection.get() + extend_direction * hb_2d_length;
  // check if some points are out of image => invalidate model for such case
  if (b.front().end[0] < 0 || b.front().end[1] < 0 ||
      b.front().end[0] >= img_size.width || b.front().end.y() >= img_size.height) {
    return false;
  }
  hb3 = b.front().asGroundLineInVehicle(camera_transformation);
  // hb start point is equal to the intersection point of the vertical and the
  // horizontal

  // extend vertical line length to minimum line length
  const double verztical_unit_length2 =
      (b.front().start - camera_transformation->transformVehicleToImageExact(
                             hb3.start + Eigen::Vector3d::UnitZ()))
          .norm();
  const double vertical3d_length_estimate = vertical0.length() / verztical_unit_length2;
  const double vertical_length2 = std::max(
      vertical0.length(),
      vertical0.length() * model_params.min_model_size_in_m / vertical3d_length_estimate);

  v.front() = vertical0;
  v.front().sortForDimensionInverse(1);
  v.front().start = intersection.get();
  v.front().end = v.front().start + v.front().directionNormalized() * vertical_length2;
  // check if some points are out of image => invalidate model for such case
  if (v.front().end.x() < 0 || v.front().end.y() < 0 ||
      v.front().end.x() >= img_size.width || v.front().end.y() >= img_size.height) {
    return false;
  }

  // Force a minimum area spanned by the two lines.
  // This is needed so that the score evaluation is staticly sigfnificant.
  // force a minimum (2d_image) angle between both lines. Otherwise the fit is
  // not stable and obstacle direction might be estimated incorrectly
  if (std::abs(v.front().directionNormalized().dot(b.front().directionNormalized())) > min_unit_cos ||
      getModelArea() < min_pixel_area) {
    return false;
  }

  model_type = ModelType::L;

  return true;
}

bool ObstacleFrontModel::extendModelWith2ndGroundLine(
    const Line2d bottom2_candidate,
    const ModelParams &model_params,
    const cv::Size &img_size,
    const common::CameraTransformation *const camera_transformation) {

  // TODO: Pramams
  const double max_intersection_tolerance_pixel = 25;
  // save primray bottom in case model creation fails and earlier model needs to
  // be restored
  const Line2d temp_primary_bottom = b.front();
  const Line2d temp_primary_vertical = v.front();

  Line3d b1_3d_line = b.front().asGroundLineInVehicle(camera_transformation);
  Line3d b2_3d = bottom2_candidate.asGroundLineInVehicle(camera_transformation);
  // check whether the givven line is within given angle constraints

  const double angle = std::acos(b2_3d.diff().dot(b1_3d_line.diff()));
  if (std::abs(M_PI_2 - angle) > model_params.ground_line_perpendicular_angle_tolerance) {
    // ROS_DEBUG(
    //   "Reject 2ndGND extension: => ground line angle constronats violated");
    return false;
  }

  boost::optional<ImagePointExact> intersection =
      b.front().intersectInfiniteRange(bottom2_candidate);
  if (!intersection || (intersection.get().x() < 0 || intersection.get().y() < 0 ||
                        intersection.get().x() >= img_size.width ||
                        intersection.get().y() >= img_size.height)) {
    // intersection point not valid(e.g out of image frame)
    // ROS_DEBUG("Reject 2ndGND extension: => invalid intersection point");
    return false;
  }

  bool v2_set = false;
  b.back().start = intersection.get();

  ImagePointExact b0_direction_from_interesect;  // needed for plausibility
                                                 // check

  if ((intersection.get() - b.front().start).squaredNorm() <
      (intersection.get() - b.front().end).squaredNorm()) {
    // all thre lines(v[0], b[0], b[1] describe the same corner
    if ((b.front().start - b.back().start).norm() > max_intersection_tolerance_pixel) {
      // intersection points too far away from each other => bad model
      // ROS_DEBUG("Reject 2ndGND extension: => Inconclusive intersection
      // point");
      return false;
    }
    // => use avergae of intersection point => override intersection
    intersection =
        boost::make_optional<ImagePointExact>(0.5 * (b.front().start + b.back().start));
    b.front().start = intersection.get();
    b.back().start = intersection.get();
    v.front().start = intersection.get();
    v.back() = v.front();
    v2_set = true;

    b0_direction_from_interesect = b.front().directionNormalized();
  } else {
    b.front().end = intersection.get();
    b0_direction_from_interesect = -b.front().directionNormalized();

    const double intersection_3distSquared =
        camera_transformation
            ->transformImageToGround(common::eigen_utils::round(intersection.get()))
            .squaredNorm();

    const Line3d b0_3d_line = b.front().asGroundLineInVehicle(camera_transformation);
    if (intersection_3distSquared > b0_3d_line.start.squaredNorm()) {
      // new intersection point must be the closest vertex. Otherwise reject
      // model.
      // ROS_DEBUG(
      //    "Reject 2ndGND extension: => Intersection point not closest
      //    vertex");
      b.front() = temp_primary_bottom;
      v.front() = temp_primary_vertical;
      return false;
    }

    if (b0_3d_line.length() < model_params.min_model_size_in_m) {
      // the first ground line  must not become smaller than the minimum
      // obstacle dimension

      // ROS_DEBUG("Reject 2ndGND extension: => to short primary gnd line");
      b.front() = temp_primary_bottom;
      v.front() = temp_primary_vertical;
      return false;
    }
  }

  const double b2_2d_length = std::max(
      bottom2_candidate.length(),
      bottom2_candidate.length() * model_params.min_model_size_in_m /
          bottom2_candidate.asGroundLineInVehicle(camera_transformation).length());

  // check which point of 2nd_ground_line is closer to the intersecion point
  if ((intersection.get() - bottom2_candidate.start).squaredNorm() <
      (intersection.get() - bottom2_candidate.end).squaredNorm()) {
    b.back().end =
        b.back().start + b2_2d_length * bottom2_candidate.directionNormalized();
  } else {
    b.back().end =
        b.back().start - b2_2d_length * bottom2_candidate.directionNormalized();
  }

  // ceheck if 2nd gnd line is invalid because it is hidden behind the
  // obstacle.(This means that the given line cannot be a valid ground line,
  // as the previous L-model is expected to be correct)
  // check if angle is greater than 90 degree(in image coordinates)
  if (b0_direction_from_interesect.dot(b[1].directionNormalized()) > 0) {
    // 2nd ground line would not be seen
    // restore previous L-model data
    // ROS_DEBUG("Reject 2ndGND extension:  => acute angle in image
    // coordinares");
    b.front() = temp_primary_bottom;
    v.front() = temp_primary_vertical;
    return false;
  }

  // check wether the 2nd ground line points towards the vehicle => reject
  b2_3d = b.back().asGroundLineInVehicle(camera_transformation);
  if (b2_3d.start.dot(b2_3d.diff()) < 0) {
    // 2nd ground line points in wrong direction
    // restore previous L-model data
    // ROS_DEBUG("Reject 2ndGND extension:  => pointing towards vehicle");
    b.front() = temp_primary_bottom;
    v.front() = temp_primary_vertical;
    return false;
  }

  // create a guess for a 2nd vertical (needed for gray scale score evaluation)
  if (!v2_set) {
    // create a guess fot v2 so that an evaluation of the model will be possible
    //=> assume obstalce at z=0
    const Line3d v2_3d(
        b2_3d.start, b2_3d.start + VehiclePoint(0, 0, model_params.min_model_size_in_m));
    v.back() = v2_3d.asImageLine(camera_transformation);
  }
  model_type = ModelType::L_2ND_GND_LINE;
  return true;
}

ObstacleVertices ObstacleFrontModel::getGroundVertices(
    const common::CameraTransformation *const camera_transformation) const {
  // Assumptions:
  // square base hull polygon
  // no incline / decline
  // Note: it is expected that the obstacle size is underestimated
  if (model_type == ModelType::L) {
    const Line3d l3_b0 = b.front().asGroundLineInVehicle(camera_transformation);
    const VehiclePoint l3_b0_diff = l3_b0.diff();
    const VehiclePoint l3_b0_dir_rotated =
        VehiclePoint(-l3_b0_diff.y(), l3_b0_diff.x(), 0);
    const VehiclePoint offset =
        (l3_b0.mean().dot(l3_b0_dir_rotated) >= 0) ? l3_b0_dir_rotated : -l3_b0_dir_rotated;

    const VehiclePoints vertices = {
        l3_b0.start, l3_b0.end, l3_b0.end + offset, l3_b0.start + offset};
    const std::vector<Obstacle::DetectionState> vertices_detected = {
        Obstacle::DetectionState::VERTEX_DETECTED,
        Obstacle::DetectionState::VERTEX_INSECURE,
        Obstacle::DetectionState::VERTEX_INSECURE,
        Obstacle::DetectionState::VERTEX_INSECURE};
    return ObstacleVertices(vertices, vertices_detected);

  } else if (model_type == ModelType::L_2ND_GND_LINE) {
    // Note the actually detected vertices are the start pts of the ground lines
    // b[0],  and potentially of b[1]
    const Line3d l3_b0 = b.front().asGroundLineInVehicle(camera_transformation);
    const Line3d l3_b1 = b.back().asGroundLineInVehicle(camera_transformation);
    const VehiclePoints vertices = {
        l3_b0.start, l3_b0.end, l3_b0.end + l3_b1.diff(), l3_b0.start + l3_b1.diff()};
    const std::vector<Obstacle::DetectionState> vertices_detected = {
        Obstacle::DetectionState::VERTEX_DETECTED,
        (b.front().end == b.back().start) ? Obstacle::DetectionState::VERTEX_DETECTED
                                          : Obstacle::DetectionState::VERTEX_INSECURE,
        Obstacle::DetectionState::VERTEX_INSECURE,
        Obstacle::DetectionState::VERTEX_INSECURE};
    return ObstacleVertices(vertices, vertices_detected);
  }

  ROS_WARN(
      "ObstacleFrontModel in invalid state for BaseHullPolygon estimation");
  // invalid state => return empty polygon
  const VehiclePoints vertices = {};
  const std::vector<Obstacle::DetectionState> vertices_detected = {};
  return ObstacleVertices(vertices, vertices_detected);
}


void ObstacleFrontModel::calcConsensusSet(const ImagePointsExact &b_pts,
                                          const ModelParams &model_params,
                                          const std::size_t model_index) {

  resetCnts(model_index);

  for (const ImagePointExact &p : b_pts) {
    if (supportsModel(b.at(model_index), p, model_params.distance_thld_pixel)) {
      b_set_sizes.at(model_index)++;
      ObstacleFrontModel::incrementLineSupportHistogram(
          b_hists.at(model_index), b.at(model_index), p);
    } else if (isInsideFrontModel(p, model_params.distance_thld_pixel, model_index)) {
      nr_penaltized.at(model_index)++;
    }
  }
}


void ObstacleFrontModel::calcConsensusSet(const ImagePointsExact &b_pts,
                                          const ImagePointsExact &v_pts,
                                          ImagePointsExact &out_not_b_inf_line,
                                          const ModelParams &model_params,
                                          const std::size_t model_index) {

  resetCnts(model_index);

  for (const ImagePointExact &p : b_pts) {
    if (supportsModel(b.at(model_index), p, model_params.distance_thld_pixel)) {
      b_set_sizes.at(model_index)++;
      ObstacleFrontModel::incrementLineSupportHistogram(
          b_hists.at(model_index), b.at(model_index), p);
    } else if (isInsideFrontModel(p, model_params.distance_thld_pixel, model_index)) {
      nr_penaltized.at(model_index)++;
    }

    if (!supportsInfLine(b.at(model_index), p, model_params.distance_thld_pixel)) {
      out_not_b_inf_line.emplace_back(p);
    }
  }

  for (const ImagePointExact &p : v_pts) {
    if (supportsModel(v.at(model_index), p, model_params.distance_thld_pixel)) {
      v_set_sizes.at(model_index)++;
      ObstacleFrontModel::incrementLineSupportHistogram(
          v_hists.at(model_index), v.at(model_index), p);
    } else if (isInsideFrontModel(p, model_params.distance_thld_pixel, model_index)) {
      nr_penaltized.at(model_index)++;
    }
  }
}

void ObstacleFrontModel::incrementLineSupportHistogram(std::array<int, hist_size> &hist_cnts,
                                                       const Line2d &line,
                                                       const ImagePointExact &p) {

  const std::size_t bin_index =
      std::min(static_cast<std::size_t>(std::floor(
                   (line.distanceToStart(p) / line.length()) * hist_cnts.size())),
               (hist_cnts.size() - 1));
  hist_cnts.at(bin_index)++;
}

double ObstacleFrontModel::getNormalizedEntropyMeasure(const std::array<int, hist_size> &hist_cnts,
                                                       const std::size_t nr_pts_in_hist,
                                                       const ModelParams &model_params) {
  // Note: before estimating entropie to each bin in histogramm 1 is added.
  // (Toavoid inf. problems)
  if (hist_cnts.size() <= 1) {
    return 0;
  }
  const double N_modified =
      static_cast<double>(nr_pts_in_hist) +
      model_params.histogram_init_value * static_cast<double>(hist_cnts.size());
  double result = 0;
  for (const double cnt : hist_cnts) {
    const double p = ((cnt + model_params.histogram_init_value) / N_modified);
    result -= p * std::log2(p);
  }
  result = result / std::log2(static_cast<double>(hist_cnts.size()));
  return result;
}

ObstacleFrontModel::ModelType ObstacleFrontModel::getModelType() const {
  return model_type;
}

void ObstacleFrontModel::meanStdDevOfGrayInsideModel(const cv::Mat &image_gray8,
                                                     double &out_mean,
                                                     double &out_std_dev,
                                                     const std::size_t model_index) const {

  const std::vector<cv::Point> &convex_polygon = asConvexPolygonCV(model_index);
  const cv::Rect rect =
      getExpandedImgCroppedRect(convex_polygon, 0, 0, image_gray8.size());
  const cv::Mat img = image_gray8(rect);
  cv::Mat mask(img.rows, img.cols, img.type(), cv::Scalar(0));

  std::vector<cv::Point> offseted_poly;
  offseted_poly.reserve(convex_polygon.size());
  std::transform(convex_polygon.begin(),
                 convex_polygon.end(),
                 std::back_inserter(offseted_poly),
                 [&rect](const cv::Point &p) { return p - rect.tl(); });

  cv::fillConvexPoly(mask, offseted_poly, cv::Scalar(255));

  cv::Scalar mean;
  cv::Scalar std_dev;
  cv::meanStdDev(img, mean, std_dev, mask);

  out_mean = mean[0];
  out_std_dev = std_dev[0];
}

double ObstacleFrontModel::evaluateScore(const cv::Mat &image_complete,
                                         const std::size_t index,
                                         const ModelParams &model_params,
                                         const common::CameraTransformation *const camera_transformation) {

  // calculate score for edge counts
  double edge_score_bottom = 0;
  double edge_score_vertical = 0;
  if (index == 0) {
    edge_score_bottom =
        linearInterpolFctPts(b_set_sizes.at(index), model_params.b0_edge_cnt_eval_fct);
    edge_score_vertical =
        linearInterpolFctPts(v_set_sizes.at(index), model_params.v0_edge_cnt_eval_fct);
  } else if (index == 1) {
    edge_score_bottom =
        linearInterpolFctPts(b_set_sizes.at(index), model_params.b1_edge_cnt_eval_fct);
    edge_score_vertical = 1;
  }

  // penaltize if points are detected inside the obstacle_front_model
  const double penalized_pts_score = linearInterpolFctPts(
      nr_penaltized.at(index), model_params.penalized_pts_eval_fct);

  // score mean and std_dev of gray-scale image inside fornt model
  double mean, std_dev;
  meanStdDevOfGrayInsideModel(image_complete, mean, std_dev, index);
  const double gray_mean_score = linearInterpolFctPts(mean, model_params.gray_mean_eval_fct);
  const double gray_stddev_score =
      linearInterpolFctPts(std_dev, model_params.gray_stddev_eval_fct);

  const double hb_entropie_score = ObstacleFrontModel::getNormalizedEntropyMeasure(
      b_hists.at(index), b_set_sizes.at(index), model_params);

  const double v_entropie_score = ObstacleFrontModel::getNormalizedEntropyMeasure(
      v_hists.at(index), v_set_sizes.at(index), model_params);

  // evaluate diff between 2d vertical and ideal 2d vertical => Note this will
  // penalize invalid verticals, but also valid verticlas in case of a slope
  const VehiclePoint ground_point = camera_transformation->transformImageToGround(
      common::eigen_utils::round(v.at(index).start));
  const ImagePointExact v_top_point = camera_transformation->transformVehicleToImageExact(
      ground_point + VehiclePoint(0, 0, 0.1));
  const ImagePointExact ideal_vertical_direction =
      (v_top_point - v.at(index).start).normalized();
  const double angle2d_diff = std::acos(
      std::abs(ideal_vertical_direction.dot(v.at(index).directionNormalized())));

  const double vertical_angle_score =
      linearInterpolFctPts(angle2d_diff, model_params.vertical_angle_image_eval_fct);

  // calculate partial score
  score.at(index) = 0;
  if (index == 0) {
    // model_index 0 always contains a complete L-type model
    score.at(index) = edge_score_bottom * edge_score_vertical *
                      penalized_pts_score * gray_mean_score * gray_stddev_score *
                      hb_entropie_score * v_entropie_score * vertical_angle_score;
  } else {
    // model_index == 1
    if (model_type == ModelType::L_2ND_GND_LINE) {
      // estimate angle between both ground lines in vehicle coordinates.
      const Line3d b03 = b.front().asGroundLineInVehicle(camera_transformation);
      const Line3d b13 = b.back().asGroundLineInVehicle(camera_transformation);
      const double angle =
          std::acos(b03.directionNormalized().dot(b13.directionNormalized()));
      const double ground_lines_angle_score = linearInterpolFctPts(
          std::abs(angle - M_PI_2), model_params.ground_lines_angle_eval_fct);

      // extended model is not expected to have a correctly chosen vertical =>
      // only judge ground line
      score.at(index) = edge_score_bottom * penalized_pts_score * gray_mean_score *
                        gray_stddev_score * hb_entropie_score * ground_lines_angle_score;

      // debug
      // debug_ground_angle_score[index] = ground_lines_angle_score;
      // debug end
    } else {
      ROS_WARN("ObstacleClassifierNew:: Invalid model state.");
    }
  }
  // save some debug output
  //  debug_edge_score_b[index] = edge_score_bottom;
  //  debug_edge_score_v[index] = edge_score_vertical;
  //  debug_peneltizde_score[index] = penalized_pts_score;
  //  debug_gray_mean[index] = mean;
  //  debug_gray_mean_score[index] = gray_mean_score;

  //  debug_gray_std_dev[index] = std_dev;
  //  debug_gray_std_dev_score[index] = gray_stddev_score;
  //  debug_b_entropie_score[index] = hb_entropie_score;
  //  debug_v_entropie_score[index] = v_entropie_score;
  //  debug_vertical_angle_score[index] = vertical_angle_score;
  // debug end

  return score.at(index);
}

double ObstacleFrontModel::evaluateScore(const cv::Mat &image_complete,
                                         const ModelParams &model_params,
                                         const common::CameraTransformation *const camera_transformation) {

  if (model_type == ObstacleFrontModel::ModelType::L) {
    total_score = penalty_factor *
                  evaluateScore(image_complete, 0, model_params, camera_transformation);

  } else if (model_type == ObstacleFrontModel::ModelType::L_2ND_GND_LINE) {
    total_score =
        penalty_factor * 0.5 *
        (evaluateScore(image_complete, 0, model_params, camera_transformation) +
         evaluateScore(image_complete, 1, model_params, camera_transformation));
  } else {
    // should never get here
    total_score = 0;
    ROS_WARN("ObstacleClassifierNew:: Invalid model state.");
  }

  return total_score;
}

void ObstacleFrontModel::setPenaltyFactor(const double factor) {
  penalty_factor = factor;
  // update total score;
  total_score = factor * total_score;
}

void ObstacleFrontModel::resetPenaltyFactor() {
  if (penalty_factor > 0) {
    total_score = total_score / penalty_factor;
  }
  penalty_factor = 1;
}

double ObstacleFrontModel::getMostRecentScore() const { return total_score; }

double ObstacleFrontModel::getFirstSubModelRecentScore() const {
  return score.front();
}

double ObstacleFrontModel::getModelArea() const {
  return std::abs(b.front().diff().x() * v.front().diff().y() -
                  v.front().diff().x() * b.front().diff().y());
}

Line2d ObstacleFrontModel::getBottomLine(const std::size_t index) const {
  return b.at(index);
}
Line2d ObstacleFrontModel::getTopLine(const std::size_t index) const {
  return Line2d(b.at(index).start + v.at(index).diff(),
                b.at(index).end + v.at(index).diff());
}

Line2d ObstacleFrontModel::getVerticalLeft(const std::size_t index) const {
  return (vertical_is_left.at(index))
             ? v.at(index)
             : Line2d(v.at(index).start + b.at(index).diff(),
                      v.at(index).end + b.at(index).diff());
}
Line2d ObstacleFrontModel::getVerticalRight(const std::size_t index) const {
  return (!vertical_is_left.at(index))
             ? v.at(index)
             : Line2d(v.at(index).start + b.at(index).diff(),
                      v.at(index).end + b.at(index).diff());
}

std::vector<cv::Point> ObstacleFrontModel::asConvexPolygonCV(const std::size_t index) const {
  return {toCV(b.at(index).start),
          toCV(b.at(index).end),
          toCV(ImagePointExact(b.at(index).end + v.at(index).diff())),
          toCV(ImagePointExact(b.at(index).start + v.at(index).diff()))};
}

bool ObstacleFrontModel::isSecondGroundLineClearlyVisible(
    const double angle_threshold,
    const common::CameraTransformation *const camera_transformation) const {
  // Assuming there is no slope => all ground points at z=0
  // NOTE angle_threshold >=0!
  const Line3d b1_3d = b.front().asGroundLineInVehicle(camera_transformation);
  const VehiclePoint vertex_point = std::min(
      b1_3d.start, b1_3d.end, [](const VehiclePoint &p1, const VehiclePoint &p2) {
        return p1.squaredNorm() < p2.squaredNorm();
      });
  const VehiclePoint b1_3d_direction = b1_3d.directionNormalized();
  const VehiclePoint b1_3d_dir_rotated =
      VehiclePoint(-b1_3d_direction.y(), b1_3d_direction.x(), 0);
  // b2_3d_direction pointing away from the vehicle
  const VehiclePoint b2_3d_direction =
      (vertex_point.dot(b1_3d_dir_rotated) >= 0) ? b1_3d_dir_rotated : -b1_3d_dir_rotated;

  const VehiclePoint ideal_vertical_top_3d = vertex_point + VehiclePoint(0, 0, 0.1);
  // project point vertical top point onto the ground plane
  const VehiclePoint vertical_ground_direction =
      (camera_transformation->transformImageToGround(
           camera_transformation->transformVehicleToImage(ideal_vertical_top_3d)) -
       vertex_point)
          .normalized();
  // calculate angles to radiaöl vector
  const double abs_angle_er_vertical =
      std::acos(vertical_ground_direction.dot(vertex_point.normalized()));
  const double abs_angle_er_b2 =
      std::acos(b2_3d_direction.dot(vertex_point.normalized()));

  return ((abs_angle_er_b2 - abs_angle_er_vertical) >= angle_threshold);
}

void ObstacleFrontModel::rosDebugAll() const {
  ROS_DEBUG("-----------------------------------");
  ROS_DEBUG("Model Type :  %d", static_cast<int>(model_type));
  ROS_DEBUG("Total Score:  %f   ", total_score);
  ROS_DEBUG("Subscores:                [0]= %.3f   |    [1]= %.3f", score[0], score[1]);
  ROS_DEBUG("b_set_sizes:              [0]= %lu       |    [1]= %lu",
            b_set_sizes.front(),
            b_set_sizes.back());
  ROS_DEBUG("v_set_sizes:              [0]= %lu       |    [1]= %lu",
            v_set_sizes.front(),
            v_set_sizes.back());
  ROS_DEBUG("Vertical_is_left:         [0]= %d        |    [1]= %d",
            vertical_is_left.front(),
            vertical_is_left.back());
  ROS_DEBUG("nr_penaltized:            [0]= %lu       |   [1]= %lu",
            nr_penaltized.front(),
            nr_penaltized.back());
  //  ROS_DEBUG("debug_edge_score_b:       [0]= %.3f   |   [1]= %.3f",
  //            debug_edge_score_b[0],
  //            debug_edge_score_b[1]);
  //  ROS_DEBUG("debug_edge_score_v:       [0]= %.3f   |   [1]= %.3f",
  //            debug_edge_score_v[0],
  //            debug_edge_score_v[1]);
  //  ROS_DEBUG("debug_peneltizde_score    [0]= %.3f   |   [1] = %.3f",
  //            debug_peneltizde_score[0],
  //            debug_peneltizde_score[1]);
  //  ROS_DEBUG("debug_gray_mean:          [0]= %.0f      |    [1]= %.0f",
  //            debug_gray_mean[0],
  //            debug_gray_mean[1]);
  //  ROS_DEBUG("debug_gray_mean_score:    [0]= %.3f    |    [1]= %.3f",
  //            debug_gray_mean_score[0],
  //            debug_gray_mean_score[1]);
  //  ROS_DEBUG("debug_gray_stddev:        [0]= %.3f    |    [1]= %.3f",
  //            debug_gray_std_dev[0],
  //            debug_gray_std_dev[1]);
  //  ROS_DEBUG("debug_gray_std_dev_score: [0]= %.3f    |    [1]= %.3f",
  //            debug_gray_std_dev_score[0],
  //            debug_gray_std_dev_score[1]);

  //  ROS_DEBUG("debug_v_entropie_score:   [0]= %.3f    |     [1]= %.3f",
  //            debug_v_entropie_score[0],
  //            debug_v_entropie_score[1]);
  //  ROS_DEBUG("debug_b_entropie_score:   [0]= %.3f    |     [1]= %.3f",
  //            debug_b_entropie_score[0],
  //            debug_b_entropie_score[1]);
  //  ROS_DEBUG("angle_score:         vert[0]= %.3f    |ground[1]= %.3f",
  //            debug_vertical_angle_score[0],
  //            debug_ground_angle_score[1]);
  ROS_DEBUG("-----------------------------------");
}

bool ObstacleFrontModel::isInsideFrontModel(const ImagePointExact &p,
                                            const double pixel_distance_thld,
                                            const std::size_t index) const {
  const Line2d vl = getVerticalLeft(index);
  const Line2d vr = getVerticalRight(index);
  Line2d ht = getTopLine(index);
  Line2d hb = getBottomLine(index);
  ht.sortForDimension(0);
  hb.sortForDimension(0);
  return (hb.distanceVectorInf(p).dot(hb.normalVectorLeftUsingLeftCoord()) > pixel_distance_thld &&
          ht.distanceVectorInf(p).dot(ht.normalVectorRightUsingLeftCoord()) > pixel_distance_thld &&
          vl.distanceVectorInf(p).dot(vl.normalVectorRightUsingLeftCoord()) > pixel_distance_thld &&
          vr.distanceVectorInf(p).dot(vr.normalVectorLeftUsingLeftCoord()) > pixel_distance_thld);
}

bool ObstacleFrontModel::supportsModel(const Line2d &line,
                                       const ImagePointExact &p,
                                       const double pixel_distance_thld) const {
  return (line.distanceBounded(p) < pixel_distance_thld);
}

bool ObstacleFrontModel::supportsInfLine(const Line2d &line,
                                         const ImagePointExact &p,
                                         const double pixel_distance_thld) const {
  return (line.distanceInf(p) < pixel_distance_thld);
}

/******************************************************************************
 *ObstacleCLassifierNew*******************************************************
 *****************************************************************************/
const std::string ObstacleClassifierNew::PARAM_NAMESPACE(
    "obstacle_classifier_new");
const std::string ObstacleClassifierNew::PARAM_NAMESPACE_OBSTALCE_DIMENSION(
    "obstacle_classifier_new/obstacle_dimension");
const std::string ObstacleClassifierNew::PARAM_NAMESPACE_FRONT_MODEL(
    "obstacle_classifier_new/front_model");
const std::string ObstacleClassifierNew::PARAM_NAMESPACE_FOOT_FINDER(
    "obstacle_classifier_new/foot_finder");
const std::string ObstacleClassifierNew::PARAM_NAMESPACE_ROI(
    "obstacle_classifier_new/roi");
const std::string ObstacleClassifierNew::PARAM_NAMESPACE_FP_DETECT(
    "obstacle_classifier_new/fp_detection");
const std::string ObstacleClassifierNew::PARAM_NAMESPACE_RANSAC(
    "obstacle_classifier_new/ransac");

ObstacleClassifierNew::ObstacleClassifierNew(const common::CameraTransformation *const camera_transformation,
                                             ParameterInterface *const parameters_ptr)
    : camera_transformation_(camera_transformation),
      parameters_ptr_(parameters_ptr),
      foot_finder_params_(parameters_ptr, PARAM_NAMESPACE_FOOT_FINDER),
      obstacle_params_(parameters_ptr, PARAM_NAMESPACE_OBSTALCE_DIMENSION),
      model_params_(parameters_ptr, PARAM_NAMESPACE_FRONT_MODEL),
      roi_params_(parameters_ptr, PARAM_NAMESPACE_ROI, camera_transformation),
      fp_detect_params_(parameters_ptr, PARAM_NAMESPACE_FP_DETECT),
      ransac_params_(parameters_ptr, PARAM_NAMESPACE_RANSAC),
      ego_vehicle_(parameters_ptr) {}

RoadObjects ObstacleClassifierNew::classify(const Features &features) {

  const ImagePoints area_roi = getGroundAreaRoi(features);
  // fit ObstacleFrontModel and and fill RoadObject Vector
  RoadObjects road_objects;
  ImagePoints fp_bottom_out, fp_vertical_left_out, fp_vertical_right_out;
  getFrontShapeFeaturePoints(
      features, area_roi, fp_bottom_out, fp_vertical_left_out, fp_vertical_right_out);
  boost::optional<ObstacleFrontModel> model = fitObstacleFrontModel(
      fp_bottom_out, fp_vertical_left_out, fp_vertical_right_out, features);
  if (model) {
    // add model to road object vector
    const ObstacleVertices obstacle_vertices =
        estimateBaseHullPolygon(model.get(), features);
    // const ObstacleVertices obstacle_vertices =
    // model->getGroundVertices(camera_transformation_);
    road_objects.push_back(std::make_unique<Obstacle>(features.timestamp,
                                                      model->getMostRecentScore(),
                                                      obstacle_vertices.vertices,
                                                      obstacle_vertices.vertices_detection_state));
  }
  return road_objects;
}

size_t ObstacleClassifierNew::getClassifierId() const {
  return typeid(ObstacleClassifierNew).hash_code();
}

ImagePoints ObstacleClassifierNew::getGroundAreaRoi(const Features &features) const {
  std::vector<cv::Point2f> ground_2d_points;
  ground_2d_points.reserve(features.cluster.feature_points_vehicle.size());
  for (const VehiclePoint &vp : features.cluster.feature_points_vehicle) {
    ground_2d_points.emplace_back(vp[0], vp[1]);
  }
  cv::RotatedRect rotated_rect = cv::minAreaRect(ground_2d_points);

  std::array<cv::Point2f, 4> roi_2d_ground_points;
  rotated_rect.points(roi_2d_ground_points.data());

  std::sort(
      roi_2d_ground_points.begin(),
      roi_2d_ground_points.end(),
      [](const cv::Point2f &p1, const cv::Point2f &p2) { return p1.x < p2.x; });

  std::array<Eigen::Vector2d, 4> gps;
  for (std::size_t i = 0; i < 4; i++) {
    gps[i] = Eigen::Vector2d(static_cast<double>(roi_2d_ground_points[i].x),
                             static_cast<double>(roi_2d_ground_points[i].y));
  }


  const double width = boost::algorithm::clamp(
      (gps[1] - gps[0]).norm() + roi_params_.area_roi_extension_in_m,
      roi_params_.min_roi_width_in_m,
      roi_params_.max_roi_width_in_m);

  const double length = boost::algorithm::clamp(
      (gps[2] - gps[0]).norm() + roi_params_.area_roi_extension_in_m,
      roi_params_.min_roi_length_in_m,
      roi_params_.max_roi_length_in_m);

  //  if (features.roi_) {
  //    ROS_DEBUG("LOOK_AT");
  //  }
  //  ROS_DEBUG("rotated rect width: %f", (gps[1] - gps[0]).norm());
  //  ROS_DEBUG("rotated rect length: %f", (gps[2] - gps[0]).norm());
  //  ROS_DEBUG("forced width: %f", width);
  //  ROS_DEBUG("forced length: %f", length);

  // set width
  const Eigen::Vector2d mean_point_0_1 = 0.5 * (gps[0] + gps[1]);
  const Eigen::Vector2d direction_width = (gps[1] - gps[0]).normalized();
  gps[0] = mean_point_0_1 - 0.5 * width * direction_width;
  gps[1] = mean_point_0_1 + 0.5 * width * direction_width;

  const Eigen::Vector2d mean_point_2_3 = 0.5 * (gps[2] + gps[3]);
  gps[2] = mean_point_2_3 - 0.5 * width * direction_width;
  gps[3] = mean_point_2_3 + 0.5 * width * direction_width;

  // set length
  const Eigen::Vector2d mean_point_0_2 = 0.5 * (gps[0] + gps[2]);
  const Eigen::Vector2d direction_length = (gps[2] - gps[0]).normalized();
  gps[0] = mean_point_0_2 - 0.5 * length * direction_length;
  gps[2] = mean_point_0_2 + 0.5 * length * direction_length;

  const Eigen::Vector2d mean_point_1_2 = 0.5 * (gps[1] + gps[3]);
  gps[1] = mean_point_1_2 - 0.5 * length * direction_length;
  gps[3] = mean_point_1_2 + 0.5 * length * direction_length;



  //  ROS_DEBUG("pts: (%f,%f), (%f,%f), (%f,%f), (%f,%f)",
  //           roi_2d_ground_points[0].x,
  //           roi_2d_ground_points[0].y,
  //           roi_2d_ground_points[1].x,
  //           roi_2d_ground_points[1].y,
  //           roi_2d_ground_points[2].x,
  //           roi_2d_ground_points[2].y,
  //           roi_2d_ground_points[3].x,
  //           roi_2d_ground_points[3].y);

  ImagePoints area_roi;
  area_roi.reserve(4);
  for (const auto &gp : gps) {
    // make sure all points are in front of the obstacle (within field of view
    // on ground plane)
    const VehiclePoint ground_point_x_constrained(
        std::max(gp[0], roi_params_.field_of_vision_min_x), gp[1], 0);

    const ImagePoint ip =
        camera_transformation_->transformVehicleToImage(ground_point_x_constrained);
    // make sure roi is within image frame
    area_roi.emplace_back(
        boost::algorithm::clamp(ip[0], 0, features.image_complete.size().width),
        boost::algorithm::clamp(ip[1], 0, features.image_complete.size().height));
  }
  return area_roi;
}

ImagePoints ObstacleClassifierNew::getVolumeRoi(const ImagePoints &area_roi,
                                                const cv::Size &img_size) const {
  ImagePoints volume_roi;
  volume_roi.reserve(2 * area_roi.size());
  std::copy(area_roi.begin(), area_roi.end(), std::back_inserter(volume_roi));

  for (const ImagePoint &ip : area_roi) {
    const VehiclePoint vp_elevated =
        camera_transformation_->transformImageToGround(ip) +
        VehiclePoint(0, 0, roi_params_.roi_height_in_m);
    const ImagePoint ip_elevated =
        camera_transformation_->transformVehicleToImage(vp_elevated);
    // constrain point to valid image coordinates
    volume_roi.emplace_back(boost::algorithm::clamp(ip_elevated[0], 0, img_size.width),
                            boost::algorithm::clamp(ip_elevated[1], 0, img_size.height));
  }
  const std::vector<cv::Point> cv_roi = toCV(volume_roi);
  std::vector<cv::Point> convex_roi;
  cv::convexHull(cv_roi, convex_roi);
  return toEigen(convex_roi);
}

bool ObstacleClassifierNew::isLineAngleAcceptedAsVertical(const Line2d &vertical_candidate,
                                                          const Features &features) const {

  // angle measured to vertical in 2d image(-y direction)
  const double angle = vertical_candidate.angleTo(ImagePointExact(0, -1));

  if (features.roi_) {
    // classifier used in look at => expect no incline here.
    const Interval<double> angle_constaints_image(
        -model_params_.vertical_angle_tolerance_image,
        model_params_.vertical_angle_tolerance_image);
    return angle_constaints_image.insideInc(angle);
  }

  // normal road watcher case =>
  // expect possible incline in lane direction
  const VehiclePoint vp = camera_transformation_->transformImageToGround(
      common::eigen_utils::round(vertical_candidate.start));

  const VehiclePoint lane_direction_normalized =
      getLaneDirectionVector(features.middle_lane_polynomial, vp);

  const VehiclePoints vertical_direction_constaints = {
      (-std::sin(model_params_.max_incline_angle) * lane_direction_normalized +
       std::cos(model_params_.max_incline_angle) * VehiclePoint::UnitZ()),
      (std::sin(model_params_.max_incline_angle) * lane_direction_normalized +
       std::cos(model_params_.max_incline_angle) * VehiclePoint::UnitZ())};

  // Calculate elevated points for maximum incline / decline angles
  const ImagePointExact ip_angle_limit_1 = (camera_transformation_->transformVehicleToImageExact(
      vertical_direction_constaints[0] + vp));
  const ImagePointExact ip_angle_limit_2 = (camera_transformation_->transformVehicleToImageExact(
      vertical_direction_constaints[1] + vp));

  // find angle constraints for vertical
  Interval<double> angle_constaints_image(
      std::acos(ImagePointExact(0, -1).dot(
          (ip_angle_limit_1 - vertical_candidate.start).normalized())),
      std::acos(ImagePointExact(0, -1).dot(
          (ip_angle_limit_2 - vertical_candidate.start).normalized())));
  angle_constaints_image.sort();
  // add angle toelreances in 2d image independent of incline
  angle_constaints_image.start =
      angle_constaints_image.start - model_params_.vertical_angle_tolerance_image;
  angle_constaints_image.end =
      angle_constaints_image.end + model_params_.vertical_angle_tolerance_image;

  //  ROS_DEBUG(
  //      "P = (%f, %f), Angle Constraints vertical [%f, "
  //      "%f], actual angle = %f",
  //      vertical_candidate.start[0],
  //      vertical_candidate.start[1],
  //      common::toDegree(angle_constaints_image.start),
  //      common::toDegree(angle_constaints_image.end),
  //      common::toDegree(angle));

  return angle_constaints_image.insideInc(angle);
}

bool ObstacleClassifierNew::isGroundLine(const Line2d &ground_line_candidate,
                                         const Line2d &ref_vertical_b2t_oriented) const {

  const boost::optional<ImagePointExact> intersection =
      ground_line_candidate.intersectInfiniteRange(ref_vertical_b2t_oriented);
  // check if horizontal is more a gnd line than a top line
  return (intersection &&
          ref_vertical_b2t_oriented.squaredDistanceToStart(intersection.get()) <
              ref_vertical_b2t_oriented.squaredDistanceToEnd(intersection.get()));
}

VehiclePoint ObstacleClassifierNew::projectOnPolynomial(const common::DynamicPolynomial &middle_lane_polynomial,
                                                        const VehiclePoint &point) const {
  return utils::findLotfusspunkt(middle_lane_polynomial,
                                 point,
                                 foot_finder_params_.epsilon_newton_method,
                                 foot_finder_params_.max_iterations_newton_method);
}

VehiclePoint ObstacleClassifierNew::getLaneDirectionVector(const common::DynamicPolynomial &middle_lane_polynomial,
                                                           const VehiclePoint &ground_point) const {
  const VehiclePoint lp = projectOnPolynomial(middle_lane_polynomial, ground_point);
  return to3D(common::tangent(middle_lane_polynomial, lp[0]));
}

VehiclePoint ObstacleClassifierNew::getPerpendicularLaneVector(const VehiclePoint &ground_point,
                                                               const Features &features,
                                                               const bool &right_not_left_lane) const {
  const VehiclePoint lane_direction =
      getLaneDirectionVector(features.middle_lane_polynomial, ground_point);
  // Note x-component should be >0 as road going in same direction as
  // vehicle
  return right_not_left_lane
             ? VehiclePoint(lane_direction[1], -lane_direction[0], 0)
             : VehiclePoint(-lane_direction[1], lane_direction[0], 0);
}

/*############################################
 *###### Shape Extraction Methods ##########
 *############################################*/

ScanLines ObstacleClassifierNew::generateScanLines(const cv::Rect &rect,
                                                   const double edge_angle,
                                                   double &out_scan_line_spacing,
                                                   ImagePointExact &out_scan_direction) const {
  // Note: precond: 0 <= edge_angle <= pi !
  const double width = static_cast<double>(rect.size().width);
  const double height = static_cast<double>(rect.size().height);

  const double diagonal = std::sqrt(width * width + height * height);
  const double scanning_diameter =
      (edge_angle <= 0.5 * M_PI)
          ? diagonal * std::cos(edge_angle - std::atan(width / height))
          : diagonal * std::cos(0.5 * M_PI - edge_angle + std::atan(height / width));

  const std::size_t number_of_scan_lines = static_cast<std::size_t>(
      std::ceil(fp_detect_params_.scan_line_density * scanning_diameter));
  const double scan_line_spacing = scanning_diameter / number_of_scan_lines;
  ScanLines scan_lines;
  scan_lines.reserve(number_of_scan_lines);

  const ImagePointExact expected_edge_direction(std::sin(edge_angle), std::cos(edge_angle));
  // scanning from left,bottom to right top of rectangular window
  const ImagePointExact scan_direction(std::cos(edge_angle), -std::sin(edge_angle));

  const ImagePointExact root_point =
      (edge_angle <= 0.5 * M_PI)
          ? ImagePointExact(rect.tl().x, rect.tl().y)
          : ImagePointExact(rect.tl().x, (rect.tl().y + rect.size().height));

  for (std::size_t i = 0; i < number_of_scan_lines; ++i) {
    ImagePointExact p0 = root_point + static_cast<double>(i) * scan_line_spacing *
                                          expected_edge_direction;
    ScanLine scan_line(common::eigen_utils::round(p0 - 2.0 * diagonal * scan_direction),
                       common::eigen_utils::round(p0 + 2.0 * diagonal * scan_direction));
    if (ScanLine::clip(rect, scan_line) &&
        length(scan_line) >= fp_detect_params_.min_scan_line_length_in_pxl) {
      scan_lines.emplace_back(scan_line);
    }
  }

  out_scan_line_spacing = scan_line_spacing;
  out_scan_direction = scan_direction;
  return scan_lines;
}

ImagePoints ObstacleClassifierNew::getFeaturePoints(const cv::Mat &image_complete,
                                                    cv::InputArray &roi_defining_points,
                                                    const ScanLines &scan_lines,
                                                    const DetectionMode &mode) const {
  auto ref_func = step_detection::createReferenceFunction(
      static_cast<std::size_t>(fp_detect_params_.step_func_size));
  const bool use_abs = (mode == DetectionMode::ALL_EDGES);
  if (mode == DetectionMode::WHITE_TO_BLACK) {
    ref_func = step_detection::invertReferenceFunction(ref_func);
  }
  ImagePoints feature_points;
  for (const auto &scan_line : scan_lines) {
    const ImagePoints detected_points = step_detection::detectStep(
        image_complete, scan_line, ref_func, fp_detect_params_.step_thld, use_abs);
    // check if detected points are within roi and outside of ego vehicle
    for (const auto &detected_point : detected_points) {
      if (cv::pointPolygonTest(roi_defining_points, toCV(detected_point), false) >= 0 &&
          cv::pointPolygonTest(ego_vehicle_.asInputArray(), toCV(detected_point), false) < 0) {
        feature_points.emplace_back(detected_point);
      }
    }
  }
  return feature_points;
}

cv::Rect ObstacleClassifierNew::getExtendedRoiRect(const std::vector<cv::Point> &convex_roi,
                                                   const int expand_pixel_even,
                                                   const cv::Size &img_size) const {
  const cv::Rect bounding_rect =
      (cv::boundingRect(convex_roi) & cv::Rect(cv::Point(0, 0), img_size));  // crop rect to image

  /// extend rect depending on the set_function length, so that it is
  /// ensured
  /// that points until the edges of the roi may be detected
  const cv::Size expand = cv::Size(expand_pixel_even, expand_pixel_even);
  const cv::Point centering = cv::Point(expand_pixel_even / 2, expand_pixel_even / 2);
  return ((bounding_rect + expand - centering) & cv::Rect(cv::Point(0, 0), img_size));
}

void ObstacleClassifierNew::getFrontShapeFeaturePoints(const Features &features,
                                                       const ImagePoints &area_roi,
                                                       ImagePoints &fp_bottom_out,
                                                       ImagePoints &fp_vertical_left_out,
                                                       ImagePoints &fp_vertical_right_out) const {

  // calculate roi
  const ImagePoints volume_roi = getVolumeRoi(area_roi, features.image_complete.size());

  const std::vector<cv::Point> cv_area_roi = toCV(area_roi);
  const std::vector<cv::Point> cv_volume_roi = toCV(volume_roi);

  /// extend rect depending on the set_function length, so that it is
  /// ensured that points close to the edges of the roi will be detected
  const cv::Rect rect_area_roi = getExtendedRoiRect(
      cv_area_roi, 2 * fp_detect_params_.step_func_size, features.image_complete.size());
  const cv::Rect rect_volume_roi =
      getExtendedRoiRect(cv_volume_roi,
                         2 * fp_detect_params_.step_func_size,
                         features.image_complete.size());

  double out_scan_line_spacing;
  ImagePointExact out_scan_direction;

  const ScanLines scan_lines_b = generateScanLines(
      rect_area_roi, M_PI_2, out_scan_line_spacing, out_scan_direction);

  const ScanLines scan_lines_v =
      generateScanLines(rect_volume_roi, 0, out_scan_line_spacing, out_scan_direction);

  // detect feature points
  fp_bottom_out = getFeaturePoints(
      features.image_complete, toInputArray(cv_area_roi), scan_lines_b, DetectionMode::BLACK_TO_WHITE);

  fp_vertical_left_out = getFeaturePoints(features.image_complete,
                                          toInputArray(cv_volume_roi),
                                          scan_lines_v,
                                          DetectionMode::BLACK_TO_WHITE);

  fp_vertical_right_out = getFeaturePoints(features.image_complete,
                                           toInputArray(cv_volume_roi),
                                           scan_lines_v,
                                           DetectionMode::WHITE_TO_BLACK);

  // remove lane line points
  const boost::optional<PolynomialWithXRange> poly_left =
      ObstacleClassifierNew::getPolynomial(features.points_left);
  const boost::optional<PolynomialWithXRange> poly_middle =
      ObstacleClassifierNew::getPolynomial(features.points_middle);
  const boost::optional<PolynomialWithXRange> poly_right =
      ObstacleClassifierNew::getPolynomial(features.points_right);

  erasePointsOnLaneLines(
      fp_bottom_out, poly_left, poly_middle, poly_right, fp_detect_params_.lane_line_removal_thld_in_m);
  erasePointsOnLaneLines(fp_vertical_left_out,
                         poly_left,
                         poly_middle,
                         poly_right,
                         fp_detect_params_.lane_line_removal_thld_in_m);
  erasePointsOnLaneLines(fp_vertical_right_out,
                         poly_left,
                         poly_middle,
                         poly_right,
                         fp_detect_params_.lane_line_removal_thld_in_m);
}

boost::optional<PolynomialWithXRange> ObstacleClassifierNew::getPolynomial(const VehiclePoints &points) const {
  if (points.size() <= 4 ||
      common::numberOfDistinctXValues(points) <= static_cast<std::size_t>(4)) {
    return boost::none;
  }
  const common::DynamicPolynomial polynoimal =
      common::fitToPoints(points, common::polynomial::PolynomialDegrees::Quartic);

  // find closest and furthest (in Vehicle coordinate x-direction) lane line
  // points
  const auto min_max_pair_it = std::minmax_element(
      points.begin(), points.end(), [](const VehiclePoint &p1, const VehiclePoint &p2) {
        return (p1[0] < p2[0]);
      });

  const Interval<double> x_range((*min_max_pair_it.first)[0],
                                 (*min_max_pair_it.second)[0]);

  return boost::make_optional<PolynomialWithXRange>(PolynomialWithXRange(polynoimal, x_range));
}

void ObstacleClassifierNew::erasePointsOnLaneLines(
    ImagePoints &in_out_ips,
    const boost::optional<PolynomialWithXRange> poly_left,
    const boost::optional<PolynomialWithXRange> poly_middle,
    const boost::optional<PolynomialWithXRange> poly_right,
    const double rem_distance_m_thld) const {

  const auto is_on_lane_polynomial = [this, &rem_distance_m_thld](
                                         const boost::optional<PolynomialWithXRange> poly,
                                         const VehiclePoint &vp) {
    if (!poly) {
      return false;
    }
    const VehiclePoint lp = projectOnPolynomial(poly->polynomial, vp);
    const VehiclePoint lane_line_point(lp[0], poly->polynomial.evaluate(lp[0]), 0);
    return ((lane_line_point - vp).squaredNorm() < rem_distance_m_thld * rem_distance_m_thld);
  };

  const auto is_on_any_lane_polynomial =
      [this, &poly_left, &poly_middle, &poly_right, &is_on_lane_polynomial](
          const ImagePoint &ip) {
        const VehiclePoint vp = camera_transformation_->transformImageToGround(ip);
        return (is_on_lane_polynomial(poly_left, vp) ||
                is_on_lane_polynomial(poly_middle, vp) ||
                is_on_lane_polynomial(poly_right, vp));
      };
  boost::remove_erase_if(in_out_ips, is_on_any_lane_polynomial);
}

void ObstacleClassifierNew::erasePointsOnLaneLines(
    VehiclePoints &in_out_ips,
    const boost::optional<PolynomialWithXRange> poly_left,
    const boost::optional<PolynomialWithXRange> poly_middle,
    const boost::optional<PolynomialWithXRange> poly_right,
    const double rem_distance_m_thld) const {

  const auto is_on_lane_polynomial = [this, &rem_distance_m_thld](
                                         const boost::optional<PolynomialWithXRange> poly,
                                         const VehiclePoint &vp) {
    if (!poly) {
      return false;
    }
    const VehiclePoint lp = projectOnPolynomial(poly->polynomial, vp);
    const VehiclePoint lane_line_point(lp[0], poly->polynomial.evaluate(lp[0]), 0);
    return ((lane_line_point - vp).squaredNorm() < rem_distance_m_thld * rem_distance_m_thld);
  };

  const auto is_on_any_lane_polynomial =
      [&poly_left, &poly_middle, &poly_right, &is_on_lane_polynomial](const VehiclePoint &vp) {
        return (is_on_lane_polynomial(poly_left, vp) ||
                is_on_lane_polynomial(poly_middle, vp) ||
                is_on_lane_polynomial(poly_right, vp));
      };
  boost::remove_erase_if(in_out_ips, is_on_any_lane_polynomial);
}

// PRE_COND: index_end-index_begin >=2
std::pair<std::size_t, std::size_t> ObstacleClassifierNew::get2RandomIndices(
    const std::size_t index_begin, const std::size_t index_end) const {
  // assert precondition in debug mode
  assert(index_end > index_begin);
  assert(index_end - index_begin >= 2);
  const auto rand_index1 =
      static_cast<std::size_t>(std::rand()) % (index_end - index_begin) + index_begin;
  auto rand_index2 =
      static_cast<std::size_t>(std::rand()) % (index_end - index_begin - 1) + index_begin;
  if (rand_index2 >= rand_index1) {
    rand_index2++;
  }
  return {rand_index1, rand_index2};
}

boost::optional<Line2d> ObstacleClassifierNew::getRandomGroundLine(
    const ImagePointsExact &points, const Line2d &ref_vertical_b2t_oriented) const {

  return getRandomCondTrueLine(
      points,
      [this, &ref_vertical_b2t_oriented](const Line2d &ground_line_candidate) {
        return isGroundLine(ground_line_candidate, ref_vertical_b2t_oriented);
      },
      ransac_params_.max_it_rand_L_lines_pts_select);
}

boost::optional<Line2d> ObstacleClassifierNew::getRandomVertical(
    const ImagePointsExact &points, const Features &features) const {

  boost::optional<Line2d> vertical = getRandomCondTrueLine(
      points,
      [this, &features](const Line2d &ground_line_candidate) {
        return isLineAngleAcceptedAsVertical(ground_line_candidate, features);
      },
      ransac_params_.max_it_rand_L_lines_pts_select);
  if (vertical) {
    vertical->sortForDimensionInverse(1);
  }
  return vertical;
}

bool ObstacleClassifierNew::randFindAndSet2ndGroundLine(const ImagePointsExact &points,
                                                        ObstacleFrontModel &model,
                                                        const Features &features) const {

  return static_cast<bool>(getRandomCondTrueLine(
      points,
      [this, &model, &features](const Line2d &line_candidate) mutable {
        return model.extendModelWith2ndGroundLine(
            line_candidate, model_params_, features.image_complete.size(), camera_transformation_);
      },
      ransac_params_.max_it_2nd_gnd_model_extension));
}

boost::optional<ObstacleFrontModel> ObstacleClassifierNew::fitObstacleFrontModel(
    const ImagePoints &bottom_pts,
    const ImagePoints &vertical_left_pts,
    const ImagePoints &vertical_right_pts,
    const Features &features) const {

  // copy points as exact inagepoints
  ImagePointsExact b_pts, vl_pts, vr_pts;
  imagePointsToImagePointsExact(bottom_pts, b_pts);
  imagePointsToImagePointsExact(vertical_left_pts, vl_pts);
  imagePointsToImagePointsExact(vertical_right_pts, vr_pts);

  boost::optional<ObstacleFrontModel> best_model = boost::none;
  ObstacleFrontModel model;

  // ransac loop
  for (std::size_t it = 0; it < ransac_params_.max_iteration; it++) {
    // ROS_DEBUG("Ransac Loop iteration %lu, ", it);
    model.reset();

    // find vertical either from left ot right dataset
    const double prob_vl_data = static_cast<double>(vl_pts.size()) /
                                static_cast<double>(vl_pts.size() + vr_pts.size());
    bool randSelectLeftData = prob_true(prob_vl_data);
    const ImagePointsExact *v1_pts = randSelectLeftData ? &vl_pts : &vr_pts;

    boost::optional<Line2d> vertical1 = getRandomVertical(*v1_pts, features);
    if (!vertical1) {
      // first data set failed to produce valid vertical
      //=>try second dataset
      randSelectLeftData = !randSelectLeftData;
      v1_pts = randSelectLeftData ? &vl_pts : &vr_pts;
      vertical1 = getRandomVertical(*v1_pts, features);
    }
    if (!vertical1) {
      // ROS_DEBUG("Reject Model => No Vertical1 found.");
      break;
    }

    // calculate consensus set and fit vertical
    ImagePointsExact v_consensus, v_non_consensus;
    vertical1->calcConsensusSetInfLine(
        *v1_pts, model_params_.distance_thld_pixel, v_consensus, v_non_consensus);
    if (v_consensus.size() < ransac_params_.min_data_set_size) {
      // vertical line supported by too few points
      // => model will never be accpeted
      continue;
    }

    // random ground line candidate: either select b_pts data or same v_data
    // that was not consensus with the previously defined vertical
    const double prob_b_data_select =
        static_cast<double>(b_pts.size()) /
        static_cast<double>(b_pts.size() + v_non_consensus.size());
    bool rand_select_b_data = prob_true(prob_b_data_select);
    const ImagePointsExact *b_pts_selected = rand_select_b_data ? &b_pts : &v_non_consensus;

    boost::optional<Line2d> ground_line0 =
        getRandomGroundLine(*b_pts_selected, vertical1.get());

    if (!ground_line0) {
      rand_select_b_data = !rand_select_b_data;
      b_pts_selected = rand_select_b_data ? &b_pts : &v_non_consensus;
      ground_line0 = getRandomGroundLine(*b_pts_selected, vertical1.get());
    }
    if (!ground_line0) {
      // ROS_DEBUG("Reject Model => No horizontal bottom found");
      break;
    }

    // generate model L
    if (!model.generateModelL(ground_line0.get(),
                              vertical1.get(),
                              features.image_complete.size(),
                              model_params_,
                              camera_transformation_,
                              randSelectLeftData)) {
      continue;
    }

    // calculate set sizes and eval model score
    ImagePointsExact out1_not_b1_inf_line;
    model.calcConsensusSet(*b_pts_selected, *v1_pts, out1_not_b1_inf_line, model_params_, 0);
    model.evaluateScore(features.image_complete, model_params_, camera_transformation_);

    // In case obsacle is viewed from the side => extend model with 2nd ground
    // line
    if (model.getMostRecentScore() > ransac_params_.min_model_score_thld &&
        model.isSecondGroundLineClearlyVisible(
            model_params_.gndline2_visible_angle_thld, camera_transformation_)) {

      // extend model with 2nd ground line to improve obstacle localization
      // make a copy for a current model in case no valid extension is found
      ObstacleFrontModel m = model;
      // given the previously selected L-Model a 2nd ground line should be
      // visible and improve the fit. If this is not the case howwever the
      // model score should be penalzized because this indicates a bad
      // fit.
      model.setPenaltyFactor(model_params_.incomplete_model_penalty);

      // select dataset for 2nd ground_line
      // assign v2_pts the vertical dataset which was not used for the primary
      // L-model
      const ImagePointsExact *v2_pts = randSelectLeftData ? &vr_pts : &vl_pts;
      // try to fit 2nd ground line with horizontal dataset (however without the
      // points consistent with the first ground line
      const ImagePointsExact *ground2_data = &out1_not_b1_inf_line;
      bool b2_set = randFindAndSet2ndGroundLine(*ground2_data, m, features);
      if (!b2_set) {
        // in case no ground line was found try the second vertical dataset
        // This might be useful if the 2nd gnd line is groing into depth and
        // therfore might have a similar 2d angle as the vertical line.
        ground2_data = v2_pts;
        b2_set = randFindAndSet2ndGroundLine(*ground2_data, m, features);
      }
      if (b2_set) {
        // calculate consensus set and score of extended model m
        m.calcConsensusSet(*ground2_data, model_params_, 1);
        m.evaluateScore(features.image_complete, model_params_, camera_transformation_);
        // only keep extended model and score if it actually performs
        // better
        if (m.getMostRecentScore() >= model.getMostRecentScore() &&
            m.getFirstSubModelRecentScore() >= model.getFirstSubModelRecentScore()) {
          model = m;
        }
      }
    }

    if ((!best_model || model.getMostRecentScore() > best_model->getMostRecentScore()) &&
        model.getMostRecentScore() > ransac_params_.min_model_score_thld) {
      best_model = boost::make_optional<ObstacleFrontModel>(model);
    }
  }
  return best_model;
}

inline auto modelSupport(const LineModel &model, const double min_dist) {
  return [&model, min_dist](const ImagePointExact &p) {
    return std::fabs(model.distance(p)) < min_dist;
  };
};

ScanLine ObstacleClassifierNew::generateBaseHullEstmateScanLine(const Line3d &model_ground_line,
                                                                double z0) const {

  const VehiclePoint start = model_ground_line.end -
                             fp_detect_params_.base_estimate_scan_line_edge_overlap_in_m *
                                 model_ground_line.directionNormalized() +
                             z0 * Eigen::Vector3d::UnitZ();

  const double scan_line_length =
      obstacle_params_.max_width - (start - model_ground_line.start).norm();
  return ScanLine(camera_transformation_->transformVehicleToImage(start),
                  camera_transformation_->transformVehicleToImage(
                      start + scan_line_length * model_ground_line.directionNormalized()));
}

boost::optional<VehiclePoint> ObstacleClassifierNew::estimateObstacleEndPoint(
    const Line3d &model_ground_line, const Features &features) const {

  auto ref_func = step_detection::createReferenceFunction(
      static_cast<std::size_t>(fp_detect_params_.step_func_size));
  ref_func = step_detection::invertReferenceFunction(ref_func);

  VehiclePoints feature_points;

  for (std::size_t i = 1; i <= fp_detect_params_.base_estimate_number_of_scan_lines; i++) {
    const double z0 = i * fp_detect_params_.base_estimate_scan_line_spacing_in_m;
    const ScanLine scan_line = generateBaseHullEstmateScanLine(model_ground_line, z0);

    const ImagePoints ips = step_detection::detectStep(
        features.image_complete, scan_line, ref_func, fp_detect_params_.step_thld, false);

    if (!ips.empty()) {
      feature_points.emplace_back(transformImageToZPlane(ips[0], z0, camera_transformation_));
    }
  }

  // remove lane line points
  const boost::optional<PolynomialWithXRange> poly_left =
      ObstacleClassifierNew::getPolynomial(features.points_left);
  const boost::optional<PolynomialWithXRange> poly_middle =
      ObstacleClassifierNew::getPolynomial(features.points_middle);
  const boost::optional<PolynomialWithXRange> poly_right =
      ObstacleClassifierNew::getPolynomial(features.points_right);
  erasePointsOnLaneLines(
      feature_points, poly_left, poly_middle, poly_right, fp_detect_params_.lane_line_removal_thld_in_m);

  // average the points not on lane lines
  if (!feature_points.empty()) {
    VehiclePoint mean_point =
        std::accumulate(
            feature_points.begin(), feature_points.end(), VehiclePoint(0, 0, 0)) /
        static_cast<double>(feature_points.size());
    mean_point[2] = 0;  // get ground point
    return boost::make_optional<VehiclePoint>(mean_point);
  }
  return boost::none;
}


void ObstacleClassifierNew::setToObstacleDefaultSize(ObstacleVertices &vertices) const {
  // assumes the only detected vertx is at vertices.vertices[0]
  vertices.vertices[1] = vertices.vertices[0] +
                         obstacle_params_.default_size *
                             (vertices.vertices[1] - vertices.vertices[0]).normalized();
  vertices.vertices[3] = vertices.vertices[0] +
                         obstacle_params_.default_size *
                             (vertices.vertices[3] - vertices.vertices[0]).normalized();
  vertices.vertices[2] =
      vertices.vertices[1] + vertices.vertices[3] - vertices.vertices[0];
}

ObstacleVertices ObstacleClassifierNew::estimateBaseHullPolygon(const ObstacleFrontModel &model,
                                                                const Features &features) const {
  // Note: in case only either width or length of the obstacle can be estimated
  // the obstacle is expected to have square base area. I no estimation of the
  // obstacle size is possible it is assumed to have default square size.
  ObstacleVertices vertices = model.getGroundVertices(camera_transformation_);

  if (model.getModelType() == ObstacleFrontModel::ModelType::L) {
    const Line3d ground_line =
        model.getBottomLine(0).asGroundLineInVehicle(camera_transformation_);
    boost::optional<VehiclePoint> end_point =
        estimateObstacleEndPoint(ground_line, features);
    if (end_point) {
      // replace base hull vertices. asume a square base.
      const double side_length = (end_point.get() - ground_line.start).norm();
      // Note: This must match the way the vertices are writen in
      // model.getGroundVertices
      vertices.vertices[1] = end_point.get();
      vertices.vertices_detection_state[1] = Obstacle::DetectionState::VERTEX_ESTIMATED;
      vertices.vertices[3] =
          vertices.vertices[0] +
          side_length * (vertices.vertices[3] - vertices.vertices[0]).normalized();
      vertices.vertices[2] =
          end_point.get() +
          side_length * (vertices.vertices[3] - vertices.vertices[0]).normalized();
    } else {
      // set default  obstacle size
      setToObstacleDefaultSize(vertices);
    }
  }

  else if (model.getModelType() == ObstacleFrontModel::ModelType::L_2ND_GND_LINE) {
    if (model.getBottomLine(0).end != model.getBottomLine(1).start) {
      // only one vertex detected => estimate both directions
      const Line3d ground_line0 =
          model.getBottomLine(0).asGroundLineInVehicle(camera_transformation_);
      boost::optional<VehiclePoint> end_point0 =
          estimateObstacleEndPoint(ground_line0, features);

      const Line3d ground_line1 =
          model.getBottomLine(1).asGroundLineInVehicle(camera_transformation_);
      boost::optional<VehiclePoint> end_point1 =
          estimateObstacleEndPoint(ground_line1, features);

      if (end_point0 && end_point1) {
        vertices.vertices[1] = end_point0.get();
        vertices.vertices_detection_state[1] = Obstacle::DetectionState::VERTEX_ESTIMATED;
        vertices.vertices[3] = end_point1.get();
        vertices.vertices_detection_state[3] = Obstacle::DetectionState::VERTEX_ESTIMATED;
        vertices.vertices[2] =
            vertices.vertices[1] + vertices.vertices[3] - vertices.vertices[0];
      } else if (end_point0) {
        // assume square obstacle base
        const double side_length0 = (end_point0.get() - ground_line0.start).norm();
        vertices.vertices[1] = end_point0.get();
        vertices.vertices_detection_state[1] = Obstacle::DetectionState::VERTEX_ESTIMATED;
        vertices.vertices[3] = vertices.vertices[0] +
                               side_length0 * ground_line1.directionNormalized();
        vertices.vertices[2] =
            vertices.vertices[1] + vertices.vertices[3] - vertices.vertices[0];
      } else if (end_point1) {
        const double side_length1 = (end_point1.get() - ground_line1.start).norm();
        vertices.vertices[3] = end_point1.get();
        vertices.vertices_detection_state[3] = Obstacle::DetectionState::VERTEX_ESTIMATED;
        vertices.vertices[1] = vertices.vertices[0] +
                               side_length1 * ground_line0.directionNormalized();
        vertices.vertices[2] =
            vertices.vertices[1] + vertices.vertices[3] - vertices.vertices[0];
      } else {

        // only vertex[0] detected / estimated
        setToObstacleDefaultSize(vertices);
      }
    } else {
      // 2 vertices are detected => only base estimation in one direction
      // necesarry
      const Line3d ground_line1 =
          model.getBottomLine(1).asGroundLineInVehicle(camera_transformation_);
      boost::optional<VehiclePoint> end_point1 =
          estimateObstacleEndPoint(ground_line1, features);
      if (end_point1) {
        vertices.vertices[2] = end_point1.get();
        vertices.vertices_detection_state[2] = Obstacle::DetectionState::VERTEX_ESTIMATED;
        vertices.vertices[3] =
            vertices.vertices[0] + vertices.vertices[2] - vertices.vertices[1];
      }
    }
  }
  return vertices;
}

/*******************************************************************
 **************  Free Functions ***********************************
 *****************************************************************/
double length(const ScanLine &scan_line) {
  const ImagePoint diff = scan_line.end - scan_line.start;
  return diff.norm();
}

void imagePointsToImagePointsExact(const ImagePoints &ips, ImagePointsExact &out_exact_points) {
  out_exact_points.reserve(ips.size());
  for (const ImagePoint &p : ips) {
    out_exact_points.emplace_back(ImagePointExact(p[0], p[1]));
  }
}

cv::Rect getExpandedImgCroppedRect(const std::vector<cv::Point> &convex_poly,
                                   const int expand_x,
                                   const int expand_y,
                                   const cv::Size &img_size) {

  const cv::Rect bounding_rect = cv::boundingRect(convex_poly);
  const cv::Size expand = cv::Size(2 * expand_x, 2 * expand_y);
  const cv::Point centering = cv::Point(expand_x, expand_y);
  return ((bounding_rect + expand - centering) & cv::Rect(cv::Point(0, 0), img_size));  // crop rect to image
}

bool prob_true(const double p) {
  return std::rand() < p * (static_cast<double>(RAND_MAX) + 1.0);
}

double linearInterpolFctPts(const double value,
                            const std::vector<Eigen::Vector2d> &sorted_function_pts) {

  if (sorted_function_pts.size() == 1) {
    // objective function is a constant
    return (sorted_function_pts[0])[1];
  }

  const auto it_p2 =
      std::find_if(sorted_function_pts.begin(),
                   sorted_function_pts.end(),
                   [&value](const Eigen::Vector2d &p) { return p[0] >= value; });
  if (it_p2 == sorted_function_pts.begin()) {
    return (*it_p2)[1];
  }
  if (it_p2 == sorted_function_pts.end()) {
    return (*(it_p2 - 1))[1];
  }

  const auto it_p1 = it_p2 - 1;
  const double slope = ((*it_p2)[1] - (*it_p1)[1]) / ((*it_p2)[0] - (*it_p1)[0]);
  const double delta_x = value - (*it_p1)[0];
  const double offset = (*it_p1)[1];
  return (slope * delta_x + offset);
}

VehiclePoint transformImageToZPlane(const ImagePoint &ip,
                                    const double z0,
                                    const common::CameraTransformation *const camera_transformation) {

  const Eigen::Matrix3d &R = camera_transformation->getRotationMatrix();
  const Eigen::Vector3d &t = camera_transformation->getTranslationVector();
  const Eigen::Matrix3d &A = camera_transformation->getIntrinsicCalibrationMatrix();
  Eigen::Matrix3d r1_r2_t;
  r1_r2_t << R.col(0), R.col(1), (t + z0 * R.col(2));
  const Eigen::Matrix3d H = A * r1_r2_t;
  const Eigen::Matrix3d H_inverse = H.fullPivHouseholderQr().inverse();

  const Eigen::Vector3d point_homogenous =
      H_inverse * common::toHomogenous(Eigen::Vector2d(ip.cast<double>()));
  return (to3D(to2D(point_homogenous) / point_homogenous.z()) +
          z0 * Eigen::Vector3d::UnitZ());
}

}  // namespace road_object_detection
