#include "feature_extraction.h"
#include <common/macros.h>

THIRD_PARTY_HEADERS_BEGIN
#include <algorithm>
#include <ros/ros.h>
#include <cmath>
#include <cstdlib>
#include <opencv2/core/core.hpp>
#include <boost/optional.hpp>

#include <boost/range/combine.hpp>
#include <boost/range/empty.hpp>
#include <boost/range/size.hpp>
#include <boost/range/algorithm/max_element.hpp>
THIRD_PARTY_HEADERS_END

#include "opencv_eigen_conversions.h"
#include "common/basic_statistics.h"
#include "common/math.h"
#include "common/console_colors.h"
#include "common/eigen_functors.h"
#include "common/adaptors.h"
#include "common/normal_shift.h"
#include "../utils/foot_finder.h"
#include "common/pca_eigen.h"

#include <opencv2/highgui/highgui.hpp>

#define WHITE_PIXEL static_cast<uchar>(255)  // Value of white pixels in cv:Mat

ScanLine transformGroundToROI(const ROIBirdsViewTransformation* const trafo,
                              const VehicleScanLine& l) {
  return {trafo->transformGroundToROI(l.start), trafo->transformGroundToROI(l.end)};
}

const ParameterString<double> FeatureExtraction::SEGMENT_BOUNDARIES_OUTER_LEFT(
    "feature_extraction/segment_boundaries/outer_left");
const ParameterString<double> FeatureExtraction::SEGMENT_BOUNDARIES_LEFT(
    "feature_extraction/segment_boundaries/left");
const ParameterString<double> FeatureExtraction::SEGMENT_BOUNDARIES_MIDDLE(
    "feature_extraction/segment_boundaries/middle");
const ParameterString<double> FeatureExtraction::SEGMENT_BOUNDARIES_RIGHT(
    "feature_extraction/segment_boundaries/right");
const ParameterString<double> FeatureExtraction::SEGMENT_BOUNDARIES_NEAR(
    "feature_extraction/segment_boundaries/near");
const ParameterString<double> FeatureExtraction::SEGMENT_BOUNDARIES_FAR(
    "feature_extraction/segment_boundaries/far");
const ParameterString<double> FeatureExtraction::GENERAL_SIGMA_FACTOR(
    "feature_extraction/general/sigma_factor");
const ParameterString<double> FeatureExtraction::GENERAL_SCAN_LINE_STEP_SIZE(
    "feature_extraction/general/scan_line_step_size");
const ParameterString<double> FeatureExtraction::GENERAL_SCAN_LINE_HALF_WIDTH(
    "feature_extraction/general/scan_line_half_size");
const ParameterString<double> FeatureExtraction::GENERAL_HISTOGRAM_BIN_SIZE(
    "feature_extraction/general/histogram_bin_size");
const ParameterString<double> FeatureExtraction::GENERAL_MAX_WIDTH_FEATURE_PAIR(
    "feature_extraction/general/max_width_feature_pair");
const ParameterString<double> FeatureExtraction::GENERAL_MAX_ABS_DOT_HYP_NORMAL_MODEL(
    "feature_extraction/general/max_abs_dot_hyp_norm_model");
const ParameterString<double> FeatureExtraction::GENERAL_EPS(
    "feature_extraction/general/eps");
const ParameterString<double> FeatureExtraction::POLYNOMIAL_MIN_X(
    "feature_extraction/scan_line_creation/discretization_params/min_x");
const ParameterString<double> FeatureExtraction::POLYNOMIAL_MAX_X(
    "feature_extraction/scan_line_creation/discretization_params/max_x");
const ParameterString<double> FeatureExtraction::POLYNOMIAL_STEP(
    "feature_extraction/scan_line_creation/discretization_params/step_x");

void FeatureExtraction::registerFeatureExtractionParameter(ParameterInterface* parameters_ptr) {
  parameters_ptr->registerParam(SEGMENT_BOUNDARIES_OUTER_LEFT);
  parameters_ptr->registerParam(SEGMENT_BOUNDARIES_LEFT);
  parameters_ptr->registerParam(SEGMENT_BOUNDARIES_MIDDLE);
  parameters_ptr->registerParam(SEGMENT_BOUNDARIES_RIGHT);
  parameters_ptr->registerParam(SEGMENT_BOUNDARIES_NEAR);
  parameters_ptr->registerParam(SEGMENT_BOUNDARIES_FAR);
  parameters_ptr->registerParam(GENERAL_SIGMA_FACTOR);
  parameters_ptr->registerParam(GENERAL_SCAN_LINE_STEP_SIZE);
  parameters_ptr->registerParam(GENERAL_SCAN_LINE_HALF_WIDTH);
  parameters_ptr->registerParam(GENERAL_HISTOGRAM_BIN_SIZE);
  parameters_ptr->registerParam(GENERAL_MAX_WIDTH_FEATURE_PAIR);
  parameters_ptr->registerParam(GENERAL_MAX_ABS_DOT_HYP_NORMAL_MODEL);
  parameters_ptr->registerParam(GENERAL_EPS);
  parameters_ptr->registerParam(POLYNOMIAL_MIN_X);
  parameters_ptr->registerParam(POLYNOMIAL_MAX_X);
  parameters_ptr->registerParam(POLYNOMIAL_STEP);
}


FeatureExtraction::FeatureExtraction(ParameterInterface* parameters_ptr,
                                     ROIBirdsViewTransformation* view_transform,
                                     EgoVehicle* ego_vehicle)
    : view_transform(view_transform),
      ego_vehicle_(ego_vehicle),
      parameters_ptr_(parameters_ptr) {
  registerFeatureExtractionParameter(parameters_ptr);
}

LineVehiclePoints FeatureExtraction::extractLineMarkings(const cv::Mat& image) const {
  const LineVehiclePoints line_ground_points = getSegmentedGroundPoints(image);

  ROS_DEBUG("left_ground points: %zu", line_ground_points[LINESPEC_LEFT].size());
  ROS_DEBUG("middle_ground points: %zu", line_ground_points[LINESPEC_MIDDLE].size());
  ROS_DEBUG("right_ground points: %zu", line_ground_points[LINESPEC_RIGHT].size());

  LineVehiclePoints line_points;
  for (const auto l_spec : {LINESPEC_LEFT, LINESPEC_RIGHT, LINESPEC_MIDDLE}) {
    line_points[l_spec] = extractLinePoints(image, line_ground_points[l_spec]);
  }
  return line_points;
}

LineVehiclePoints FeatureExtraction::extractLineMarkings(const cv::Mat& image,
                                                         const LaneModel& line_model) const {
  LineVehiclePoints line_points;
  for (const auto l_spec : {LINESPEC_LEFT, LINESPEC_RIGHT, LINESPEC_MIDDLE}) {
    ROS_DEBUG_STREAM("extracting points for: " << toString(l_spec));
    const auto reference = l_spec == LINESPEC_MIDDLE ? LINESPEC_RIGHT : l_spec;
    line_points[l_spec] = extractLinePointsByLine(
        image, line_model[l_spec], line_model[reference], l_spec);
  }
  return line_points;
}

VehiclePoints FeatureExtraction::extractLinePoints(
    const cv::Mat& image,
    const VehiclePoints& ground_points,
    const boost::optional<common::DynamicPolynomial>& ref_line,
    const boost::optional<LineSpec> l_spec) const {

  const ScanLines scan_lines = createScanLines(ground_points);
  const ImagePoints image_points =
      extractImagePointsByScanLines(image, scan_lines, ref_line, l_spec);
  return view_transform->transformROIToGround(image_points);
}

VehiclePoints FeatureExtraction::extractLinePointsByLine(
    const cv::Mat& image,
    const boost::optional<common::DynamicPolynomial>& line,
    const boost::optional<common::DynamicPolynomial>& ref_line,
    const boost::optional<LineSpec> l_spec) const {
  return line.is_initialized() ? extractLinePointsByLine(image, *line, ref_line, l_spec)
                               : VehiclePoints();
}

VehiclePoints FeatureExtraction::extractLinePointsByLine(
    const cv::Mat& image,
    const common::DynamicPolynomial& line,
    const boost::optional<common::DynamicPolynomial>& ref_line,
    const boost::optional<LineSpec> l_spec) const {
  const ScanLines scan_lines = createScanLines(line);
  const ImagePoints image_points =
      extractImagePointsByScanLines(image, scan_lines, ref_line, l_spec);
  return view_transform->transformROIToGround(image_points);
}

ImagePoints FeatureExtraction::extractImagePointsByScanLines(
    const cv::Mat& image,
    const ScanLines& scan_lines,
    const boost::optional<common::DynamicPolynomial>& ref_line,
    const boost::optional<LineSpec> l_spec) const {
  ImagePoints image_points;
  image_points.reserve(scan_lines.size());
  const CenterOfConnectedPointsParam params{
      parameters_ptr_->getParam(GENERAL_MAX_WIDTH_FEATURE_PAIR),
      parameters_ptr_->getParam(POLYNOMIAL_STEP),
      parameters_ptr_->getParam(GENERAL_MAX_ABS_DOT_HYP_NORMAL_MODEL),
      parameters_ptr_->getParam(GENERAL_EPS)};

  for (const ScanLine& scan_line : scan_lines) {
    getCenterOfConnectedLinePoints(image, scan_line, &image_points, params, ref_line, l_spec);
  }
  return image_points;
}

ScanLines FeatureExtraction::createScanLines(const common::DynamicPolynomial& line) const {
  const double scan_line_half_width =
      parameters_ptr_->getParam(GENERAL_SCAN_LINE_HALF_WIDTH);
  const common::DiscretizationParams discretization_params = {
      parameters_ptr_->getParam(POLYNOMIAL_MIN_X),
      parameters_ptr_->getParam(POLYNOMIAL_MAX_X),
      parameters_ptr_->getParam(POLYNOMIAL_STEP)};

  VehiclePoints left, right;
  common::normalShift(line, -scan_line_half_width, discretization_params, &left);
  common::normalShift(line, scan_line_half_width, discretization_params, &right);
  assert(left.size() == right.size());

  ScanLines scan_lines;
  scan_lines.reserve(left.size());
  for (const auto ground_scan_line : boost::range::combine(left, right)) {
    const VehicleScanLine l{boost::get<0>(ground_scan_line), boost::get<1>(ground_scan_line)};
    if (!isAboveLowerImageEdge(l) || isOccludedByMask(l)) {
      continue;
    }
    clampAndPush(transformGroundToROI(view_transform, l), &scan_lines);
  }
  return scan_lines;
}

template <class Range>
double computeModalValue(const Range& points, const double bin_size) {
  const auto minmax = std::minmax_element(std::begin(points), std::end(points));
  const double min = *minmax.first;
  const double max = *minmax.second;
  std::vector<int> hist(static_cast<std::size_t>((max - min) / bin_size) + 1, 0);
  for (const double p : points) {
    hist[static_cast<int>((p - min) / bin_size)]++;
  }

  const auto idx = std::distance(hist.begin(), boost::max_element(hist));
  return idx * bin_size + min;
}

ScanLines FeatureExtraction::createScanLines(const VehiclePoints& ground_points) const {

  ScanLines scan_lines;
  if (ground_points.size() < 2) {  // statistics need more than two points
    return scan_lines;
  }

  const auto xs = ground_points | common::x_values;
  const auto ys = ground_points | common::y_values;

  const auto x_minmax = std::minmax_element(std::begin(xs), std::end(xs));
  const double x_min = *x_minmax.first, x_max = *x_minmax.second;
  const double stddev = std::sqrt(common::variance(ys, common::mean(ys)));
  const double mu =
      computeModalValue(ys, parameters_ptr_->getParam(GENERAL_HISTOGRAM_BIN_SIZE));
  const double sigma = parameters_ptr_->getParam(GENERAL_SIGMA_FACTOR) * stddev;

  const double SCAN_LINE_STEP_SIZE = parameters_ptr_->getParam(GENERAL_SCAN_LINE_STEP_SIZE);
  scan_lines.reserve(static_cast<std::size_t>((x_max - x_min) / SCAN_LINE_STEP_SIZE));
  for (double x = x_min; x < x_max; x += SCAN_LINE_STEP_SIZE) {
    clampAndPush(transformGroundToROI(
                     view_transform,
                     WorldScanLine({x, mu + sigma, 0.0}, {x, mu - sigma, 0.0})),
                 &scan_lines);
  }
  return scan_lines;
}

void FeatureExtraction::clampAndPush(ScanLine scan_line, ScanLines* scan_lines) const {
  if (ScanLine::clip(roi.size(), scan_line)) {
    scan_lines->push_back(scan_line);
  }
}

void FeatureExtraction::getCenterOfConnectedLinePoints(
    const cv::Mat& image,
    const ScanLine& scan_line,
    ImagePoints* center_points,
    const CenterOfConnectedPointsParam& params,
    const boost::optional<common::DynamicPolynomial>& ref_line,
    const boost::optional<LineSpec> l_spec) const {

  const auto max_width_feature_pair = params.max_width_feature_pair;
  const auto step = params.step;

  enum InternalParseState { STATE_BLACK = 0, STATE_WHITE = 1 };
  const auto get_point_pairs = [&image](const ScanLine& sl) {
    cv::LineIterator it(image, toCV(sl.start), toCV(sl.end), 8);
    cv::Point2i start_point, last_point;

    std::vector<std::pair<ImagePoint, ImagePoint> > pairs;

    // state machine to detect connected line points on a scan line.
    InternalParseState state = STATE_BLACK;
    for (int i = 0; i < it.count; i++, it++) {
      if (state == STATE_WHITE && **it != WHITE_PIXEL) {
        pairs.emplace_back(toEigen(start_point), toEigen(last_point));
        state = STATE_BLACK;
      }
      if (state == STATE_BLACK && **it == WHITE_PIXEL) {
        start_point = it.pos();
        state = STATE_WHITE;
      }
      last_point = it.pos();
    }
    return pairs;
  };

  const auto get_principal_component = [&](std::size_t bin_size, double x) {
    Eigen::MatrixXd stacked_points(bin_size, 3);

    for (std::size_t i = 0; i < bin_size; ++i) {
      const auto loc_x = x - (bin_size - i) * step;
      const VehiclePoint p(loc_x, ref_line->evaluate(loc_x), 0.0);

      stacked_points.row(i) = p;
    }

    return common::pca_eigen::getPrincipalComponent(stacked_points).normalized();
  };


  const auto pairs = get_point_pairs(scan_line);
  const auto pairs_swapped = get_point_pairs(scan_line.swapped());

  const auto sl_start = view_transform->transformROIToGround(scan_line.start);
  const auto sl_end = view_transform->transformROIToGround(scan_line.end);
  const auto sl_mid = (sl_start + sl_end) / 2.0;

  const auto sl_normal = sl_end.y() < sl_start.y()
                             ? (sl_end - sl_start).unitOrthogonal()
                             : (sl_start - sl_end).unitOrthogonal();
  const ScanLine future_sl(
      view_transform->transformGroundToROI(sl_start + sl_normal * params.step),
      view_transform->transformGroundToROI(sl_end + sl_normal * params.step));

  const auto future_pairs = get_point_pairs(future_sl);

  if (!pairs.empty() && !pairs_swapped.empty()) {

    const auto center_point_first = (pairs.front().first + pairs.front().second) / 2;
    const auto center_point_last =
        (pairs_swapped.front().first + pairs_swapped.front().second) / 2;

    if (center_point_first == center_point_last) {

      // if the future scanline has more points and the current one is unique,
      // it is a very bad to use this point
      if (future_pairs.size() > 1)
        return;

      const VehiclePoint pair_pt1 =
          view_transform->transformROIToGround(pairs.front().first);
      const VehiclePoint pair_pt2 =
          view_transform->transformROIToGround(pairs.front().second);
      const auto pair_dist = (pair_pt1 - pair_pt2).norm();

      if (ref_line.is_initialized()) {
        const auto x =
            utils::findLotfusspunktX(*ref_line, to2D(pair_pt1 + pair_pt2) / 2.0);
        const auto pc = get_principal_component(10, x);
        const VehiclePoint hyp = (pair_pt1 - pair_pt2).normalized();
        const auto dot_abs = std::fabs(hyp.dot(to3D(pc)));

        auto previous_cond = [&]() {
          if (!center_points->empty()) {
            const auto& previous_point = center_points->back();
            const auto hyp_previous =
                (view_transform->transformROIToGround(center_point_first) -
                 view_transform->transformROIToGround(previous_point)).normalized();
            const auto normal_pc = to3D(pc.unitOrthogonal());
            const auto dot_previous_abs = std::fabs(hyp_previous.dot(normal_pc));

            // return dot_previous_abs < 0.5;
          }

          return true;
        }();

        if (pair_dist < max_width_feature_pair && dot_abs < params.max_abs_dot && previous_cond) {
          // ROS_DEBUG_STREAM("unique, with reference: " << center_point_first);
          center_points->push_back(center_point_first);
        }
      } else {
        if (pair_dist < max_width_feature_pair) {
          // ROS_DEBUG_STREAM("unique, no reference");
          center_points->push_back(center_point_first);
        }
      }
    } else if (l_spec.is_initialized()) {
      // if (*l_spec != LINESPEC_MIDDLE) return;
      if (ref_line.is_initialized()) {

        boost::optional<ImagePoint> selected_point;
        boost::optional<ImagePoint> fallback_point;


        if (center_points->empty()) {
          auto max_dist = (sl_start - sl_end).norm();
          auto max_dot = params.max_abs_dot;

          if (pairs.size() == future_pairs.size()) {

            // ROS_DEBUG_STREAM("empty, mult candidates, with reference " <<
            // pairs.size());

            std::vector<VehiclePoint> candidates;

            for (const auto& dual_pair : boost::range::combine(pairs, future_pairs)) {
              const auto& pair = boost::get<0>(dual_pair);
              const auto center = (pair.first + pair.second) / 2;
              const auto& future_pair = boost::get<1>(dual_pair);
              const auto future_center = (future_pair.first + future_pair.second) / 2;

              const VehiclePoint center_on_ground =
                  view_transform->transformROIToGround(center);
              const VehiclePoint future_center_on_ground =
                  view_transform->transformROIToGround(future_center);

              const VehiclePoint hyp =
                  (future_center_on_ground - center_on_ground).normalized();

              candidates.push_back(hyp);
            }

            bool candidates_parallel = true;
            for (std::size_t i = 0; i < candidates.size() - 1; ++i) {
              for (std::size_t j = i + 1; j < candidates.size(); ++j) {
                if (candidates[i].dot(candidates[j]) < 0.9) {
                  candidates_parallel = false;
                  break;
                }
              }
            }

            for (const auto& dual_pair : boost::range::combine(pairs, future_pairs)) {
              const auto& pair = boost::get<0>(dual_pair);
              const auto center = (pair.first + pair.second) / 2;
              const VehiclePoint center_on_ground =
                  view_transform->transformROIToGround(center);
              const VehiclePoint pair_pt1 =
                  view_transform->transformROIToGround(pair.first);
              const VehiclePoint pair_pt2 =
                  view_transform->transformROIToGround(pair.second);

              const auto& future_pair = boost::get<1>(dual_pair);
              const auto future_center = (future_pair.first + future_pair.second) / 2;
              const VehiclePoint future_center_on_ground =
                  view_transform->transformROIToGround(future_center);

              const auto dist = (center_on_ground - sl_mid).norm();

              const auto x =
                  utils::findLotfusspunktX(*ref_line, to2D(pair_pt1 + pair_pt2) / 2.0);
              // const auto x = (pair_pt1.x() + pair_pt2.x())/2.0;

              const auto pc = get_principal_component(10, x);

              const auto normal_pc = to3D(pc.unitOrthogonal());

              const VehiclePoint hyp =
                  (future_center_on_ground - center_on_ground).normalized();
              const auto dot_abs = std::fabs(hyp.dot(to3D(normal_pc)));

              const bool same_dir = (hyp.y() * pc.y()) > 0;

              if (!candidates_parallel) {
                if ((dot_abs < max_dot - params.eps) && same_dir) {
                  max_dot = dot_abs;
                  selected_point = center;
                }
              } else {
                if ((dist < max_dist)) {
                  max_dist = dist;
                  selected_point = center;
                }
              }
            }
          }
        } else {

          const auto& previous_point = center_points->back();

          const auto x = utils::findLotfusspunktX(*ref_line, to2D(sl_mid));

          const auto pc = get_principal_component(10, x);

          const auto normal_pc = to3D(pc.unitOrthogonal());

          auto max_dot = params.max_abs_dot;
          for (const auto& pair : pairs) {
            const auto center = (pair.first + pair.second) / 2;

            const VehiclePoint pair_pt1 = view_transform->transformROIToGround(pair.first);
            const VehiclePoint pair_pt2 =
                view_transform->transformROIToGround(pair.second);
            const auto pair_dist = (pair_pt1 - pair_pt2).norm();

            const VehiclePoint hyp =
                (view_transform->transformROIToGround(center) -
                 view_transform->transformROIToGround(previous_point)).normalized();

            const auto dot_abs = std::fabs(hyp.dot(normal_pc));
            const bool same_dir = (hyp.y() * pc.y()) > 0;

            if ((dot_abs < max_dot - params.eps) && (pair_dist < max_width_feature_pair)) {
              max_dot = dot_abs;
              if (same_dir) {
                selected_point = center;
              } else {
                fallback_point = center;
              }
              // ROS_DEBUG_STREAM("initialized, mult candidates, with
              // reference");
            }
          }
        }

        if (selected_point.is_initialized()) {
          center_points->push_back(*selected_point);
        } else if (fallback_point.is_initialized()) {
          center_points->push_back(*fallback_point);
        }
      }
    }
  }
}


LineVehiclePoints FeatureExtraction::getSegmentedGroundPoints(const cv::Mat& image) const {
  std::vector<cv::Point> pixels;
  cv::findNonZero(image, pixels);

  const double BOUNDARIES_OUTER_LEFT =
      parameters_ptr_->getParam(SEGMENT_BOUNDARIES_OUTER_LEFT);
  const double BOUNDARIES_LEFT = parameters_ptr_->getParam(SEGMENT_BOUNDARIES_LEFT);
  const double BOUNDARIES_MIDDLE = parameters_ptr_->getParam(SEGMENT_BOUNDARIES_MIDDLE);
  const double BOUNDARIES_RIGHT = parameters_ptr_->getParam(SEGMENT_BOUNDARIES_RIGHT);
  const double BOUNDARIES_NEAR = parameters_ptr_->getParam(SEGMENT_BOUNDARIES_NEAR);
  const double BOUNDARIES_FAR = parameters_ptr_->getParam(SEGMENT_BOUNDARIES_FAR);

  LineVehiclePoints ground_points;
  for (const cv::Point& pixel : pixels) {
    const VehiclePoint pt = view_transform->transformROIToGround(toEigen(pixel));

    if (pt.x() < BOUNDARIES_NEAR || pt.x() > BOUNDARIES_FAR || pt.y() > BOUNDARIES_OUTER_LEFT) {
      continue;
    }

    if (pt.y() > BOUNDARIES_LEFT) {
      ground_points[LINESPEC_LEFT].push_back(pt);
    } else if (pt.y() > BOUNDARIES_MIDDLE) {
      ground_points[LINESPEC_MIDDLE].push_back(pt);
    } else if (pt.y() > BOUNDARIES_RIGHT) {
      ground_points[LINESPEC_RIGHT].push_back(pt);
    }
  }
  return ground_points;
}

void FeatureExtraction::updateLowerImageEdgeX() {
  lower_image_edge_x = std::max(
      view_transform->transformROIToGround(ImagePoint(0, roi.height)).x(),
      view_transform->transformROIToGround(ImagePoint(roi.width, roi.height)).x());
}

bool FeatureExtraction::isDoubleLine(const cv::Mat& image,
                                     const ScanLines& lines,
                                     VehiclePoints* left_line,
                                     VehiclePoints* right_line) {
  bool double_line_present = false;
  for (const ScanLine& line : lines) {
    cv::LineIterator it(image, toCV(line.start), toCV(line.end));
    double_line_present |= doubleDetected(it, left_line, right_line);
  }
  return double_line_present;
}

bool FeatureExtraction::doubleDetected(cv::LineIterator& it,
                                       VehiclePoints* left_line,
                                       VehiclePoints* right_line) {

  const uchar white_state = WHITE_PIXEL;
  const uchar black_state = 0;
  uchar exp_state = white_state;
  std::vector<cv::Point> detected_points;
  for (int i = 0; i < it.count; i++, ++it) {
    if (**it == exp_state) {
      exp_state = (exp_state == white_state) ? black_state : white_state;
      detected_points.push_back(it.pos());
    }
  }

  const std::size_t number_of_lines = detected_points.size() / 2;
  if (number_of_lines == 0) {
    return false;
  }
  if (number_of_lines == 1) {
    right_line->push_back(view_transform->transformROIToGround(
        toEigen(detected_points.front() + detected_points.back()) / 2));
    return false;
  }
  if (number_of_lines == 2) {
    right_line->push_back(view_transform->transformROIToGround(
        toEigen((detected_points[0] + detected_points[1]) / 2)));
    left_line->push_back(view_transform->transformROIToGround(
        toEigen((detected_points[2] + detected_points[3]) / 2)));
    return true;
  }
  ROS_DEBUG(COLOR_YELLOW
            "More than two middle lines detected, dropping according "
            "points!" COLOR_DEBUG);
  ROS_DEBUG("mid lines detected: %zu", number_of_lines);
  return false;
}

bool FeatureExtraction::isAboveLowerImageEdge(const VehicleScanLine& l) const {
  return l.start.x() > lower_image_edge_x && l.end.x() > lower_image_edge_x;
}

bool FeatureExtraction::isOccludedByMask(const VehicleScanLine& l) const {
  return ego_vehicle_->contains(toCV(view_transform->transformGroundToROI(l.start)));
}
