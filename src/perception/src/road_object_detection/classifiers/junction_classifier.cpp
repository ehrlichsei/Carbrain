#include "junction_classifier.h"

#include "common/basic_statistics.h"

#include <limits>
#include "../../utils/step_detection.h"
#include "common/math.h"

THIRD_PARTY_HEADERS_BEGIN
#include <opencv_eigen_conversions.h>
#include <boost/algorithm/clamp.hpp>
#include <boost/algorithm/cxx11/copy_if.hpp>
#include <boost/range/algorithm/transform.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv/cv.hpp>

THIRD_PARTY_HEADERS_END

#include "common/eigen_utils.h"
#include "common/join.h"
#include "common/make_vector.h"
#include "common/eigen_functors.h"
#include "common/eigen_adaptors.h"
#include "common/minmax_element.h"
#include "common/pca_eigen.h"
#include "common/make_vector.h"
#include "common/polynomial_utils.h"
#include "common/angle_conversions.h"
#include "common/unique_erase.h"

#include <iostream>
#include <fstream>

namespace road_object_detection {


void JunctionClassifier::registrate(ParameterInterface* parameter_interface,
                                    const TrapezeParameterStrings& p) {
  parameter_interface->registerParam(p.MIN);
  parameter_interface->registerParam(p.MAX);
  parameter_interface->registerParam(p.FALL_DIST);
}

JunctionClassifier::Trapeze JunctionClassifier::getParams(
    const ParameterInterface* const parameter_interface,
    const JunctionClassifier::TrapezeParameterStrings& p) {
  return {parameter_interface->getParam(p.MIN),
          parameter_interface->getParam(p.MAX),
          parameter_interface->getParam(p.FALL_DIST)};
}

const std::string JunctionClassifier::NAMESPACE("junction_classifier");

const ParameterString<int> JunctionClassifier::STEP_THRESHOLD(
    NAMESPACE + "/step_threshold");
const ParameterString<int> JunctionClassifier::STEP_DIST(NAMESPACE +
                                                         "/step_dist");
const ParameterString<int> JunctionClassifier::STEP_FUNCTION_LENGHT(
    NAMESPACE + "/step_function_lenght");
const ParameterString<int> JunctionClassifier::STEP_FUNCTION_OFFSET(
    NAMESPACE + "/step_function_offset");
const ParameterString<int> JunctionClassifier::MIN_POINT_DIST(
    NAMESPACE + "/min_point_dist");
const ParameterString<int> JunctionClassifier::RECT_ADD_SPACE_X(
    NAMESPACE + "/rect_add_space_x");
const ParameterString<int> JunctionClassifier::RECT_ADD_SPACE_Y(
    NAMESPACE + "/rect_add_space_y");
const ParameterString<int> JunctionClassifier::RECT_MIN_X_SIZE(
    NAMESPACE + "/rect_min_x_size");
const ParameterString<int> JunctionClassifier::RECT_MIN_Y_SIZE(
    NAMESPACE + "/rect_min_y_size");

const ParameterString<int> JunctionClassifier::ADAPTIVE_THRESH_BLOCKSIZE(
    NAMESPACE + "/adaptive_thresh_blocksize");
const ParameterString<int> JunctionClassifier::ADAPTIVE_THRESH_CONST(
    NAMESPACE + "/adaptive_thresh_const");

const ParameterString<double> JunctionClassifier::DEFAULT_IS_GIVE_WAY_LINE_LEFT(
    NAMESPACE + "/default_is_give_way_line_left");
const ParameterString<double> JunctionClassifier::DEFAULT_IS_GIVE_WAY_LINE_RIGHT(
    NAMESPACE + "/default_is_give_way_line_right");
const ParameterString<double> JunctionClassifier::DEFAULT_IS_STOP_LINE_LEFT(
    NAMESPACE + "/default_is_stop_line_left");
const ParameterString<double> JunctionClassifier::DEFAULT_IS_STOP_LINE_RIGHT(
    NAMESPACE + "/default_is_stop_line_right");

const JunctionClassifier::TrapezeParameterStrings JunctionClassifier::ANGLE_DIFF =
    TrapezeParameterStrings::Create(NAMESPACE + "/angle_diff");

const JunctionClassifier::TrapezeParameterStrings JunctionClassifier::GIVE_WAY_LINE_LEFT_MEAN1 =
    TrapezeParameterStrings::Create(NAMESPACE + "/give_way_line_left_mean1");

const JunctionClassifier::TrapezeParameterStrings JunctionClassifier::GIVE_WAY_LINE_LEFT_MEAN0 =
    TrapezeParameterStrings::Create(NAMESPACE + "/give_way_line_left_mean0");

const JunctionClassifier::TrapezeParameterStrings JunctionClassifier::GIVE_WAY_LINE_LEFT_DIST_MIDDLELANE_PCA =
    TrapezeParameterStrings::Create(NAMESPACE +
                                    "/give_way_line_left_dist_middlelane_pca");

const JunctionClassifier::TrapezeParameterStrings JunctionClassifier::GIVE_WAY_LINE_OUTER_LINE_STEPS =
    TrapezeParameterStrings::Create(NAMESPACE +
                                    "/give_way_line_outer_line_steps");

const JunctionClassifier::TrapezeParameterStrings JunctionClassifier::GIVE_WAY_LINE_MIDDLE_LINE_STEPS =
    TrapezeParameterStrings::Create(NAMESPACE +
                                    "/give_way_line_middle_line_steps");

const JunctionClassifier::TrapezeParameterStrings JunctionClassifier::GIVE_WAY_LINE_POS_NEG_RATIO =
    TrapezeParameterStrings::Create(NAMESPACE + "/give_way_line_pos_neg_ratio");

const JunctionClassifier::TrapezeParameterStrings JunctionClassifier::GIVE_WAY_LINE_RIGHT_MEAN1 =
    TrapezeParameterStrings::Create(NAMESPACE + "/give_way_line_right_mean1");

const JunctionClassifier::TrapezeParameterStrings JunctionClassifier::GIVE_WAY_LINE_RIGHT_MEAN0 =
    TrapezeParameterStrings::Create(NAMESPACE + "/give_way_line_right_mean0");

const JunctionClassifier::TrapezeParameterStrings JunctionClassifier::GIVE_WAY_LINE_RIGHT_DIST_MIDDLELANE_PCA =
    TrapezeParameterStrings::Create(NAMESPACE +
                                    "/give_way_line_right_dist_middlelane_pca");

const JunctionClassifier::TrapezeParameterStrings JunctionClassifier::STOP_LINE_LEFT_MEAN1 =
    TrapezeParameterStrings::Create(NAMESPACE + "/stop_line_left_mean1");

const JunctionClassifier::TrapezeParameterStrings JunctionClassifier::STOP_LINE_LEFT_MEAN0 =
    TrapezeParameterStrings::Create(NAMESPACE + "/stop_line_left_mean0");

const JunctionClassifier::TrapezeParameterStrings JunctionClassifier::STOP_LINE_LEFT_DIST_MIDDLELANE_PCA =
    TrapezeParameterStrings::Create(NAMESPACE +
                                    "/stop_line_left_dist_middlelane_pca");

const JunctionClassifier::TrapezeParameterStrings JunctionClassifier::STOP_LINE_OUTER_LINE_STEPS =
    TrapezeParameterStrings::Create(NAMESPACE + "/stop_line_outer_line_steps");

const JunctionClassifier::TrapezeParameterStrings JunctionClassifier::STOP_LINE_MIDDLE_LINE_STEPS =
    TrapezeParameterStrings::Create(NAMESPACE + "/stop_line_middle_line_steps");

const JunctionClassifier::TrapezeParameterStrings JunctionClassifier::STOP_LINE_POS_NEG_RATIO =
    TrapezeParameterStrings::Create(NAMESPACE + "/stop_line_pos_neg_ratio");

const JunctionClassifier::TrapezeParameterStrings JunctionClassifier::STOP_LINE_RIGHT_MEAN1 =
    TrapezeParameterStrings::Create(NAMESPACE + "/stop_line_right_mean1");

const JunctionClassifier::TrapezeParameterStrings JunctionClassifier::STOP_LINE_RIGHT_MEAN0 =
    TrapezeParameterStrings::Create(NAMESPACE + "/stop_line_right_mean0");

const JunctionClassifier::TrapezeParameterStrings JunctionClassifier::STOP_LINE_RIGHT_DIST_MIDDLELANE_PCA =
    TrapezeParameterStrings::Create(NAMESPACE +
                                    "/stop_line_right_dist_middlelane_pca");

const JunctionClassifier::TrapezeParameterStrings JunctionClassifier::GIVE_WAY_MIDDLE_LINE_NO_MEDIAN =
    TrapezeParameterStrings::Create(NAMESPACE +
                                    "/give_way_middle_line_no_median");

const JunctionClassifier::TrapezeParameterStrings JunctionClassifier::STOP_MIDDLE_LINE_NO_MEDIAN =
    TrapezeParameterStrings::Create(NAMESPACE + "/stop_middle_line_no_median");

const JunctionClassifier::TrapezeParameterStrings JunctionClassifier::MIRROR_SIDE_NO_MEDIAN =
    TrapezeParameterStrings::Create(NAMESPACE + "/mirror_side_no_median");

const ParameterString<int> JunctionClassifier::OFFSET_EIGENLINE(
    NAMESPACE + "/offset_eigenline");
const ParameterString<int> JunctionClassifier::LENGHT_EIGENLINE(
    NAMESPACE + "/lenght_eigenline");
const ParameterString<int> JunctionClassifier::LENGHT_EIGENLINE_OFFSET(
    NAMESPACE + "/lenght_eigenline_offset");
const ParameterString<int> JunctionClassifier::LENGHT_STEP_REF_FUNC(
    NAMESPACE + "/lenght_step_ref_func");
const ParameterString<int> JunctionClassifier::THRESHOLD_STEP_EIGENLINE(
    NAMESPACE + "/threshold_step_eigenline");

const ParameterString<int> JunctionClassifier::VERTICAL_OFFSET(
    NAMESPACE + "/vertical_offset");

const ParameterString<int> JunctionClassifier::FEATURE_WINDOW_HEIGHT(
    NAMESPACE + "/feature_window_height");

const ParameterString<int> JunctionClassifier::MIN_X_DIST(NAMESPACE +
                                                          "/min_x_dist");

const ParameterString<int> JunctionClassifier::MIN_POINT_NR(NAMESPACE +
                                                            "/min_point_nr");

const ParameterString<int> JunctionClassifier::FILTER_SIZE_MEDIAN(
    NAMESPACE + "/filter_size_median");

JunctionClassifier::JunctionClassifier(ParameterInterface* parameter_interface,
                                       const common::CameraTransformation* const camera_transform)
    : camera_transformation_(camera_transform),
      parameter_interface_(parameter_interface) {
  parameter_interface->registerParam(STEP_THRESHOLD);
  parameter_interface->registerParam(STEP_DIST);
  parameter_interface->registerParam(STEP_FUNCTION_LENGHT);
  parameter_interface->registerParam(STEP_FUNCTION_OFFSET);
  parameter_interface->registerParam(MIN_POINT_DIST);
  parameter_interface->registerParam(RECT_ADD_SPACE_X);
  parameter_interface->registerParam(RECT_ADD_SPACE_Y);
  parameter_interface->registerParam(RECT_MIN_X_SIZE);
  parameter_interface->registerParam(RECT_MIN_Y_SIZE);
  parameter_interface->registerParam(DEFAULT_IS_GIVE_WAY_LINE_LEFT);
  parameter_interface->registerParam(DEFAULT_IS_STOP_LINE_LEFT);
  parameter_interface->registerParam(DEFAULT_IS_GIVE_WAY_LINE_RIGHT);
  parameter_interface->registerParam(DEFAULT_IS_STOP_LINE_RIGHT);
  parameter_interface->registerParam(ADAPTIVE_THRESH_BLOCKSIZE);
  parameter_interface->registerParam(ADAPTIVE_THRESH_CONST);

  registrate(parameter_interface, ANGLE_DIFF);

  registrate(parameter_interface, GIVE_WAY_LINE_LEFT_MEAN1);
  registrate(parameter_interface, GIVE_WAY_LINE_LEFT_MEAN0);

  registrate(parameter_interface, GIVE_WAY_LINE_LEFT_DIST_MIDDLELANE_PCA);

  registrate(parameter_interface, GIVE_WAY_LINE_OUTER_LINE_STEPS);

  registrate(parameter_interface, GIVE_WAY_LINE_MIDDLE_LINE_STEPS);

  registrate(parameter_interface, GIVE_WAY_LINE_POS_NEG_RATIO);

  registrate(parameter_interface, GIVE_WAY_LINE_RIGHT_MEAN1);
  registrate(parameter_interface, GIVE_WAY_LINE_RIGHT_MEAN0);

  registrate(parameter_interface, GIVE_WAY_LINE_RIGHT_DIST_MIDDLELANE_PCA);

  registrate(parameter_interface, GIVE_WAY_MIDDLE_LINE_NO_MEDIAN);

  registrate(parameter_interface, STOP_LINE_LEFT_MEAN1);
  registrate(parameter_interface, STOP_LINE_LEFT_MEAN0);

  registrate(parameter_interface, STOP_LINE_LEFT_DIST_MIDDLELANE_PCA);

  registrate(parameter_interface, STOP_LINE_OUTER_LINE_STEPS);
  registrate(parameter_interface, STOP_LINE_MIDDLE_LINE_STEPS);

  registrate(parameter_interface, STOP_LINE_POS_NEG_RATIO);

  registrate(parameter_interface, STOP_LINE_RIGHT_MEAN1);
  registrate(parameter_interface, STOP_LINE_RIGHT_MEAN0);

  registrate(parameter_interface, STOP_LINE_RIGHT_DIST_MIDDLELANE_PCA);

  registrate(parameter_interface, STOP_MIDDLE_LINE_NO_MEDIAN);

  parameter_interface->registerParam(OFFSET_EIGENLINE);
  parameter_interface->registerParam(LENGHT_EIGENLINE);
  parameter_interface->registerParam(LENGHT_EIGENLINE_OFFSET);
  parameter_interface->registerParam(LENGHT_STEP_REF_FUNC);
  parameter_interface->registerParam(THRESHOLD_STEP_EIGENLINE);
  parameter_interface->registerParam(VERTICAL_OFFSET);

  registrate(parameter_interface, MIRROR_SIDE_NO_MEDIAN);

  parameter_interface->registerParam(FEATURE_WINDOW_HEIGHT);
  parameter_interface->registerParam(MIN_POINT_NR);
  parameter_interface->registerParam(MIN_X_DIST);

  parameter_interface->registerParam(FILTER_SIZE_MEDIAN);
}

JunctionClassifier::Params JunctionClassifier::readParameters() const {
  Params params{};

  params.min_point_dist = parameter_interface_->getParam(MIN_POINT_DIST);
  params.min_point_nr = parameter_interface_->getParam(MIN_POINT_NR);
  params.vertical_offset = parameter_interface_->getParam(VERTICAL_OFFSET);
  params.offset_eigenline = parameter_interface_->getParam(OFFSET_EIGENLINE);
  params.lenght_eigenline = parameter_interface_->getParam(LENGHT_EIGENLINE);
  params.lenght_eigenline_offset = parameter_interface_->getParam(LENGHT_EIGENLINE_OFFSET);
  params.lenght_step_ref_func = parameter_interface_->getParam(LENGHT_STEP_REF_FUNC);
  params.threshold_step_eigenline = parameter_interface_->getParam(THRESHOLD_STEP_EIGENLINE);
  params.filter_size_median = parameter_interface_->getParam(FILTER_SIZE_MEDIAN);
  params.adaptive_thresh_blocksize =
      parameter_interface_->getParam(ADAPTIVE_THRESH_BLOCKSIZE);
  params.adaptive_thresh_const = parameter_interface_->getParam(ADAPTIVE_THRESH_CONST);
  params.is_stop_line_left = parameter_interface_->getParam(DEFAULT_IS_STOP_LINE_LEFT);
  params.is_give_way_line_left =
      parameter_interface_->getParam(DEFAULT_IS_GIVE_WAY_LINE_LEFT);
  params.is_stop_line_right = parameter_interface_->getParam(DEFAULT_IS_STOP_LINE_RIGHT);
  params.is_give_way_line_right =
      parameter_interface_->getParam(DEFAULT_IS_GIVE_WAY_LINE_RIGHT);

  params.angle_diff = getParams(parameter_interface_, ANGLE_DIFF);

  params.stop_line_left_mean1 = getParams(parameter_interface_, STOP_LINE_LEFT_MEAN1);

  params.stop_line_left_mean0 = getParams(parameter_interface_, STOP_LINE_LEFT_MEAN0);

  params.stop_line_left_middlelane_pca =
      getParams(parameter_interface_, STOP_LINE_LEFT_DIST_MIDDLELANE_PCA);

  params.stop_line_outer_line_steps =
      getParams(parameter_interface_, STOP_LINE_OUTER_LINE_STEPS);

  params.stop_line_middle_line_steps =
      getParams(parameter_interface_, STOP_LINE_MIDDLE_LINE_STEPS);

  params.stop_line_pos_neg_ratio = getParams(parameter_interface_, STOP_LINE_POS_NEG_RATIO);

  params.stop_line_right_mean1 = getParams(parameter_interface_, STOP_LINE_RIGHT_MEAN1);
  params.stop_line_right_mean0 = getParams(parameter_interface_, STOP_LINE_RIGHT_MEAN0);

  params.stop_line_right_middlelane_pca =
      getParams(parameter_interface_, STOP_LINE_RIGHT_DIST_MIDDLELANE_PCA);

  params.give_way_line_left_mean1 =
      getParams(parameter_interface_, GIVE_WAY_LINE_LEFT_MEAN1);
  params.give_way_line_left_mean0 =
      getParams(parameter_interface_, GIVE_WAY_LINE_LEFT_MEAN0);

  params.give_way_line_left_middlelane_pca =
      getParams(parameter_interface_, GIVE_WAY_LINE_LEFT_DIST_MIDDLELANE_PCA);

  params.give_way_line_outer_line_steps =
      getParams(parameter_interface_, GIVE_WAY_LINE_OUTER_LINE_STEPS);

  params.give_way_line_middle_line_steps =
      getParams(parameter_interface_, GIVE_WAY_LINE_MIDDLE_LINE_STEPS);

  params.give_way_line_pos_neg_ratio =
      getParams(parameter_interface_, GIVE_WAY_LINE_POS_NEG_RATIO);

  params.give_way_line_right_mean1 =
      getParams(parameter_interface_, GIVE_WAY_LINE_RIGHT_MEAN1);
  params.give_way_line_right_mean0 =
      getParams(parameter_interface_, GIVE_WAY_LINE_RIGHT_MEAN0);

  params.give_way_line_right_middlelane_pca =
      getParams(parameter_interface_, GIVE_WAY_LINE_RIGHT_DIST_MIDDLELANE_PCA);

  params.give_way_middle_line_no_median =
      getParams(parameter_interface_, GIVE_WAY_MIDDLE_LINE_NO_MEDIAN);

  params.stop_middle_line_no_median =
      getParams(parameter_interface_, STOP_MIDDLE_LINE_NO_MEDIAN);

  params.mirror_side_no_median = getParams(parameter_interface_, MIRROR_SIDE_NO_MEDIAN);

  params.min_x_dist = parameter_interface_->getParam(MIN_X_DIST);

  return params;
}


cv::Rect JunctionClassifier::computeBordersRect(const Features& features) const {

  const int rect_add_space_x = parameter_interface_->getParam(RECT_ADD_SPACE_X);
  const int rect_add_space_y = parameter_interface_->getParam(RECT_ADD_SPACE_Y);
  const int rect_min_x_size = parameter_interface_->getParam(RECT_MIN_X_SIZE);
  const int rect_min_y_size = parameter_interface_->getParam(RECT_MIN_Y_SIZE);
  const cv::Point rect_offset_point = cv::Point(rect_add_space_x, rect_add_space_y);
  const cv::Size rect_offset_size = cv::Size(2 * rect_add_space_x, 2 * rect_add_space_y);

  ImagePoints image_feature_points;
  image_feature_points.reserve(features.cluster.feature_points_vehicle.size());

  boost::transform(features.cluster.feature_points_vehicle,
                   std::back_inserter(image_feature_points),
                   [features](const VehiclePoint& i) {
                     return features.birdsview_patch.vehicleToImage(i);
                   });

  cv::Rect borders_rect = cv::boundingRect(toCV(image_feature_points));
  borders_rect -= rect_offset_point;
  borders_rect += rect_offset_size;

  if (borders_rect.width <= rect_min_x_size) {
    int borders_rect_x_diff = rect_min_x_size - borders_rect.width;
    borders_rect -= cv::Point(borders_rect_x_diff / 2, 0);
    borders_rect += cv::Size(borders_rect_x_diff, 0);
  }

  if (borders_rect.height <= rect_min_y_size) {
    int borders_rect_y_diff = rect_min_y_size - borders_rect.height;
    borders_rect -= cv::Point(0, borders_rect_y_diff / 2);
    borders_rect += cv::Size(0, borders_rect_y_diff);
  }

  return borders_rect;
}

RoadObjects JunctionClassifier::classify(const Features& features) {
  const cv::Mat* birdsview = features.birdsview_patch.getImage();

  const int threshold = parameter_interface_->getParam(STEP_THRESHOLD);
  const int step_dist = parameter_interface_->getParam(STEP_DIST);
  const int step_function_lenght = parameter_interface_->getParam(STEP_FUNCTION_LENGHT);
  const int step_function_offset = parameter_interface_->getParam(STEP_FUNCTION_OFFSET);
  const int min_point_dist = parameter_interface_->getParam(MIN_POINT_DIST);

  const ImagePoints pos_steps = getStepPointsNoDoubling(
      step_dist, step_function_lenght, step_function_offset, min_point_dist, *birdsview, threshold, true);
  const ImagePoints neg_steps = getStepPointsNoDoubling(
      step_dist, step_function_lenght, step_function_offset, min_point_dist, *birdsview, threshold, false);


  const cv::Rect borders_rect = computeBordersRect(features);
  const auto is_inlayer = [&borders_rect](const ImagePoint& x) {
    return borders_rect.contains(toCV(x));
  };

  ImagePoints pos_steps_box;
  ImagePoints neg_steps_box;
  boost::algorithm::copy_if(pos_steps, std::back_inserter(pos_steps_box), is_inlayer);
  boost::algorithm::copy_if(neg_steps, std::back_inserter(neg_steps_box), is_inlayer);

  const auto winning_candidates = findFourCandidates(features, pos_steps_box, neg_steps_box);

  return createRoadObjects(winning_candidates, features);
}

JunctionClassifier::FourCandidates JunctionClassifier::findFourCandidates(
    const Features& features, const ImagePoints& pos_steps_box, const ImagePoints& neg_steps_box) const {
  const auto params = readParameters();
  ImagePoints steps_box =
      common::make_eigen_vector(common::join(pos_steps_box, neg_steps_box));

  cv::Rect feature_window;
  feature_window.x = 0;
  feature_window.width = features.birdsview_patch.getImage()->cols;
  feature_window.height = parameter_interface_->getParam(FEATURE_WINDOW_HEIGHT);
  std::sort(steps_box.begin(), steps_box.end(), common::less_y());
  ROS_DEBUG_STREAM("steps_box.size(): " << steps_box.size());

  auto before_deleate = steps_box;
  const auto equal_y =
      [params](const auto& a, const auto& b) { return std::abs( a.y() - b.y() ) < params.min_point_dist && std::abs( a.x() - b.x() ) < params.min_point_dist  ; };
  common::unique_erase(steps_box, equal_y);
  ROS_DEBUG_STREAM("steps_box.size() uniqued: " << steps_box.size());

  FourCandidates winning_candidates;

  std::string unique_id = std::to_string(features.timestamp.toNSec());
  int loop_counter = 0;
  float last_point_y = 0;

  for (const auto& step_box : steps_box) {
    if (std::abs(last_point_y - step_box.y()) >= ( params.min_point_dist / 2)){

      last_point_y = step_box.y();
      feature_window.y = step_box.y();

      const CandidateData candidate_data = JunctionClassifier::classifyCandidate(
          features, feature_window, pos_steps_box, neg_steps_box, params, 
          loop_counter, unique_id);

      if (candidate_data.score[Junction::stopline_left] >
          winning_candidates[Junction::stopline_left].score[Junction::stopline_left]) {
        winning_candidates[Junction::stopline_left] = candidate_data;
      }
      if (candidate_data.score[Junction::stopline_right] >
          winning_candidates[Junction::stopline_right].score[Junction::stopline_right]) {
        winning_candidates[Junction::stopline_right] = candidate_data;
      }
      if (candidate_data.score[Junction::givewayline_left] >
          winning_candidates[Junction::givewayline_left].score[Junction::givewayline_left]) {
        winning_candidates[Junction::givewayline_left] = candidate_data;
      }
      if (candidate_data.score[Junction::givewayline_right] >
          winning_candidates[Junction::givewayline_right].score[Junction::givewayline_right]) {
        winning_candidates[Junction::givewayline_right] = candidate_data;
      }

    loop_counter++;
    }

  }
  return winning_candidates;
}

RoadObjects JunctionClassifier::createRoadObjects(const FourCandidates& winning_candidates,
                                                  const Features& features) const {
  RoadObjects road_objects;
  road_objects.reserve(allJunctionTypes().size());
  for (const auto junction_type : allJunctionTypes()) {
    const auto& candidate = winning_candidates[junction_type];
    const auto pose = getLinePose(features, candidate.pca_data);
    road_objects.push_back(std::make_unique<Junction>(
        junction_type,
        features.timestamp,
        candidate.score[junction_type],
        pose,
        computeBaseHullPolygon(features, pose, junction_type)));
  }
  return road_objects;
}

JunctionClassifier::CandidateData JunctionClassifier::classifyCandidate(
    const Features& features,
    const cv::Rect& feature_window,
    const ImagePoints& pos_steps_box,
    const ImagePoints& neg_steps_box,
    const JunctionClassifier::Params& params,
    const int& loop_counter,
    const std::string& unique_id) const {

  JunctionClassifier::CandidateData candidate_data;
  candidate_data.window = feature_window;
  const cv::Mat* birdsview = features.birdsview_patch.getImage();

  const auto offset_eigenline = params.offset_eigenline;
  const auto lenght_eigenline = params.lenght_eigenline;

  const auto is_inlayer = [feature_window](const ImagePoint& x) {
    return feature_window.contains(toCV(x));
  };

  boost::algorithm::copy_if(
      pos_steps_box, std::back_inserter(candidate_data.pos_steps_window), is_inlayer);
  boost::algorithm::copy_if(
      neg_steps_box, std::back_inserter(candidate_data.neg_steps_window), is_inlayer);
  candidate_data.steps_window = common::make_eigen_vector(common::join(
      candidate_data.pos_steps_window, candidate_data.neg_steps_window));

  const auto& neg_steps_window = candidate_data.neg_steps_window;
  const auto& pos_steps_window = candidate_data.pos_steps_window;
  const auto& steps_window = candidate_data.steps_window;

  if (steps_window.size() <= static_cast<double>(params.min_point_nr)) {
    return candidate_data;
  }

  const int WHITE_PIXEL = 255;

  cv::Mat birdsview_binary;
  cv::adaptiveThreshold(*birdsview,
                        birdsview_binary,
                        WHITE_PIXEL,
                        cv::ADAPTIVE_THRESH_GAUSSIAN_C,
                        cv::THRESH_BINARY,
                        params.adaptive_thresh_blocksize,
                        params.adaptive_thresh_const);


  cv::Mat birdsview_median;
  cv::medianBlur(birdsview_binary, birdsview_median, params.filter_size_median);

  const int MINIMUM_PCA_POINT_NUMBER = 2;

  candidate_data.pca_data = steps_window.size() < MINIMUM_PCA_POINT_NUMBER
                                ? PCAData()
                                : JunctionClassifier::getOrientation(steps_window);

  candidate_data.pos_neg_ratio =
      neg_steps_window.empty()
          ? std::numeric_limits<double>::infinity()
          : static_cast<double>(pos_steps_window.size()) / neg_steps_window.size();

  const common::DynamicPolynomial& middle_lane = features.middle_lane_polynomial;
  candidate_data.angle_diff =
      JunctionClassifier::getAngleDiff(features, middle_lane, candidate_data.pca_data);
  candidate_data.dist_middlelane_pca_mean =
      getDistMiddleLanePcaMean(features, middle_lane, candidate_data.pca_data);


  candidate_data.horizontal_eigen_line_plus = JunctionClassifier::GetEigenLine(
      candidate_data.pca_data, offset_eigenline, lenght_eigenline, Alignment::HORIZONTAL);

  candidate_data.horizontal_eigen_line = JunctionClassifier::GetEigenLine(
      candidate_data.pca_data, 0, lenght_eigenline, Alignment::HORIZONTAL);

  candidate_data.horizontal_eigen_line_minus = JunctionClassifier::GetEigenLine(
      candidate_data.pca_data, -offset_eigenline, lenght_eigenline, Alignment::HORIZONTAL);

  candidate_data.horizontal_eigen_line_no_median = JunctionClassifier::GetEigenLine(
      candidate_data.pca_data, 0, lenght_eigenline, Alignment::HORIZONTAL);

  candidate_data.vertical_eigen_line_plus = JunctionClassifier::GetEigenLine(
      candidate_data.pca_data, params.vertical_offset, lenght_eigenline, Alignment::VERTICAL);

  candidate_data.vertical_eigen_line_minus = JunctionClassifier::GetEigenLine(
      candidate_data.pca_data, -params.vertical_offset, lenght_eigenline, Alignment::VERTICAL);


  const ImagePoints horizontal_eigen_line_plus_points =
      GetStepsOnMeanEigenLine(birdsview_median,
                              candidate_data.horizontal_eigen_line_plus,
                              params.lenght_step_ref_func,
                              params.min_point_dist,
                              params.threshold_step_eigenline);

  const ImagePoints horizontal_eigen_line_points =
      GetStepsOnMeanEigenLine(birdsview_median,
                              candidate_data.horizontal_eigen_line,
                              params.lenght_step_ref_func,
                              params.min_point_dist,
                              params.threshold_step_eigenline);

  const ImagePoints horizontal_eigen_line_minus_points =
      GetStepsOnMeanEigenLine(birdsview_median,
                              candidate_data.horizontal_eigen_line_minus,
                              params.lenght_step_ref_func,
                              params.min_point_dist,
                              params.threshold_step_eigenline);

  const ImagePoints horizontal_eigen_line_points_no_median =
      GetStepsOnMeanEigenLine(birdsview_binary,
                              candidate_data.horizontal_eigen_line_no_median,
                              params.lenght_step_ref_func,
                              params.min_point_dist,
                              params.threshold_step_eigenline);

  ImagePoints vertical_eigen_line_points_no_median;
  if (candidate_data.dist_middlelane_pca_mean > 0) {
    vertical_eigen_line_points_no_median =
        GetStepsOnMeanEigenLine(birdsview_binary,
                                candidate_data.vertical_eigen_line_plus,
                                params.lenght_step_ref_func,
                                params.min_point_dist,
                                params.threshold_step_eigenline);
  } else {
    vertical_eigen_line_points_no_median =
        GetStepsOnMeanEigenLine(birdsview_binary,
                                candidate_data.vertical_eigen_line_minus,
                                params.lenght_step_ref_func,
                                params.min_point_dist,
                                params.threshold_step_eigenline);
  }

  candidate_data.nr_horizontal_eigen_line_points = horizontal_eigen_line_points.size();
  candidate_data.nr_horizontal_eigen_line_plus_points =
      horizontal_eigen_line_plus_points.size();
  candidate_data.nr_horizontal_eigen_line_points_no_median =
      horizontal_eigen_line_points_no_median.size();
  candidate_data.nr_vertical_eigen_line_points_no_median =
      vertical_eigen_line_points_no_median.size();
  candidate_data.nr_horizontal_eigen_line_minus_points =
      horizontal_eigen_line_minus_points.size();

  const VehiclePoint pca_means_vehicle =
      features.birdsview_patch.imageToVehicle(candidate_data.pca_data.mean);


  candidate_data.score = pca_means_vehicle.x() < params.min_x_dist
                             ? Scoring{}
                             : getScore(candidate_data, params);


  if (JunctionClassifier::DEBUG_MODE) {
  //Save Image for Debug

  cv::Mat debug_birdsview = birdsview->clone();
  cv::cvtColor(debug_birdsview, debug_birdsview, cv::COLOR_GRAY2RGB);

  for (const ImagePoint& point : neg_steps_window) {
    cv::circle(debug_birdsview, toCV(point), 1, cv::Scalar(0, 255, 0), -1);
  }

  for (const ImagePoint& point : pos_steps_window) {
    cv::circle(debug_birdsview, toCV(point), 1, cv::Scalar(255, 0, 255), -1);
  }

  std::string folder = "/home/";
  std::string image_name = folder + 
    unique_id + "_" + 
    std::to_string(loop_counter);
  //cv::imwrite(image_name + ".jpg", debug_birdsview);

  std::string junction_features = std::to_string(candidate_data.pca_data.eigen_vals[0])
    + "," + std::to_string(candidate_data.pca_data.eigen_vals[1])
    + "," + std::to_string(candidate_data.angle_diff)
    + "," + std::to_string(candidate_data.dist_middlelane_pca_mean)
    + "," + std::to_string(candidate_data.nr_horizontal_eigen_line_minus_points)
    + "," + std::to_string(candidate_data.nr_horizontal_eigen_line_points)
    + "," + std::to_string(candidate_data.nr_horizontal_eigen_line_plus_points)
    + "," + std::to_string(candidate_data.pos_neg_ratio)
    + "," + std::to_string(candidate_data.nr_horizontal_eigen_line_points_no_median)
    + "," + std::to_string(candidate_data.nr_vertical_eigen_line_points_no_median);

  std::ofstream myfile;
  //myfile.open (image_name + ".csv");
  //myfile << junction_features;
  //myfile.close();

  }

  return candidate_data;
};

size_t JunctionClassifier::getClassifierId() const {
  return typeid(JunctionClassifier).hash_code();
}

JunctionClassifier::PCAData JunctionClassifier::getOrientation(const ImagePoints& pts) const {

  const Eigen::MatrixX2d data_points = common::toMatrix2D<Eigen::MatrixX2d>(pts);
  const Eigen::Vector2d mean = data_points.colwise().mean().transpose();
  const Eigen::MatrixX2d centered = data_points.rowwise() - mean.transpose();
  const Eigen::Matrix2d cov = centered.adjoint() * centered / data_points.rows();
  const Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> eigen_solver(cov);

  PCAData pca_result;
  pca_result.mean = mean;
  pca_result.eigen_vals = eigen_solver.eigenvalues();
  pca_result.eigen_vecs = eigen_solver.eigenvectors();

  pca_result.angle = common::toYaw(pca_result.eigen_vecs);

  return pca_result;
}

JunctionClassifier::Scoring JunctionClassifier::getScore(
    const CandidateData& candidate_data, const JunctionClassifier::Params& params) const {

  double is_stop_line_left = params.is_stop_line_left;
  double is_give_way_line_left = params.is_give_way_line_left;
  double is_stop_line_right = params.is_stop_line_right;
  double is_give_way_line_right = params.is_give_way_line_right;

  is_stop_line_left *=
      trapeze(params.stop_line_left_mean1, candidate_data.pca_data.eigen_vals[0]);

  is_stop_line_left *=
      trapeze(params.stop_line_left_mean0, candidate_data.pca_data.eigen_vals[1]);

  is_stop_line_left *= trapeze(params.angle_diff, candidate_data.angle_diff);

  is_stop_line_left *= trapeze(params.stop_line_left_middlelane_pca,
                               candidate_data.dist_middlelane_pca_mean);

  is_stop_line_left *= trapeze(params.stop_line_outer_line_steps,
                               candidate_data.nr_horizontal_eigen_line_minus_points);

  is_stop_line_left *= trapeze(params.stop_line_middle_line_steps,
                               candidate_data.nr_horizontal_eigen_line_points);

  is_stop_line_left *= trapeze(params.stop_line_outer_line_steps,
                               candidate_data.nr_horizontal_eigen_line_plus_points);

  is_stop_line_left *= trapeze(params.stop_line_pos_neg_ratio, candidate_data.pos_neg_ratio);

  is_stop_line_left *= trapeze(params.stop_middle_line_no_median,
                               candidate_data.nr_horizontal_eigen_line_points_no_median);

  is_stop_line_left *= trapeze(params.mirror_side_no_median,
                               candidate_data.nr_vertical_eigen_line_points_no_median);


  is_give_way_line_left *= trapeze(params.give_way_line_left_mean1,
                                   candidate_data.pca_data.eigen_vals[0]);

  is_give_way_line_left *= trapeze(params.give_way_line_left_mean0,
                                   candidate_data.pca_data.eigen_vals[1]);

  is_give_way_line_left *= trapeze(params.angle_diff, candidate_data.angle_diff);

  is_give_way_line_left *= trapeze(params.give_way_line_left_middlelane_pca,
                                   candidate_data.dist_middlelane_pca_mean);

  is_give_way_line_left *= trapeze(params.give_way_line_outer_line_steps,
                                   candidate_data.nr_horizontal_eigen_line_minus_points);

  is_give_way_line_left *= trapeze(params.give_way_line_middle_line_steps,
                                   candidate_data.nr_horizontal_eigen_line_points);

  is_give_way_line_left *= trapeze(params.give_way_line_outer_line_steps,
                                   candidate_data.nr_horizontal_eigen_line_plus_points);

  is_give_way_line_left *=
      trapeze(params.give_way_line_pos_neg_ratio, candidate_data.pos_neg_ratio);

  is_stop_line_right *=
      trapeze(params.stop_line_right_mean1, candidate_data.pca_data.eigen_vals[0]);


  is_stop_line_right *=
      trapeze(params.stop_line_right_mean0, candidate_data.pca_data.eigen_vals[1]);

  is_stop_line_right *= trapeze(params.angle_diff, candidate_data.angle_diff);

  is_stop_line_right *= trapeze(params.stop_line_right_middlelane_pca,
                                candidate_data.dist_middlelane_pca_mean);

  is_stop_line_right *= trapeze(params.stop_line_outer_line_steps,
                                candidate_data.nr_horizontal_eigen_line_minus_points);

  is_stop_line_right *= trapeze(params.stop_line_middle_line_steps,
                                candidate_data.nr_horizontal_eigen_line_points);

  is_stop_line_right *= trapeze(params.stop_line_outer_line_steps,
                                candidate_data.nr_horizontal_eigen_line_plus_points);

  is_stop_line_right *=
      trapeze(params.stop_line_pos_neg_ratio, candidate_data.pos_neg_ratio);

  is_give_way_line_right *= trapeze(params.give_way_line_right_mean1,
                                    candidate_data.pca_data.eigen_vals[0]);
  is_give_way_line_right *= trapeze(params.angle_diff, candidate_data.angle_diff);

  is_give_way_line_right *= trapeze(params.give_way_line_right_mean0,
                                    candidate_data.pca_data.eigen_vals[1]);

  is_give_way_line_right *= trapeze(params.give_way_line_right_middlelane_pca,
                                    candidate_data.dist_middlelane_pca_mean);

  is_give_way_line_right *= trapeze(params.give_way_line_outer_line_steps,
                                    candidate_data.nr_horizontal_eigen_line_minus_points);

  is_give_way_line_right *= trapeze(params.give_way_line_middle_line_steps,
                                    candidate_data.nr_horizontal_eigen_line_points);

  is_give_way_line_right *= trapeze(params.give_way_line_outer_line_steps,
                                    candidate_data.nr_horizontal_eigen_line_plus_points);

  is_give_way_line_right *=
      trapeze(params.give_way_line_pos_neg_ratio, candidate_data.pos_neg_ratio);

  is_stop_line_right *= trapeze(params.stop_middle_line_no_median,
                                candidate_data.nr_horizontal_eigen_line_points_no_median);

  is_give_way_line_left *= trapeze(params.give_way_middle_line_no_median,
                                   candidate_data.nr_horizontal_eigen_line_points_no_median);

  is_give_way_line_right *=
      trapeze(params.give_way_middle_line_no_median,
              candidate_data.nr_horizontal_eigen_line_points_no_median);

  is_stop_line_right *= trapeze(params.mirror_side_no_median,
                                candidate_data.nr_vertical_eigen_line_points_no_median);

  is_give_way_line_left *= trapeze(params.mirror_side_no_median,
                                   candidate_data.nr_vertical_eigen_line_points_no_median);

  is_give_way_line_right *= trapeze(params.mirror_side_no_median,
                                    candidate_data.nr_vertical_eigen_line_points_no_median);

  is_stop_line_left = boost::algorithm::clamp(is_stop_line_left, 0, 1);
  is_stop_line_right = boost::algorithm::clamp(is_stop_line_right, 0, 1);
  is_give_way_line_left = boost::algorithm::clamp(is_give_way_line_left, 0, 1);
  is_give_way_line_right = boost::algorithm::clamp(is_give_way_line_right, 0, 1);

  return {{is_stop_line_left, is_stop_line_right, is_give_way_line_left, is_give_way_line_right}};
}

double JunctionClassifier::trapeze(Trapeze params, double pos) const {
  double value;
  if (pos > (params.max + params.fall_dist)) {
    value = 0.0;
  } else if (pos >= params.min && pos <= params.max) {
    value = 1.0;
  } else if (pos < (params.min - params.fall_dist)) {
    value = 0.0;
  } else if (pos > params.max && pos <= (params.max + params.fall_dist)) {
    value = 1.0 - (pos - params.max) / params.fall_dist;
  } else if (pos >= (params.min - params.fall_dist) && pos < params.min) {
    value = (pos - (params.min - params.fall_dist)) / params.fall_dist;
  } else {
    value = 0.0;
  }
  return boost::algorithm::clamp(value, 0, 1);
}

ImagePoints JunctionClassifier::getStepPointsNoDoubling(int step_dist,
                                                        int step_function_lenght,
                                                        int step_function_offset,
                                                        int min_point_dist,
                                                        const cv::Mat& img,
                                                        int threshold,
                                                        bool direction) const {
  ImagePoints recognized_steps;
  for (int i = 0; i < img.rows; i = i + step_dist) {
    ScanLine line = ScanLine();
    if (direction) {
      line = ScanLine(ImagePoint(i, step_function_offset),
                      ImagePoint(i, img.rows - step_function_offset));
    } else {
      line = ScanLine(ImagePoint(i, img.rows - step_function_offset),
                      ImagePoint(i, step_function_offset));
    }

    ImagePoints steps = step_detection::detectStep(
        img, line, step_detection::createReferenceFunction(step_function_lenght), threshold, false);

    for (const ImagePoint& step : steps) {
      if (!std::any_of(recognized_steps.cbegin(),
                       recognized_steps.cend(),
                       [step, min_point_dist](const ImagePoint& ip) {
                         return std::abs(ip.x() - step.x()) < min_point_dist;
                       })) {
        recognized_steps.insert(recognized_steps.end(), steps.begin(), steps.end());
      }
    }
  }
  return recognized_steps;
}

ImagePoints JunctionClassifier::getLineStepPointsNoDoubling(const ScanLine& line,
                                                            int step_function_lenght,
                                                            int min_point_dist,
                                                            const cv::Mat& img,
                                                            int threshold) const {
  const ImagePoints steps = step_detection::detectStep(
      img, line, step_detection::createReferenceFunction(step_function_lenght), threshold, true);

  ImagePoints recognized_steps;
  for (const ImagePoint& step : steps) {
    if (!std::any_of(recognized_steps.cbegin(),
                     recognized_steps.cend(),
                     [step, min_point_dist](const ImagePoint& i) {
                       return std::abs(i[0] - step[0]) < min_point_dist;
                     })) {
      recognized_steps.insert(recognized_steps.end(), steps.begin(), steps.end());
    }
  }
  return recognized_steps;
}

double JunctionClassifier::getAngleDiff(const Features& features,
                                        const common::DynamicPolynomial& middle_lane,
                                        const PCAData& pca_data) const {

  // Two points are transformed from image to vehicle points to transform an
  // angle as well.
  // The times 20 is needed to make the eigen vector larger than one pixel.
  // After transformation in Vehicle Point could else happen rounding
  // mistakes.
  ImagePoint image_ref_point = ImagePoint(0, 0);
  ImagePoint image_eigen_zero_vector =
      common::round(pca_data.eigen_vecs.col(1) * 20.0).cast<int>();

  VehiclePoint vehicle_ref_point = features.birdsview_patch.imageToVehicle(image_ref_point);
  VehiclePoint vehicle_eigen_zero_vector =
      features.birdsview_patch.imageToVehicle(image_eigen_zero_vector);

  const double vehicle_angle_eigen_zero =
      atan2(vehicle_eigen_zero_vector[0] - vehicle_ref_point[0],
            vehicle_ref_point[1] - vehicle_eigen_zero_vector[1]);
  const double vehicle_angle_middle_poly =
      atan2(middle_lane.derivate().evaluate(vehicle_eigen_zero_vector[0]), 1);

  double angle_diff = vehicle_angle_eigen_zero - vehicle_angle_middle_poly;
  return angle_diff;
}


double JunctionClassifier::getDistMiddleLanePcaMean(const Features& features,
                                                    const common::DynamicPolynomial& middle_lane,
                                                    const PCAData& pca_result) const {

  const VehiclePoint vehicle_pca_mean =
      features.birdsview_patch.imageToVehicle(pca_result.mean);

  double dist_middlelane_pca_mean =
      vehicle_pca_mean[1] - middle_lane.evaluate(vehicle_pca_mean[0]);
  return dist_middlelane_pca_mean;
}

cv::Point JunctionClassifier::CvPointOrthogonalOffset(
    const cv::Point& point, int offset, const JunctionClassifier::Alignment alignment) const {
  cv::Point pointwithoffset = cv::Point();
  if (alignment == Alignment::VERTICAL) {
    pointwithoffset = point + cv::Point(offset, 0);
  } else if (alignment == Alignment::HORIZONTAL) {
    pointwithoffset = point + cv::Point(0, offset);
  }
  return pointwithoffset;
}

ImagePoints JunctionClassifier::GetStepsOnMeanEigenLine(const cv::Mat& birdsview,
                                                        const ScanLine& line,
                                                        const int& step_function_lenght,
                                                        const int& min_point_dist,
                                                        const int& threshold) const {

  return JunctionClassifier::getLineStepPointsNoDoubling(
      line, step_function_lenght, min_point_dist, birdsview, threshold);
}

ScanLine JunctionClassifier::GetEigenLine(const PCAData& pca_result,
                                          const int& offset,
                                          const double& eigen_lenght,
                                          const JunctionClassifier::Alignment alignment) const {
  cv::Point eigen_vec;
  if (alignment == Alignment::HORIZONTAL) {
    eigen_vec = toCV(common::round(pca_result.eigen_vecs.col(1)).cast<int>());
  } else if (alignment == Alignment::VERTICAL) {
    eigen_vec = toCV(common::round(pca_result.eigen_vecs.col(0)).cast<int>());
  }


  cv::Point mean = toCV(common::round(pca_result.mean).cast<int>());
  cv::Point mean_and_offset =
      JunctionClassifier::CvPointOrthogonalOffset(mean, offset, alignment);
  cv::Point mean_and_offset_start = mean_and_offset + eigen_vec * eigen_lenght;
  cv::Point mean_and_offset_end = mean_and_offset - eigen_vec * eigen_lenght;

  return {toEigen(mean_and_offset_start), toEigen(mean_and_offset_end)};
}

VehiclePoints JunctionClassifier::computeBaseHullPolygon(const Features& /*features*/,
                                                         const VehiclePose& line_pose,
                                                         const Junction::JunctionType type) const {
  const double lane_width = 0.4;
  const double width = type == Junction::givewayline_left || type == Junction::stopline_left
                           ? -lane_width
                           : lane_width;
  const double start_x = 0.05;
  const double end_x = 0.35;
  const std::array<VehiclePoint, 4> hull_polygon_in_junction_pose = {
      {{end_x, 0, 0}, {start_x, 0, 0}, {start_x, -width, 0}, {end_x, -width, 0}}};

  return common::make_eigen_vector(hull_polygon_in_junction_pose |
                                   common::eigen_transformed(line_pose));
}

VehiclePose JunctionClassifier::getLinePose(const Features& features,
                                            const PCAData& pca_result) const {

  const VehiclePoint pca_means_vehicle =
      features.birdsview_patch.imageToVehicle(pca_result.mean);
  const VehiclePoint pca_on_middlelane =
      to3D(common::point(features.middle_lane_polynomial, pca_means_vehicle.x()));

  return Eigen::Translation3d(pca_on_middlelane) *
         Eigen::AngleAxisd(features.cluster_center_lane_orientation,
                           Eigen::Vector3d::UnitZ());
}


}  // namespace road_object_detection
