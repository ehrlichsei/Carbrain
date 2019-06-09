#include "../../../src/perpendicular_parking/parking_end_classifier.h"
#include <common/angle_conversions.h>
#include <common/macros.h>
#include <common/test/dummy_parameter_handler.h>
#include "../../../src/perpendicular_parking/parking_lot.h"
#include "../../../src/utils/tf_helper.h"
#include "parking_lot_generator.h"

THIRD_PARTY_HEADERS_BEGIN
#include <gtest/gtest.h>
#include <tf2_ros/buffer.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
THIRD_PARTY_HEADERS_END

DISABLE_SUGGEST_OVERRIDE_WARNING

namespace perpendicular_parking {
namespace testing {

/*!
 * \brief The MyDummyParamHandler class
 * DummyParameterHandler that already contains all parameters needed for the
 * test
 */
class MyDummyParamHandler : public DummyParameterHandler {

 public:
  MyDummyParamHandler() {
    addParam(LEFT_LANE_POLY_STEP, 0.02);
    addParam(SCAN_LINE_LENGTH, 0.45);
    addParam(STEP_DETECTION_THLD, 3.5);
    addParam(STEP_DETECTION_REF_FUNC_SIZE, 9);
    addParam(PARKING_SPOT_DEPTH, 0.35);
    addParam(LEFT_LANE_SCANLINE_PADDING, 0.05);
    addParam(ADDITIONAL_PADDING, 0.1);

    addParam(SCANLINE_X_OFFSET, 0.2);
    addParam(MINIMAL_HYPOTHESE_BELIEF, 0.85);
    addParam(MARKING_SCANLINE_SHIFT, 0.5);
    addParam(NO_MARKING_SCANLINES, 10);
    addParam(MIN_CLUSTER_SIZE, 7);

    addParam(RANSAC_MIN_DIST, 0.02);
    addParam(RANSAC_NR_LINES, 4);
    addParam(RANSAC_MIN_CLUSTER_SIZE, 5);
    addParam(MINIMUM_ENDLINE_ANGLE, -0.9);
    addParam(MAXIMUM_ENDLINE_ANGLE, -0.42);

    // Initialize camera parameters
    addParam(CAMERA_FOCAL_LENGTH_X, 386.704446);
    addParam(CAMERA_FOCAL_LENGTH_Y, 385.973724);
    addParam(CAMERA_OPTICAL_CENTER_X, 623.233113);
    addParam(CAMERA_OPTICAL_CENTER_Y, 125.752428);

    addParam(CAMERA_R11, 0.0214821);
    addParam(CAMERA_R12, -0.999761);
    addParam(CAMERA_R13, -0.00402491);
    addParam(CAMERA_R21, -0.149211);
    addParam(CAMERA_R22, 0.00077467);
    addParam(CAMERA_R23, -0.988805);
    addParam(CAMERA_R31, 0.988572);
    addParam(CAMERA_R32, 0.0218421);
    addParam(CAMERA_R33, -0.149159);

    addParam(CAMERA_T1, 7.42816);
    addParam(CAMERA_T2, 262.261);
    addParam(CAMERA_T3, 32.6001);

    // World coordinate helper parameters
    addParam(MAX_TF_LOOKUP_DURATION, 0.0);

    // dbscan
    addParam(DBSCAN_CORE_EPSILON, 0.1);
    addParam(DBSCAN_MIN_POINTS_TO_BE_CORE, 1);

    // parking_start_classifier
    addParam(ParameterString<double>("parking_start_classifier/horizon"), 2.2);
    addParam(ParameterString<double>(
                 "parking_start_classifier/max_distance_end_of_points"),
             0.15);
    addParam(
        ParameterString<double>("parking_start_classifier/max_angle_delta"), 0.175);
    addParam(
        ParameterString<double>("parking_start_classifier/second_pca_thld"), 0.04);
    addParam(ParameterString<double>(
                 "parking_start_classifier/max_dist_to_be_close"),
             0.15);
    addParam(
        ParameterString<int>("parking_start_classifier/min_nr_close_points"), 2);
    addParam(
        ParameterString<double>("parking_start_classifier/required_certainty"), 0.9);

    // db_scan
    addParam(ParameterString<double>("dbscan/core_epsilon"), 0.1);
    addParam(ParameterString<int>("dbscan/min_points_to_be_core"), 4);


    // parking lot
    addParam(PARKING_SPOT_DEPTH, 0.5);

    addParam(STEP_REFERENCE_FUNCTION_LENGTH, 9);
    addParam(GRADIENT_DETECTION_THLD, 4.0);

    addParam(IMAGE_PADDING, 10);

    addParam(MARKING_SCAN_STEP_WIDTH, 0.02);
    addParam(MARKING_SCAN_LINE_WIDTH, 0.08);
    addParam(MARKKING_SCAN_LINE_PADDING_FRONT, 0.07);
    addParam(MARKKING_SCAN_LINE_PADDING_BACK, 0.15);

    addParam(N_SCAN_LINES_PER_SPOT, 19);
    addParam(SPOT_SCAN_LINE_PADDING_X, 0.03);
    addParam(SPOT_SCAN_LINE_PADDING_Y, 0.08);
    addParam(SPOT_SCAN_LINE_OBSTACLE_SAFETY_MARGIN, 0.02);

    addParam(N_MIN_MARKINGS, 3);
    addParam(MAX_SPAWN_DISTANCE, 1.8);
    addParam(MAX_MARKING_UPDATE_DISTANCE, 0.8);
    addParam(MARKING_UPDATE_SHIFT_FACTOR, 0.7);
    addParam(START_POSE_MAX_ALLOWED_DEVIATION, 0.2);

    addParam(PARKING_SPOT_WIDTH, 0.35);
    addParam(MARKING_WIDTH, 0.35);

    addParam(NEWTON_EPSILON, 0.01);
    addParam(NEWTON_MAX_ITERATIONS, 20);

    addParam(FIELD_OF_VISION_TOP, 90);
    addParam(FIELD_OF_VISION_BOTTOM, 550);
    addParam(FIELD_OF_VISION_LEFT, 0);
    addParam(FIELD_OF_VISION_RIGHT, 1289);

    addParam(POINTS_TO_SKIP_FRONT, 3);
    addParam(POINTS_TO_SKIP_BACK, -2);
    addParam(EPSILON_X_CLASSIFIER, 0.2);
    addParam(MIN_NUMBER_HITS, 4);
    addParam(SMALLEST_OBSTACLE_AREA, 0.02);
    addParam(FREE_MIN_DEPTH_TH, 0.3);

    addParam(
        ParameterString<double>("parking_line/max_time_with_no_observation"), 0.1);
    addParam(ParameterString<double>("parking_line/init_covariance"), 1000.0);
    addParam(ParameterString<double>("parking_line/max_position_delta"), 0.1);
    addParam(MAX_ANGLE_DELTA, 0.1);

    addParam(ParameterString<double>("endline/min_free_space"), 0.45);
    addParam(ParameterString<double>("endline/min_free_space_belief"), 0.5);

    addParam(
        ParameterString<double>("parking_spot/ransac/minimal_model_distance"), 0.01);
    addParam(
        ParameterString<int>("parking_spot/ransac/minimal_consensus_set_size"), 5);
    addParam(ParameterString<int>("parking_spot/ransac/expected_nr_of_lines"), 3);
  }

  // ParkingEndClassifier class parameters
  static const std::string NAMESPACE;

  static const ParameterString<double> LEFT_LANE_POLY_STEP;
  static const ParameterString<double> SCAN_LINE_LENGTH;
  static const ParameterString<double> STEP_DETECTION_THLD;
  static const ParameterString<int> STEP_DETECTION_REF_FUNC_SIZE;
  static const ParameterString<double> PARKING_SPOT_DEPTH;
  static const ParameterString<double> PARKING_SPOT_WIDTH;
  static const ParameterString<double> MARKING_WIDTH;
  static const ParameterString<double> LEFT_LANE_SCANLINE_PADDING;
  static const ParameterString<double> ADDITIONAL_PADDING;
  static const ParameterString<double> SCANLINE_X_OFFSET;
  static const ParameterString<double> MINIMAL_HYPOTHESE_BELIEF;
  static const ParameterString<double> MARKING_SCANLINE_SHIFT;
  static const ParameterString<int> NO_MARKING_SCANLINES;
  static const ParameterString<int> MIN_CLUSTER_SIZE;
  static const ParameterString<double> RANSAC_MIN_DIST;
  static const ParameterString<int> RANSAC_MIN_CLUSTER_SIZE;
  static const ParameterString<int> RANSAC_NR_LINES;
  static const ParameterString<double> MINIMUM_ENDLINE_ANGLE;
  static const ParameterString<double> MAXIMUM_ENDLINE_ANGLE;

  // Camera Parameters
  static const ParameterString<double> CAMERA_FOCAL_LENGTH_X;
  static const ParameterString<double> CAMERA_FOCAL_LENGTH_Y;
  static const ParameterString<double> CAMERA_OPTICAL_CENTER_X;
  static const ParameterString<double> CAMERA_OPTICAL_CENTER_Y;

  static const ParameterString<double> CAMERA_R11;
  static const ParameterString<double> CAMERA_R12;
  static const ParameterString<double> CAMERA_R13;
  static const ParameterString<double> CAMERA_R21;
  static const ParameterString<double> CAMERA_R22;
  static const ParameterString<double> CAMERA_R23;
  static const ParameterString<double> CAMERA_R31;
  static const ParameterString<double> CAMERA_R32;
  static const ParameterString<double> CAMERA_R33;

  static const ParameterString<double> CAMERA_T1;
  static const ParameterString<double> CAMERA_T2;
  static const ParameterString<double> CAMERA_T3;

  // World coorfinate helper parameters
  static const ParameterString<double> MAX_TF_LOOKUP_DURATION;

  // dbscan
  static const ParameterString<double> DBSCAN_CORE_EPSILON;
  static const ParameterString<int> DBSCAN_MIN_POINTS_TO_BE_CORE;

  // parking lot
  static const std::string NAMESPACE_LOT;

  static const ParameterString<int> STEP_REFERENCE_FUNCTION_LENGTH;
  static const ParameterString<double> GRADIENT_DETECTION_THLD;

  static const ParameterString<int> IMAGE_PADDING;

  static const ParameterString<double> MARKING_SCAN_STEP_WIDTH;
  static const ParameterString<double> MARKING_SCAN_LINE_WIDTH;
  static const ParameterString<double> MARKKING_SCAN_LINE_PADDING_FRONT;
  static const ParameterString<double> MARKKING_SCAN_LINE_PADDING_BACK;

  static const ParameterString<int> N_SCAN_LINES_PER_SPOT;
  static const ParameterString<double> SPOT_SCAN_LINE_PADDING_X;
  static const ParameterString<double> SPOT_SCAN_LINE_PADDING_Y;
  static const ParameterString<double> SPOT_SCAN_LINE_OBSTACLE_SAFETY_MARGIN;

  static const ParameterString<int> N_MIN_MARKINGS;
  static const ParameterString<double> MAX_SPAWN_DISTANCE;
  static const ParameterString<double> MAX_MARKING_UPDATE_DISTANCE;
  static const ParameterString<double> MARKING_UPDATE_SHIFT_FACTOR;
  static const ParameterString<double> START_POSE_MAX_ALLOWED_DEVIATION;

  static const ParameterString<double> NEWTON_EPSILON;
  static const ParameterString<int> NEWTON_MAX_ITERATIONS;

  static const ParameterString<int> POINTS_TO_SKIP_FRONT;
  static const ParameterString<int> POINTS_TO_SKIP_BACK;
  static const ParameterString<double> EPSILON_X_CLASSIFIER;
  static const ParameterString<int> MIN_NUMBER_HITS;
  static const ParameterString<double> SMALLEST_OBSTACLE_AREA;
  static const ParameterString<double> FREE_MIN_DEPTH_TH;

  static const ParameterString<int> FIELD_OF_VISION_TOP;
  static const ParameterString<int> FIELD_OF_VISION_BOTTOM;
  static const ParameterString<int> FIELD_OF_VISION_LEFT;
  static const ParameterString<int> FIELD_OF_VISION_RIGHT;

  static const ParameterString<double> MAX_ANGLE_DELTA;
};

const std::string MyDummyParamHandler::NAMESPACE("parking_end_classifier");

const ParameterString<double> MyDummyParamHandler::LEFT_LANE_POLY_STEP(
    NAMESPACE + "/left_lane_poly_step");
const ParameterString<double> MyDummyParamHandler::SCAN_LINE_LENGTH(
    NAMESPACE + "/scan_line_length");
const ParameterString<double> MyDummyParamHandler::STEP_DETECTION_THLD(
    NAMESPACE + "/step_detection_threshold");
const ParameterString<int> MyDummyParamHandler::STEP_DETECTION_REF_FUNC_SIZE(
    NAMESPACE + "/step_detection_ref_func_size");
const ParameterString<double> MyDummyParamHandler::PARKING_SPOT_DEPTH(
    "parking_spot_depth");
const ParameterString<double> MyDummyParamHandler::PARKING_SPOT_WIDTH(
    "parking_spot_width");
const ParameterString<double> MyDummyParamHandler::MARKING_WIDTH(
    "marking_width");
const ParameterString<double> MyDummyParamHandler::LEFT_LANE_SCANLINE_PADDING(
    NAMESPACE + "/left_lane_scanline_padding");
const ParameterString<double> MyDummyParamHandler::ADDITIONAL_PADDING(
    NAMESPACE + "/additional_padding");
const ParameterString<double> MyDummyParamHandler::SCANLINE_X_OFFSET(
    NAMESPACE + "/scanline_x_offset");
const ParameterString<double> MyDummyParamHandler::MINIMAL_HYPOTHESE_BELIEF(
    NAMESPACE + "/minimal_hypothese_belief");
const ParameterString<double> MyDummyParamHandler::MARKING_SCANLINE_SHIFT(
    NAMESPACE + "/marking_scanline_shift");
const ParameterString<int> MyDummyParamHandler::NO_MARKING_SCANLINES(
    NAMESPACE + "/no_marking_scanlines");
const ParameterString<int> MyDummyParamHandler::MIN_CLUSTER_SIZE(
    NAMESPACE + "/min_cluster_size");
const ParameterString<double> MyDummyParamHandler::RANSAC_MIN_DIST(
    NAMESPACE + "/ransac/minimal_model_distance");
const ParameterString<int> MyDummyParamHandler::RANSAC_MIN_CLUSTER_SIZE(
    NAMESPACE + "/ransac/minimal_consensus_set_size");
const ParameterString<int> MyDummyParamHandler::RANSAC_NR_LINES(
    NAMESPACE + "/ransac/expected_nr_of_lines");
const ParameterString<double> MyDummyParamHandler::MINIMUM_ENDLINE_ANGLE(
    NAMESPACE + "/minimum_endline_angle");
const ParameterString<double> MyDummyParamHandler::MAXIMUM_ENDLINE_ANGLE(
    NAMESPACE + "/maximum_endline_angle");

// Camera Parameters
const ParameterString<double> MyDummyParamHandler::CAMERA_FOCAL_LENGTH_X(
    "/camera/focal_length_x");
const ParameterString<double> MyDummyParamHandler::CAMERA_FOCAL_LENGTH_Y(
    "/camera/focal_length_y");
const ParameterString<double> MyDummyParamHandler::CAMERA_OPTICAL_CENTER_X(
    "/camera/optical_center_x");
const ParameterString<double> MyDummyParamHandler::CAMERA_OPTICAL_CENTER_Y(
    "/camera/optical_center_y");

const ParameterString<double> MyDummyParamHandler::CAMERA_R11("/camera/r11");
const ParameterString<double> MyDummyParamHandler::CAMERA_R12("/camera/r12");
const ParameterString<double> MyDummyParamHandler::CAMERA_R13("/camera/r13");
const ParameterString<double> MyDummyParamHandler::CAMERA_R21("/camera/r21");
const ParameterString<double> MyDummyParamHandler::CAMERA_R22("/camera/r22");
const ParameterString<double> MyDummyParamHandler::CAMERA_R23("/camera/r23");
const ParameterString<double> MyDummyParamHandler::CAMERA_R31("/camera/r31");
const ParameterString<double> MyDummyParamHandler::CAMERA_R32("/camera/r32");
const ParameterString<double> MyDummyParamHandler::CAMERA_R33("/camera/r33");

const ParameterString<double> MyDummyParamHandler::CAMERA_T1("/camera/t1");
const ParameterString<double> MyDummyParamHandler::CAMERA_T2("/camera/t2");
const ParameterString<double> MyDummyParamHandler::CAMERA_T3("/camera/t3");

// coordinate helper
const ParameterString<double> MyDummyParamHandler::MAX_TF_LOOKUP_DURATION(
    "max_tf_lookup_duration");

// dbscan
const ParameterString<double> MyDummyParamHandler::DBSCAN_CORE_EPSILON(
    NAMESPACE + "/dbscan/core_epsilon");
const ParameterString<int> MyDummyParamHandler::DBSCAN_MIN_POINTS_TO_BE_CORE(
    NAMESPACE + "/dbscan/min_points_to_be_core");

const std::string MyDummyParamHandler::NAMESPACE_LOT("parking_lot");

const ParameterString<int> MyDummyParamHandler::STEP_REFERENCE_FUNCTION_LENGTH(
    NAMESPACE_LOT + "/step_reference_function_length");
const ParameterString<double> MyDummyParamHandler::GRADIENT_DETECTION_THLD(
    NAMESPACE_LOT + "/gradient_detection_thld");

const ParameterString<int> MyDummyParamHandler::IMAGE_PADDING(
    NAMESPACE_LOT + "/image_padding");

const ParameterString<double> MyDummyParamHandler::MARKING_SCAN_STEP_WIDTH(
    NAMESPACE_LOT + "/marking_scan_step_width");
;
const ParameterString<double> MyDummyParamHandler::MARKING_SCAN_LINE_WIDTH(
    NAMESPACE_LOT + "/marking_scan_line_width");
;
const ParameterString<double> MyDummyParamHandler::MARKKING_SCAN_LINE_PADDING_FRONT(
    NAMESPACE_LOT + "/marking_scan_line_padding_front");
const ParameterString<double> MyDummyParamHandler::MARKKING_SCAN_LINE_PADDING_BACK(
    NAMESPACE_LOT + "/marking_scan_line_padding_back");

const ParameterString<int> MyDummyParamHandler::N_SCAN_LINES_PER_SPOT(
    NAMESPACE_LOT + "/n_scan_lines_per_spot");
const ParameterString<double> MyDummyParamHandler::SPOT_SCAN_LINE_PADDING_X(
    "spot_scan_line_padding_x");
const ParameterString<double> MyDummyParamHandler::SPOT_SCAN_LINE_PADDING_Y(
    NAMESPACE_LOT + "/spot_scan_line_padding_y");
const ParameterString<double> MyDummyParamHandler::SPOT_SCAN_LINE_OBSTACLE_SAFETY_MARGIN(
    NAMESPACE_LOT + "/spot_scan_line_obstacle_safety_margin");

const ParameterString<int> MyDummyParamHandler::N_MIN_MARKINGS(
    NAMESPACE_LOT + "/n_min_markings");
const ParameterString<double> MyDummyParamHandler::MAX_SPAWN_DISTANCE(
    NAMESPACE_LOT + "/max_spawn_distance");
const ParameterString<double> MyDummyParamHandler::MAX_MARKING_UPDATE_DISTANCE(
    NAMESPACE_LOT + "/max_marking_update_distance");
const ParameterString<double> MyDummyParamHandler::MARKING_UPDATE_SHIFT_FACTOR(
    NAMESPACE_LOT + "/marking_update_shift_factor");
const ParameterString<double> MyDummyParamHandler::START_POSE_MAX_ALLOWED_DEVIATION(
    NAMESPACE_LOT + "/start_pose_max_allowed_deviation");

const ParameterString<double> MyDummyParamHandler::NEWTON_EPSILON(
    NAMESPACE_LOT + "/newton_epsilon");
const ParameterString<int> MyDummyParamHandler::NEWTON_MAX_ITERATIONS(
    NAMESPACE_LOT + "/newton_max_iterations");

const ParameterString<int> MyDummyParamHandler::FIELD_OF_VISION_TOP(
    "parking_lot/field_of_vision/top");
const ParameterString<int> MyDummyParamHandler::FIELD_OF_VISION_BOTTOM(
    "parking_lot/field_of_vision/bottom");
const ParameterString<int> MyDummyParamHandler::FIELD_OF_VISION_LEFT(
    "parking_lot/field_of_vision/left");
const ParameterString<int> MyDummyParamHandler::FIELD_OF_VISION_RIGHT(
    "parking_lot/field_of_vision/right");

const ParameterString<int> MyDummyParamHandler::POINTS_TO_SKIP_FRONT(
    "parking_spot/points_to_skip_front");
const ParameterString<int> MyDummyParamHandler::POINTS_TO_SKIP_BACK(
    "parking_spot/points_to_skip_back");
const ParameterString<double> MyDummyParamHandler::EPSILON_X_CLASSIFIER(
    "parking_spot/epsilon_x_classifier");
const ParameterString<int> MyDummyParamHandler::MIN_NUMBER_HITS(
    "parking_spot/min_number_hits");
const ParameterString<double> MyDummyParamHandler::SMALLEST_OBSTACLE_AREA(
    "parking_spot/smallest_obstacle_area");
const ParameterString<double> MyDummyParamHandler::FREE_MIN_DEPTH_TH(
    "parking_spot/free_min_depth_th");

// parking_line
const ParameterString<double> MyDummyParamHandler::MAX_ANGLE_DELTA(
    "parking_line/max_angle_delta");

template <typename Scalar>
class TFHelperMock : public tf_helper::TFHelperInterface<Scalar> {
 public:
  bool update(const ros::Time &) override { return true; }

  typename tf_helper::TFHelperInterface<Scalar>::AffineTransformation getTransform() const override {
    return tf_helper::TFHelperInterface<Scalar>::AffineTransformation::Identity();
  }
};

struct TestData {
  cv::Mat img_;
  common::DynamicPolynomial left_lane_polynomial_;
  VehiclePoints left_lane_points_;
  std::unique_ptr<ParkingLot> parking_lot_;
};

// Test fixure
class ParkingEndClassifierTest : public ::testing::Test {

 public:
  ParkingEndClassifierTest()
      : param_(), camera_transformation_(&param_), world_coordinates_helper_() {

    world_coordinates_helper_.update(ros::Time(0));
  }

  ~ParkingEndClassifierTest() = default;

 protected:
  MyDummyParamHandler param_;
  common::CameraTransformation camera_transformation_;
  TFHelperMock<double> world_coordinates_helper_;

  TestData generateTestData(
      const std::vector<GeneratedOccupationState> &generated_occupation_states,
      const double parking_start_line_angle_in_radian = 1.0472,
      const double parking_slot_width_in_m = 0.350,
      const double parking_slot_depth_in_m = 0.500,
      double angle_lot_to_vehicle = 0.0,
      Eigen::Vector3d translation_lot_to_vehicle = Eigen::Vector3d(0.3, 0.4, 0.0)) {

    ParkingLotGenerator parking_lot_generator(generated_occupation_states,
                                              parking_start_line_angle_in_radian,
                                              parking_slot_width_in_m,
                                              parking_slot_depth_in_m);
    std::unique_ptr<ParkingLot> parking_lot = std::make_unique<ParkingLot>(
        &param_, &camera_transformation_, &world_coordinates_helper_);

    // construct points
    const auto start_line_pose = parking_lot_generator.getParkingStartPose();
    // default startline
    const auto start_line_cluster =
        toStartLineCluster(start_line_pose, camera_transformation_);
    // get left lane points
    VehiclePoints left_lane_points;
    left_lane_points.reserve(100);
    parking_lot_generator.getLeftLanePoints(left_lane_points, 100);

    // Transformations
    auto parkinglot_to_vehicle_transform = Eigen::Affine3d::Identity();

    // Note: do rotation first translation always in new coordinates
    const auto angle_axis =
        Eigen::AngleAxisd(angle_lot_to_vehicle, Eigen::Vector3d::UnitZ());
    parkinglot_to_vehicle_transform.rotate(angle_axis);
    parkinglot_to_vehicle_transform =
        Eigen::Translation3d(translation_lot_to_vehicle) * parkinglot_to_vehicle_transform;

    const auto left_lane_polynomial =
        parking_lot_generator.getLeftLanePolynom(parkinglot_to_vehicle_transform);

    // generate image
    cv::Mat img;
    parking_lot_generator.drawParkingLot(
        img, parkinglot_to_vehicle_transform, camera_transformation_);

    const auto parkinglot_to_world_transform = parkinglot_to_vehicle_transform;

    const auto parking_start_pose =
        parkinglot_to_world_transform * parking_lot_generator.getParkingStartPose();

    const auto parking_stop_pose =
        parkinglot_to_world_transform * parking_lot_generator.getParkingEndPose();

    parking_lot->setStart(parking_start_pose);
    parking_lot->setEnd(parking_stop_pose);

    for (std::size_t i = 0; i < 20; i++) {
      parking_lot->update(img, left_lane_polynomial);
    }

    return TestData{img, left_lane_polynomial, left_lane_points, std::move(parking_lot)};
  }

  FeaturePointCluster toStartLineCluster(const Eigen::Affine3d &pose,
                                         const common::CameraTransformation &cam_trafo,
                                         const double angle = 1.0472,
                                         const double depth = 0.5,
                                         const double step = 0.02) const {
    const auto length = depth / std::fabs(std::sin(angle));
    const auto nr_points = static_cast<std::size_t>(length / step);

    VehiclePoints start_line_points;
    start_line_points.reserve(nr_points);
    for (std::size_t i = 0; i < nr_points; i++) {
      start_line_points.emplace_back(
          pose.translation() +
          step * i * (pose.linear() * VehiclePoint::UnitX()).normalized());
    }

    ImagePoints feature_points_img;
    cam_trafo.transformGroundToImage(start_line_points, &feature_points_img);

    return FeaturePointCluster{feature_points_img, start_line_points};
  }
};

TEST_F(ParkingEndClassifierTest, NoParkingSpots) {
  ParkingSpotsConstRef spots;
  cv::Mat img;
  ros::Time timestamp;
  common::DynamicPolynomial left_lane_polynom;
  LineVehiclePoints lanes;
  MapPose world_T_map;

  std::unique_ptr<ParkingEndClassifier> parking_end_classifier =
      std::make_unique<ParkingEndClassifier>(
          &param_, &camera_transformation_, &world_coordinates_helper_);

  lanes[LINESPEC_LEFT] = VehiclePoints();

  EXPECT_EQ(boost::none,
            parking_end_classifier->detectEnd(
                spots, img, timestamp, left_lane_polynom, lanes, world_T_map));
}

TEST_F(ParkingEndClassifierTest, NoLanes) {
  ros::Time timestamp;
  LineVehiclePoints lanes;

  std::unique_ptr<ParkingEndClassifier> parking_end_classifier =
      std::make_unique<ParkingEndClassifier>(
          &param_, &camera_transformation_, &world_coordinates_helper_);

  std::vector<GeneratedOccupationState> generated_occupation_states = {
      GeneratedOccupationState::FREE, GeneratedOccupationState::X, GeneratedOccupationState::FREE};

  const auto test_data = generateTestData(generated_occupation_states);

  EXPECT_EQ(boost::none,
            parking_end_classifier->detectEnd(test_data.parking_lot_->allParkingSpots(),
                                              test_data.img_,
                                              timestamp,
                                              test_data.left_lane_polynomial_,
                                              lanes,
                                              test_data.parking_lot_->mapPose()));
}

TEST_F(ParkingEndClassifierTest, ValidEndLine) {
  ros::Time timestamp;
  const double test_angle =
      (param_.getParam(MyDummyParamHandler::MAXIMUM_ENDLINE_ANGLE) +
       param_.getParam(MyDummyParamHandler::MINIMUM_ENDLINE_ANGLE)) /
      2.0;

  std::unique_ptr<ParkingEndClassifier> parking_end_classifier =
      std::make_unique<ParkingEndClassifier>(
          &param_, &camera_transformation_, &world_coordinates_helper_);

  std::vector<GeneratedOccupationState> generated_occupation_states = {
      GeneratedOccupationState::FREE, GeneratedOccupationState::X, GeneratedOccupationState::FREE};

  const auto test_data = generateTestData(generated_occupation_states, -test_angle);

  LineVehiclePoints lanes;
  lanes[LINESPEC_LEFT] = test_data.left_lane_points_;

  boost::optional<WorldPose> parking_lot_end;

  std::size_t nr_of_updates = 20;
  for (std::size_t i = 0; i < nr_of_updates; i++) {
    parking_lot_end =
        parking_end_classifier->detectEnd(test_data.parking_lot_->allParkingSpots(),
                                          test_data.img_,
                                          timestamp,
                                          test_data.left_lane_polynomial_,
                                          lanes,
                                          test_data.parking_lot_->mapPose());

    timestamp = ros::Time(i * 0.1);
  }

  EXPECT_NE(boost::none, parking_lot_end);
}

TEST_F(ParkingEndClassifierTest, AngleTooSmall) {
  ros::Time timestamp;
  const double test_angle =
      param_.getParam(MyDummyParamHandler::MINIMUM_ENDLINE_ANGLE) -
      1.1 * param_.getParam(MyDummyParamHandler::MAX_ANGLE_DELTA);

  const double parking_start_line_angle_in_radian = -test_angle;
  const double parking_slot_width_in_m = 0.350;
  const double parking_slot_depth_in_m = 0.500;

  std::unique_ptr<ParkingEndClassifier> parking_end_classifier =
      std::make_unique<ParkingEndClassifier>(
          &param_, &camera_transformation_, &world_coordinates_helper_);

  std::vector<GeneratedOccupationState> generated_occupation_states = {
      GeneratedOccupationState::FREE, GeneratedOccupationState::FREE};

  const Eigen::Vector3d test_translation(0.0, 0.8, 0.0);

  const auto test_data = generateTestData(generated_occupation_states,
                                          parking_start_line_angle_in_radian,
                                          parking_slot_width_in_m,
                                          parking_slot_depth_in_m,
                                          test_angle,
                                          test_translation);

  LineVehiclePoints lanes;
  lanes[LINESPEC_LEFT] = test_data.left_lane_points_;

  boost::optional<WorldPose> parking_lot_end;

  std::size_t nr_of_updates = 20;
  for (std::size_t i = 0; i < nr_of_updates; i++) {
    parking_lot_end =
        parking_end_classifier->detectEnd(test_data.parking_lot_->allParkingSpots(),
                                          test_data.img_,
                                          timestamp,
                                          test_data.left_lane_polynomial_,
                                          lanes,
                                          test_data.parking_lot_->mapPose());

    timestamp = ros::Time(i * 0.1);
  }

  EXPECT_EQ(boost::none, parking_lot_end);
}

TEST_F(ParkingEndClassifierTest, AngleTooLarge) {
  ros::Time timestamp;
  const double test_angle =
      param_.getParam(MyDummyParamHandler::MAXIMUM_ENDLINE_ANGLE) +
      1.1 * param_.getParam(MyDummyParamHandler::MAX_ANGLE_DELTA);

  const double parking_start_line_angle_in_radian = -test_angle;
  const double parking_slot_width_in_m = 0.350;
  const double parking_slot_depth_in_m = 0.500;

  std::unique_ptr<ParkingEndClassifier> parking_end_classifier =
      std::make_unique<ParkingEndClassifier>(
          &param_, &camera_transformation_, &world_coordinates_helper_);

  std::vector<GeneratedOccupationState> generated_occupation_states = {
      GeneratedOccupationState::FREE, GeneratedOccupationState::FREE};

  const Eigen::Vector3d test_translation(0.0, 0.2, 0.0);

  const auto test_data = generateTestData(generated_occupation_states,
                                          parking_start_line_angle_in_radian,
                                          parking_slot_width_in_m,
                                          parking_slot_depth_in_m,
                                          test_angle,
                                          test_translation);

  LineVehiclePoints lanes;
  lanes[LINESPEC_LEFT] = test_data.left_lane_points_;

  boost::optional<WorldPose> parking_lot_end;

  std::size_t nr_of_updates = 20;
  for (std::size_t i = 0; i < nr_of_updates; i++) {
    parking_lot_end =
        parking_end_classifier->detectEnd(test_data.parking_lot_->allParkingSpots(),
                                          test_data.img_,
                                          timestamp,
                                          test_data.left_lane_polynomial_,
                                          lanes,
                                          test_data.parking_lot_->mapPose());

    timestamp = ros::Time(i * 0.1);
  }

  EXPECT_EQ(boost::none, parking_lot_end);
}

TEST_F(ParkingEndClassifierTest, TooFarAway) {
  ros::Time timestamp;

  const double parking_start_line_angle_in_radian = 1.0472;
  const double parking_slot_width_in_m = 0.350;
  const double parking_slot_depth_in_m = 0.500;

  std::unique_ptr<ParkingEndClassifier> parking_end_classifier =
      std::make_unique<ParkingEndClassifier>(
          &param_, &camera_transformation_, &world_coordinates_helper_);

  std::vector<GeneratedOccupationState> generated_occupation_states = {
      GeneratedOccupationState::FREE, GeneratedOccupationState::X, GeneratedOccupationState::FREE};

  const Eigen::Vector3d test_translation(
      generated_occupation_states.size() * parking_slot_depth_in_m, 0.0, 0.0);

  const auto test_data = generateTestData(generated_occupation_states,
                                          parking_start_line_angle_in_radian,
                                          parking_slot_width_in_m,
                                          parking_slot_depth_in_m,
                                          0,
                                          test_translation);

  LineVehiclePoints lanes;
  lanes[LINESPEC_LEFT] = test_data.left_lane_points_;

  boost::optional<WorldPose> parking_lot_end;

  std::size_t nr_of_updates = 20;
  for (std::size_t i = 0; i < nr_of_updates; i++) {
    parking_lot_end =
        parking_end_classifier->detectEnd(test_data.parking_lot_->allParkingSpots(),
                                          test_data.img_,
                                          timestamp,
                                          test_data.left_lane_polynomial_,
                                          lanes,
                                          test_data.parking_lot_->mapPose());

    timestamp = ros::Time(i * 0.1);
  }

  EXPECT_EQ(boost::none, parking_lot_end);
}

}  // namespace testing
}  // namespace perpendicular_parking

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
