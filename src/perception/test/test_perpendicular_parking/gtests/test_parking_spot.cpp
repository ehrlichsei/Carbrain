#include <common/macros.h>
//#include "../src/perpendicular_parking/parking_lot.h"
#include "../../../src/utils/tf_helper_interface.h"
#include "../src/perpendicular_parking/parking_spot.h"
#include "common/eigen_utils.h"
#include "common/test/dummy_parameter_handler.h"
#include "parking_lot_generator.h"

THIRD_PARTY_HEADERS_BEGIN
#include <gtest/gtest.h>
#include <Eigen/Geometry>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>
THIRD_PARTY_HEADERS_END

DISABLE_SUGGEST_OVERRIDE_WARNING

namespace perpendicular_parking {
namespace testing {

using SpotPoint = Eigen::Vector3d;

class MyDummyParamHandler : public DummyParameterHandler {

 public:
  MyDummyParamHandler() {
    // Initialize parking lot parameters
    addParam(PARKING_SPOT_DEPTH, 0.5);
    addParam(PARKING_SPOT_WIDTH, 0.35);

    addParam(N_SCAN_LINES_PER_SPOT, 19);
    addParam(SPOT_SCAN_LINE_PADDING_X, 0.03);

    addParam(RANSAC_MIN_DIST, 0.01);
    addParam(RANSAC_MIN_CLUSTER_SIZE, 5);
    addParam(RANSAC_NR_LINES, 3);

    addParam(FIELD_OF_VISION_TOP, 90);
    addParam(FIELD_OF_VISION_BOTTOM, 550);
    addParam(FIELD_OF_VISION_LEFT, 0);
    addParam(FIELD_OF_VISION_RIGHT, 1289);

    // Initialize ParkinSpot Parameters
    addParam(POINTS_TO_SKIP_FRONT, 3);
    addParam(POINTS_TO_SKIP_BACK, -2);
    addParam(EPSILON_X_CLASSIFIER, 0.2);
    addParam(MIN_NUMBER_HITS, 4);
    addParam(SMALLEST_OBSTACLE_AREA, 0.02);
    addParam(FREE_MIN_DEPTH_TH, 0.3);

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

    addParam(
        ParameterString<double>("parking_spot/ransac/minimal_model_distance"), 0.01);
    addParam(
        ParameterString<int>("parking_spot/ransac/minimal_consensus_set_size"), 5);
    addParam(ParameterString<int>("parking_spot/ransac/expected_nr_of_lines"), 3);
  }

  // ParkingLot class parameters
  static const std::string NAMESPACE;

  static const ParameterString<double> PARKING_SPOT_DEPTH;
  static const ParameterString<double> PARKING_SPOT_WIDTH;

  static const ParameterString<int> N_SCAN_LINES_PER_SPOT;
  static const ParameterString<double> SPOT_SCAN_LINE_PADDING_X;

  static const ParameterString<double> RANSAC_MIN_DIST;
  static const ParameterString<int> RANSAC_MIN_CLUSTER_SIZE;
  static const ParameterString<int> RANSAC_NR_LINES;

  static const ParameterString<int> FIELD_OF_VISION_TOP;
  static const ParameterString<int> FIELD_OF_VISION_BOTTOM;
  static const ParameterString<int> FIELD_OF_VISION_LEFT;
  static const ParameterString<int> FIELD_OF_VISION_RIGHT;

  // ParkingSpot class parameters
  static const ParameterString<int> POINTS_TO_SKIP_FRONT;
  static const ParameterString<int> POINTS_TO_SKIP_BACK;
  static const ParameterString<double> EPSILON_X_CLASSIFIER;
  static const ParameterString<int> MIN_NUMBER_HITS;
  static const ParameterString<double> FREE_MIN_DEPTH_TH;
  static const ParameterString<double> SMALLEST_OBSTACLE_AREA;

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
};

// ParkingLot Parameters
const std::string MyDummyParamHandler::NAMESPACE(std::string("parking_lot"));

const ParameterString<double> MyDummyParamHandler::PARKING_SPOT_DEPTH(
    "parking_spot_depth");
const ParameterString<double> MyDummyParamHandler::PARKING_SPOT_WIDTH(
    "parking_spot_width");

const ParameterString<int> MyDummyParamHandler::N_SCAN_LINES_PER_SPOT(
    NAMESPACE + "/n_scan_lines_per_spot");

const ParameterString<double> MyDummyParamHandler::SPOT_SCAN_LINE_PADDING_X(
    "spot_scan_line_padding_x");

const ParameterString<double> MyDummyParamHandler::RANSAC_MIN_DIST(
    NAMESPACE + "/ransac/minimal_model_distance");
const ParameterString<int> MyDummyParamHandler::RANSAC_MIN_CLUSTER_SIZE(
    NAMESPACE + "/ransac/minimal_consensus_set_size");
const ParameterString<int> MyDummyParamHandler::RANSAC_NR_LINES(
    NAMESPACE + "/ransac/expected_nr_of_lines");

// ParkingSpot Class Parameter
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

const ParameterString<int> MyDummyParamHandler::FIELD_OF_VISION_TOP(
    "parking_lot/field_of_vision/top");
const ParameterString<int> MyDummyParamHandler::FIELD_OF_VISION_BOTTOM(
    "parking_lot/field_of_vision/bottom");
const ParameterString<int> MyDummyParamHandler::FIELD_OF_VISION_LEFT(
    "parking_lot/field_of_vision/left");
const ParameterString<int> MyDummyParamHandler::FIELD_OF_VISION_RIGHT(
    "parking_lot/field_of_vision/right");

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

class WorldCoordinatesTFHelperDummy : public tf_helper::TFHelperInterface<double> {
 public:
  virtual bool update(const ros::Time &) override { return true; }

  virtual AffineTransformation getTransform() const override {
    return Eigen::Affine3d::Identity();
  }
};

class ParkingSpotTest : public ::testing::Test {
 public:
  ParkingSpotTest()
      : cam_transform(&param_),
        n_scan_lines_per_spot(param_.getParam(MyDummyParamHandler::N_SCAN_LINES_PER_SPOT)),
        parking_spot_depth(param_.getParam(MyDummyParamHandler::PARKING_SPOT_DEPTH)),
        parking_spot_width(param_.getParam(MyDummyParamHandler::PARKING_SPOT_WIDTH)) {

    const int fov_top = param_.getParam(MyDummyParamHandler::FIELD_OF_VISION_TOP);
    const int fov_bottom = param_.getParam(MyDummyParamHandler::FIELD_OF_VISION_BOTTOM);
    const int fov_left = param_.getParam(MyDummyParamHandler::FIELD_OF_VISION_LEFT);
    const int fov_right = param_.getParam(MyDummyParamHandler::FIELD_OF_VISION_RIGHT);

    const common::EigenAlignedVector<ImagePoint> ground_in_image = {
        ImagePoint(fov_left, fov_top),
        ImagePoint(fov_right, fov_top),
        ImagePoint(fov_right, fov_bottom),
        ImagePoint(fov_left, fov_bottom)};

    common::EigenAlignedVector<VehiclePoint> fov_on_ground;
    cam_transform.transformImageToGround(ground_in_image, &fov_on_ground);
    field_of_vision_ = {{toCV(to2D(fov_on_ground[0])),
                         toCV(to2D(fov_on_ground[1])),
                         toCV(to2D(fov_on_ground[2])),
                         toCV(to2D(fov_on_ground[3]))}};

    // set ParkingSpot at 0.5 offset with pi/2 angle
    const Eigen::Affine3d world_to_spot_left_ =
        Eigen::Translation3d(0.5, 0.3, 0) *
        Eigen::AngleAxisd(M_PI_2, Eigen::Vector3d::UnitZ());
    const Eigen::Affine3d world_to_spot_right_ =
        Eigen::Translation3d(0.5 + parking_spot_width, 0.3, 0) *
        Eigen::AngleAxisd(M_PI_2, Eigen::Vector3d::UnitZ());

    left_entrance_ = std::make_shared<const MapPose>(world_to_spot_left_);
    right_entrance_ = std::make_shared<const MapPose>(world_to_spot_right_);
  }

  ParkingSpot makeParkingSpot() const {
    return ParkingSpot(0, left_entrance_, right_entrance_, &param_, &cam_transform, field_of_vision_);
  }

  void visualizeSpot(ObservationPoints obs_points) {
    cv::Mat img;
    img.create(720 * static_cast<int>(parking_spot_depth) + 10,
               720 * static_cast<int>(parking_spot_width) + 10,
               CV_8U);
    img = cv::Scalar::all(255);

    cv::Point2i top_left = {5, 5};
    cv::Point2i bottom_right = {
        static_cast<int>(std::round(720 * parking_spot_width)) + 5,
        static_cast<int>(std::round(720 * parking_spot_depth)) + 5};
    cv::rectangle(img, top_left, bottom_right, cv::Scalar(0, 0, 0), 3);

    WorldPoint point;
    for (const auto &obs_point : obs_points) {
      point = left_entrance_->inverse() * obs_point.second.point;
      cv::Point2i point_ = {
          static_cast<int>(std::round(720 * (-point[1]))) + 5,
          static_cast<int>(std::round(720 * (parking_spot_depth - point[0]))) + 5};
      cv::line(img, point_, point_, cv::Scalar(0, 0, 255), 5);
    }
    // save image
    bool img_saved = cv::imwrite("/tmp/debug.png", img);
    if (img_saved) {
      std::cout << "\n image: "
                << "/tmp/debug.png"
                << " saved. " << std::endl;
    }
  }

  ObservationPoints generateFreeParkingSpot() const {
    ObservationPoints new_obs_points;
    for (int i = 0; i < n_scan_lines_per_spot; i++) {
      const double y_ = static_cast<double>(i) / n_scan_lines_per_spot * parking_spot_width;
      const double x_ = parking_spot_depth;
      // transform into world coordinates
      ObservationPoint new_obs_point;
      new_obs_point.point = *left_entrance_ * VehiclePoint{x_, -y_, 0.0};
      new_obs_point.type = ObservationPoint::Type::VISIBILITY_LIMIT;
      new_obs_points.emplace(i, new_obs_point);
    }
    return new_obs_points;
  }

 private:
  MyDummyParamHandler param_;
  WorldCoordinatesTFHelperDummy tfhelper_dummy;
  const common::CameraTransformation cam_transform;

 protected:
  const int n_scan_lines_per_spot;
  const double parking_spot_depth;
  const double parking_spot_width;

  std::array<cv::Point2f, 4> field_of_vision_;

  const std::pair<double, double> angle_limit = {M_PI_2, -M_PI_2};

  MapPoseConstPtr left_entrance_;
  MapPoseConstPtr right_entrance_;
};

TEST_F(ParkingSpotTest, is_FREE) {
  const ObservationPoints new_obs_points = generateFreeParkingSpot();
  ParkingSpot parking_spot = makeParkingSpot();
  parking_spot.updateObservation(new_obs_points, angle_limit, MapPose::Identity());
  EXPECT_EQ(OccupationState::FREE, parking_spot.occupationState());
}

TEST_F(ParkingSpotTest, is_OCCUPIED) {
  const double obstacle_width = 0.2;
  const double obstacle_depth = 0.3;

  const int lines_per_obs = static_cast<int>(
      std::ceil(obstacle_width / parking_spot_width * n_scan_lines_per_spot));

  const auto isPartOfObstacle = [&](int i) {
    return i >= 0.5 * (n_scan_lines_per_spot - lines_per_obs) &&
           i <= 0.5 * (n_scan_lines_per_spot - lines_per_obs) + lines_per_obs;
  };

  ObservationPoints new_obs_points;
  for (int i = 0; i < n_scan_lines_per_spot; i++) {
    const bool on_obstacle = isPartOfObstacle(i);
    const double y_ = static_cast<double>(i) / n_scan_lines_per_spot * parking_spot_width;
    const double x_ = on_obstacle ? parking_spot_depth - obstacle_depth : parking_spot_depth;
    ObservationPoint new_obs_point;
    new_obs_point.point = *left_entrance_ * VehiclePoint{x_, -y_, 0.0};
    new_obs_point.type = on_obstacle ? ObservationPoint::Type::OBSTACLE
                                     : ObservationPoint::Type::VISIBILITY_LIMIT;

    new_obs_points.emplace(i, new_obs_point);
  }

  ParkingSpot parking_spot = makeParkingSpot();
  parking_spot.updateObservation(new_obs_points, angle_limit, MapPose::Identity());
  EXPECT_EQ(OccupationState::OCCUPIED, parking_spot.occupationState());
}

TEST_F(ParkingSpotTest, is_X) {
  const double slope = parking_spot_depth / parking_spot_width;
  ObservationPoints new_obs_points;
  for (int i = 0; i <= n_scan_lines_per_spot; i++) {
    const double y_ = static_cast<double>(i) / n_scan_lines_per_spot * parking_spot_width;
    const double x_ =
        (i <= n_scan_lines_per_spot / 2)
            ? slope * y_
            : parking_spot_depth / 2 - slope * (y_ - parking_spot_width / 2);
    ObservationPoint new_obs_point;
    new_obs_point.point = {x_, -y_, 0.0};
    new_obs_point.type = ObservationPoint::Type::OBSTACLE;
    new_obs_points.emplace(i, new_obs_point);
  }

  ParkingSpot parking_spot = makeParkingSpot();
  parking_spot.updateObservation(new_obs_points, angle_limit, MapPose::Identity());
  EXPECT_EQ(OccupationState::X, parking_spot.occupationState());
}

TEST_F(ParkingSpotTest, is_UNKNOWN_placement) {
  const ObservationPoints new_obs_points = generateFreeParkingSpot();
  ParkingSpot parking_spot = makeParkingSpot();
  parking_spot.updateObservation(
      new_obs_points, angle_limit, MapPose(Eigen::Translation3d(10.0, 0.0, 0.0)));
  EXPECT_EQ(OccupationState::UNKNOWN, parking_spot.occupationState());
}

TEST_F(ParkingSpotTest, is_UNKNOWN_empty) {
  ParkingSpot parking_spot = makeParkingSpot();
  parking_spot.updateObservation(ObservationPoints{}, angle_limit, MapPose::Identity());
  EXPECT_EQ(OccupationState::UNKNOWN, parking_spot.occupationState());
}
}  // namespace testing
}  // namespace perpendicular_parking

int main(int argc, char **argv) {
  // bool is_test_implemented = true;
  // ASSERT_TRUE(is_test_implemented);
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
