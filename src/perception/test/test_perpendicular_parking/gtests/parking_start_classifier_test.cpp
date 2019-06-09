#include "../../../src/perpendicular_parking/parking_start_classifier.h"
#include <common/macros.h>
#include "common/camera_transformation.h"
#include "common/polynomialfit.h"
#include "common/test/dummy_parameter_handler.h"
#include "parking_lot_generator.h"

THIRD_PARTY_HEADERS_BEGIN
#include <gtest/gtest.h>
THIRD_PARTY_HEADERS_END

DISABLE_SUGGEST_OVERRIDE_WARNING

namespace perpendicular_parking {
namespace testing {

template <typename Scalar>
class TFHelperMock : public tf_helper::TFHelperInterface<Scalar> {
 public:
  bool update(const ros::Time &) override { return true; }

  typename tf_helper::TFHelperInterface<Scalar>::AffineTransformation getTransform() const override {
    return tf_helper::TFHelperInterface<Scalar>::AffineTransformation::Identity();
  }
};

class ParkingStartClassifierTest : public ::testing::Test {
  // helper methods etc go here
 public:
  ParkingStartClassifierTest() {
    // camera parameters
    params_.addParam(ParameterString<double>("/camera/focal_length_x"), 386.704446);
    params_.addParam(ParameterString<double>("/camera/focal_length_y"), 385.973724);
    params_.addParam(ParameterString<double>("/camera/optical_center_x"), 623.233113);
    params_.addParam(ParameterString<double>("/camera/optical_center_y"), 125.752428);

    params_.addParam(ParameterString<double>("/camera/r11"), 0.0214821);
    params_.addParam(ParameterString<double>("/camera/r12"), -0.999761);
    params_.addParam(ParameterString<double>("/camera/r13"), -0.00402491);
    params_.addParam(ParameterString<double>("/camera/r21"), -0.149211);
    params_.addParam(ParameterString<double>("/camera/r22"), 0.00077467);
    params_.addParam(ParameterString<double>("/camera/r23"), -0.988805);
    params_.addParam(ParameterString<double>("/camera/r31"), 0.988572);
    params_.addParam(ParameterString<double>("/camera/r32"), 0.0218421);
    params_.addParam(ParameterString<double>("/camera/r33"), -0.149159);

    params_.addParam(ParameterString<double>("/camera/t1"), 7.42816);
    params_.addParam(ParameterString<double>("/camera/t2"), 262.261);
    params_.addParam(ParameterString<double>("/camera/t3"), 32.6001);


    // db_scan
    params_.addParam(ParameterString<double>("dbscan/core_epsilon"), 0.1);
    params_.addParam(ParameterString<int>("dbscan/min_points_to_be_core"), 4);

    // parking_start_classifier
    params_.addParam(
        ParameterString<double>("parking_start_classifier/horizon"), 2.2);
    params_.addParam(ParameterString<double>(
                         "parking_start_classifier/max_distance_end_of_points"),
                     0.15);
    params_.addParam(
        ParameterString<double>("parking_start_classifier/max_angle_delta"), 0.175);
    params_.addParam(
        ParameterString<double>("parking_start_classifier/second_pca_thld"), 0.04);
    params_.addParam(ParameterString<double>(
                         "parking_start_classifier/max_dist_to_be_close"),
                     0.15);
    params_.addParam(
        ParameterString<int>("parking_start_classifier/min_nr_close_points"), 2);
    params_.addParam(
        ParameterString<double>("parking_start_classifier/required_certainty"), 0.9);
    params_.addParam(
        ParameterString<double>("parking_line/max_time_with_no_observation"), 0.1);
    params_.addParam(ParameterString<double>("parking_line/init_covariance"), 1000.0);
    params_.addParam(ParameterString<double>("parking_line/max_position_delta"), 0.1);
    params_.addParam(ParameterString<double>("parking_line/max_angle_delta"), 0.1);
    params_.addParam(
        ParameterString<double>(
            "parking_start_classifier/ransac/minimal_model_distance"),
        0.02);
    params_.addParam(
        ParameterString<int>(
            "parking_start_classifier/ransac/minimal_consensus_set_size"),
        5);
    params_.addParam(
        ParameterString<int>(
            "parking_start_classifier/ransac/expected_nr_of_lines"),
        4);
    params_.addParam(ParameterString<double>(
                         "parking_start_classifier/minimum_startline_angle"),
                     0.78);
    params_.addParam(ParameterString<double>(
                         "parking_start_classifier/maximum_startline_angle"),
                     1.05);
  }

 protected:
  TFHelperMock<double> coordinates_helper_;
  DummyParameterHandler params_;
};

const FeaturePointCluster toStartLineCluster(const Eigen::Affine3d &pose,
                                             const common::CameraTransformation &cam_trafo,
                                             const double angle = 1.0472,
                                             const double depth = 0.5,
                                             const double step = 0.02) {
  const auto length = depth / std::fabs(std::sin(angle));
  const auto nr_points = static_cast<std::size_t>(length / step);

  VehiclePoints start_line_points;
  start_line_points.reserve(nr_points);
  for (std::size_t i = 0; i < nr_points; i++) {
    start_line_points.emplace_back(
        pose.translation() + step * i * (pose.linear() * VehiclePoint::UnitX()).normalized());
  }

  ImagePoints feature_points_img;
  cam_trafo.transformGroundToImage(start_line_points, &feature_points_img);

  return FeaturePointCluster{feature_points_img, start_line_points};
}

TEST_F(ParkingStartClassifierTest, DefaultStartLine) {
  const std::vector<GeneratedOccupationState> dummy_occ_states_ = {
      {GeneratedOccupationState::OBSTACLE_MEDIUM, GeneratedOccupationState::OBSTACLE_LARGE}};
  // create ParkingLot with default startline
  const double angle = 1.0472;
  const double width = 0.35;
  const double depth = 0.5;
  ParkingLotGenerator scene_generator_(dummy_occ_states_, angle, width, depth);

  // construct points
  const Eigen::Affine3d start_line_pose = scene_generator_.getParkingStartPose();
  const common::CameraTransformation cam_transform(&params_);
  // default startline
  auto start_line_cluster = toStartLineCluster(start_line_pose, cam_transform);
  // get left lane points
  VehiclePoints left_lane_points;
  left_lane_points.reserve(100);
  scene_generator_.getLeftLanePoints(left_lane_points, 100);

  // declare object
  ParkingStartClassifier classifier(&params_, &cam_transform, &coordinates_helper_);

  auto type = Type::UNKNOWN;
  std::size_t nr_of_updates = 20;
  const common::DynamicPolynomial left_lane_polynom =
      common::fitToPoints(left_lane_points, 1);
  for (std::size_t i = 0; i < nr_of_updates; i++) {
    type = classifier.classify(start_line_cluster, left_lane_polynom, ros::Time(i * 0.1));
  }

  EXPECT_EQ(type, Type::START);

  const auto eps = 0.01;

  const auto pose_out = classifier.startLinePose();

  EXPECT_LE((pose_out.translation() - start_line_pose.translation()).norm(), eps);
  EXPECT_LE((pose_out.linear() - start_line_pose.linear()).array().abs().sum(), eps);
}

TEST_F(ParkingStartClassifierTest, NoStartLineLarge) {
  const std::vector<GeneratedOccupationState> dummy_occ_states_ = {
      {GeneratedOccupationState::OBSTACLE_MEDIUM, GeneratedOccupationState::OBSTACLE_LARGE}};
  // create ParkingLot with default startline
  const double angle = 1.15;
  const double width = 0.35;
  const double depth = 0.5;
  ParkingLotGenerator scene_generator_(dummy_occ_states_, angle, width, depth);

  // construct points
  const Eigen::Affine3d start_line_pose = scene_generator_.getParkingStartPose();
  const common::CameraTransformation cam_transform(&params_);
  // default startline
  auto start_line_cluster = toStartLineCluster(start_line_pose, cam_transform);
  // get left lane points
  VehiclePoints left_lane_points;
  left_lane_points.reserve(100);
  scene_generator_.getLeftLanePoints(left_lane_points, 100);

  // declare object
  ParkingStartClassifier classifier(&params_, &cam_transform, &coordinates_helper_);

  auto type = Type::START;
  std::size_t nr_of_updates = 20;
  const common::DynamicPolynomial left_lane_polynom =
      common::fitToPoints(left_lane_points, 1);
  for (std::size_t i = 0; i < nr_of_updates; i++) {
    type = classifier.classify(start_line_cluster, left_lane_polynom, ros::Time(i * 0.1));
  }

  EXPECT_EQ(type, Type::UNKNOWN);
}

TEST_F(ParkingStartClassifierTest, NoStartLineSmall) {
  const std::vector<GeneratedOccupationState> dummy_occ_states_ = {
      {GeneratedOccupationState::OBSTACLE_MEDIUM, GeneratedOccupationState::OBSTACLE_LARGE}};
  // create ParkingLot with default startline
  const double angle = 0.75;
  const double width = 0.35;
  const double depth = 0.5;
  ParkingLotGenerator scene_generator_(dummy_occ_states_, angle, width, depth);

  // construct points
  const Eigen::Affine3d start_line_pose = scene_generator_.getParkingStartPose();
  const common::CameraTransformation cam_transform(&params_);
  // default startline
  auto start_line_cluster = toStartLineCluster(start_line_pose, cam_transform);
  // get left lane points
  VehiclePoints left_lane_points;
  left_lane_points.reserve(100);
  scene_generator_.getLeftLanePoints(left_lane_points, 100);

  // declare object
  ParkingStartClassifier classifier(&params_, &cam_transform, &coordinates_helper_);

  auto type = Type::START;
  std::size_t nr_of_updates = 20;
  const common::DynamicPolynomial left_lane_polynom =
      common::fitToPoints(left_lane_points, 1);
  for (std::size_t i = 0; i < nr_of_updates; i++) {
    type = classifier.classify(start_line_cluster, left_lane_polynom, ros::Time(i * 0.1));
  }

  EXPECT_EQ(type, Type::UNKNOWN);
}

TEST_F(ParkingStartClassifierTest, LargeAngleStartLine) {
  const std::vector<GeneratedOccupationState> dummy_occ_states_ = {
      {GeneratedOccupationState::OBSTACLE_MEDIUM, GeneratedOccupationState::OBSTACLE_LARGE}};
  // create ParkingLot with default startline
  const double angle = M_PI / 3.0;
  const double width = 0.35;
  const double depth = 0.5;
  ParkingLotGenerator scene_generator_(dummy_occ_states_, angle, width, depth);

  // construct points
  const Eigen::Affine3d start_line_pose = scene_generator_.getParkingStartPose();
  const common::CameraTransformation cam_transform(&params_);
  // default startline
  auto start_line_cluster = toStartLineCluster(start_line_pose, cam_transform);
  // get left lane points
  VehiclePoints left_lane_points;
  left_lane_points.reserve(100);
  scene_generator_.getLeftLanePoints(left_lane_points, 100);

  // declare object
  ParkingStartClassifier classifier(&params_, &cam_transform, &coordinates_helper_);

  auto type = Type::UNKNOWN;
  std::size_t nr_of_updates = 20;
  const common::DynamicPolynomial left_lane_polynom =
      common::fitToPoints(left_lane_points, 1);
  for (std::size_t i = 0; i < nr_of_updates; i++) {
    type = classifier.classify(start_line_cluster, left_lane_polynom, ros::Time(i * 0.1));
  }

  EXPECT_EQ(type, Type::START);

  const auto eps = 0.01;

  const auto pose_out = classifier.startLinePose();

  EXPECT_LE((pose_out.translation() - start_line_pose.translation()).norm(), eps);
  EXPECT_LE((pose_out.linear() - start_line_pose.linear()).array().abs().sum(), eps);
}

TEST_F(ParkingStartClassifierTest, SmallAngleStartLine) {
  const std::vector<GeneratedOccupationState> dummy_occ_states_ = {
      {GeneratedOccupationState::OBSTACLE_MEDIUM, GeneratedOccupationState::OBSTACLE_LARGE}};
  // create ParkingLot with default startline
  const double angle = M_PI_4;
  const double width = 0.35;
  const double depth = 0.5;
  ParkingLotGenerator scene_generator_(dummy_occ_states_, angle, width, depth);

  // construct points
  const Eigen::Affine3d start_line_pose = scene_generator_.getParkingStartPose();
  const common::CameraTransformation cam_transform(&params_);
  // default startline
  auto start_line_cluster = toStartLineCluster(start_line_pose, cam_transform);
  // get left lane points
  VehiclePoints left_lane_points;
  left_lane_points.reserve(100);
  scene_generator_.getLeftLanePoints(left_lane_points, 100);

  // declare object
  ParkingStartClassifier classifier(&params_, &cam_transform, &coordinates_helper_);

  auto type = Type::UNKNOWN;
  std::size_t nr_of_updates = 20;
  const common::DynamicPolynomial left_lane_polynom =
      common::fitToPoints(left_lane_points, 1);
  for (std::size_t i = 0; i < nr_of_updates; i++) {
    type = classifier.classify(start_line_cluster, left_lane_polynom, ros::Time(i * 0.1));
  }

  EXPECT_EQ(type, Type::START);

  const auto eps = 0.01;

  const auto pose_out = classifier.startLinePose();

  EXPECT_LE((pose_out.translation() - start_line_pose.translation()).norm(), eps);
  EXPECT_LE((pose_out.linear() - start_line_pose.linear()).array().abs().sum(), eps);
}

TEST_F(ParkingStartClassifierTest, TooFarAway) {
  const std::vector<GeneratedOccupationState> dummy_occ_states_ = {
      {GeneratedOccupationState::OBSTACLE_MEDIUM, GeneratedOccupationState::OBSTACLE_LARGE}};
  // create ParkingLot with default startline
  const double angle = M_PI_4;
  const double width = 0.35;
  const double depth = 0.5;
  ParkingLotGenerator scene_generator_(dummy_occ_states_, angle, width, depth);

  // define pose and translate so that it's farer away than horizon parameter
  Eigen::Affine3d start_line_pose = scene_generator_.getParkingStartPose();
  start_line_pose.translate(Eigen::Vector3d{3.0, 0.0, 0.0});

  const common::CameraTransformation cam_transform(&params_);
  // default startline
  auto start_line_cluster = toStartLineCluster(start_line_pose, cam_transform);
  // get left lane points
  VehiclePoints left_lane_points;
  left_lane_points.reserve(100);
  scene_generator_.getLeftLanePoints(left_lane_points, 100);

  // declare object
  ParkingStartClassifier classifier(&params_, &cam_transform, &coordinates_helper_);

  auto type = Type::START;
  std::size_t nr_of_updates = 10;
  const common::DynamicPolynomial left_lane_polynom =
      common::fitToPoints(left_lane_points, 1);
  for (std::size_t i = 0; i < nr_of_updates; i++) {
    type = classifier.classify(start_line_cluster, left_lane_polynom, ros::Time(i * 0.1));
  }

  EXPECT_EQ(type, Type::UNKNOWN);
}

}  // namespace testing
}  // namespace perpendicular_parking

// nothing to do here
int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
