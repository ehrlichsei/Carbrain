#include <common/macros.h>

THIRD_PARTY_HEADERS_BEGIN
#include <gtest/gtest.h>
#include <tf2_eigen/tf2_eigen.h>
THIRD_PARTY_HEADERS_END

#include "navigation/gate.h"
#include "navigation_msgs/Gate.h"

DISABLE_SUGGEST_OVERRIDE_WARNING

// execute tests with one of the following command:
// catkin_make run_tests_navigation
// catkin_make run_tests_navigation_gtest_gate_test

namespace Eigen {

void PrintTo(const Eigen::Vector3d& vector, ::std::ostream* os) {
  *os << "\n" << vector;
}

void PrintTo(const Eigen::Matrix3d& matrix, ::std::ostream* os) {
  *os << "\n" << matrix;
}

void PrintTo(const Eigen::Affine3d::ConstTranslationPart& vector, ::std::ostream* os) {
  *os << "\n" << vector;
}

void PrintTo(const Eigen::Affine3d::TranslationPart& vector, ::std::ostream* os) {
  *os << "\n" << vector;
}

}  // namespace Eigen

namespace {

bool isApproxTranslation(const Eigen::Affine3d::ConstTranslationPart& expected,
                         const Eigen::Affine3d::TranslationPart& actual) {
  return expected.isApprox(actual);
}

bool isApproxRotation(const Eigen::Matrix3d& expected, const Eigen::Matrix3d& actual) {
  return expected.isApprox(actual);
}

bool isApproxVector(const Eigen::Vector3d& expected, const Eigen::Vector3d& actual) {
  return expected.isApprox(actual);
}

class GateTest : public ::testing::Test {
 public:
  static const Eigen::Vector3d LEFT_POLE;
  static const Eigen::Vector3d RIGHT_POLE;
  static const Eigen::Vector3d LEFT_POLE_GATE_FRAME;
  static const Eigen::Vector3d RIGHT_POLE_GATE_FRAME;
  static const Eigen::Vector3d CENTER;
  static const Eigen::Vector3d POINT_A;
  static const Eigen::Vector3d POINT_B;
  static const Eigen::Vector3d POINT_C;
  static const double ANGLE;

 protected:
  static Gate createGate() { return Gate(42, LEFT_POLE, RIGHT_POLE); }

  static Gate createDegeneratedGate() {
    return Gate(0, 0.5, 0.5, LEFT_POLE, RIGHT_POLE, 0.5);
  }

  static Eigen::Affine3d getGatePoseInWorldFrame() {
    const Eigen::Translation3d translation(LEFT_POLE);
    const Eigen::AngleAxisd rotation(ANGLE, Eigen::Vector3d::UnitZ());
    return translation * rotation;
  }

  static Eigen::Affine3d createTransformationFromGateFrame() {
    return getGatePoseInWorldFrame();
  }

  static Eigen::Affine3d createTransformationToGateFrame() {
    return createTransformationFromGateFrame().inverse();
  }
};

//
//     ^
//  8 -|                 A+
//     |
//  6 -|     left o
//     | C+       |
//  4 -|     Gate +      B+
//     |          |
//  2 -|    right o
//     |
//     |--+-------+-------+-->
//        |       |       |
//        1       5       9
//
const Eigen::Vector3d GateTest::LEFT_POLE(5, 6, 0);
const Eigen::Vector3d GateTest::RIGHT_POLE(5, 2, 0);
const Eigen::Vector3d GateTest::LEFT_POLE_GATE_FRAME(0, 0, 0);
const Eigen::Vector3d GateTest::RIGHT_POLE_GATE_FRAME(4, 0, 0);
const Eigen::Vector3d GateTest::CENTER(0.5 * (LEFT_POLE + RIGHT_POLE));
const Eigen::Vector3d GateTest::POINT_A(9, 8, 0);
const Eigen::Vector3d GateTest::POINT_B(9, 4, 0);
const Eigen::Vector3d GateTest::POINT_C(1, 5, 0);
const double GateTest::ANGLE(-0.5 * M_PI);

TEST_F(GateTest, getLeftPole) {
  const Gate gate = createGate();
  ASSERT_EQ(LEFT_POLE, gate.getLeftPole());
}

TEST_F(GateTest, getRightPole) {
  const Gate gate = createGate();
  ASSERT_EQ(RIGHT_POLE, gate.getRightPole());
}

TEST_F(GateTest, getCenter) {
  const Gate gate = createGate();
  ASSERT_EQ(CENTER, gate.getCenter());
}

TEST_F(GateTest, getWidth) {
  const Gate gate = createGate();
  ASSERT_EQ((LEFT_POLE - RIGHT_POLE).norm(), gate.getWidth());
}

TEST_F(GateTest, isDegeneratedNegative) {
  const Gate gate = createGate();
  ASSERT_FALSE(gate.isDegenerated());
}

TEST_F(GateTest, isDegeneratedPositive) {
  const Gate gate = createDegeneratedGate();
  ASSERT_TRUE(gate.isDegenerated());
}

TEST_F(GateTest, getTransformationToGateFrameTranslation) {
  const Gate gate = createGate();
  const Eigen::Affine3d expected = createTransformationToGateFrame();
  ASSERT_PRED2(isApproxTranslation,
               expected.translation(),
               gate.getTransformationToGateFrame().translation());
}

TEST_F(GateTest, getTransformationToGateFrameRotation) {
  const Gate gate = createGate();
  const Eigen::Affine3d expected = createTransformationToGateFrame();
  ASSERT_PRED2(isApproxRotation,
               expected.rotation(),
               gate.getTransformationToGateFrame().rotation());
}

TEST_F(GateTest, getTransformationToGateFrameLeftPole) {
  const Gate gate = createGate();
  const Eigen::Affine3d transformation = gate.getTransformationToGateFrame();
  ASSERT_PRED2(isApproxVector, LEFT_POLE_GATE_FRAME, transformation * gate.getLeftPole());
}

TEST_F(GateTest, getTransformationToGateFrameRightPole) {
  const Gate gate = createGate();
  const Eigen::Affine3d transformation = gate.getTransformationToGateFrame();
  ASSERT_PRED2(isApproxVector, RIGHT_POLE_GATE_FRAME, transformation * gate.getRightPole());
}

TEST_F(GateTest, getTransformationFromGateFrameTranslation) {
  const Gate gate = createGate();
  const Eigen::Affine3d expected = createTransformationFromGateFrame();
  ASSERT_PRED2(isApproxTranslation,
               expected.translation(),
               gate.getTransformationFromGateFrame().translation());
}

TEST_F(GateTest, getTransformationFromGateFrameRotation) {
  const Gate gate = createGate();
  const Eigen::Affine3d expected = createTransformationFromGateFrame();
  ASSERT_PRED2(isApproxRotation,
               expected.rotation(),
               gate.getTransformationFromGateFrame().rotation());
}

TEST_F(GateTest, getTransformationFromGateFrameLeftPole) {
  const Gate gate = createGate();
  const Eigen::Affine3d transformation = gate.getTransformationFromGateFrame();
  ASSERT_PRED2(isApproxVector, LEFT_POLE, transformation * LEFT_POLE_GATE_FRAME);
}

TEST_F(GateTest, getTransformationFromGateFrameRightPole) {
  const Gate gate = createGate();
  const Eigen::Affine3d transformation = gate.getTransformationFromGateFrame();
  ASSERT_PRED2(isApproxVector, RIGHT_POLE, transformation * RIGHT_POLE_GATE_FRAME);
}

TEST_F(GateTest, setLeftPole) {
  Gate gate = createGate();
  gate.setLeftPole(CENTER);
  ASSERT_EQ(CENTER, gate.getLeftPole());
  ASSERT_EQ(RIGHT_POLE, gate.getRightPole());
}

TEST_F(GateTest, setRightPole) {
  Gate gate = createGate();
  gate.setRightPole(CENTER);
  ASSERT_EQ(LEFT_POLE, gate.getLeftPole());
  ASSERT_EQ(CENTER, gate.getRightPole());
}

TEST_F(GateTest, collapseToCenter) {
  Gate gate = createGate();
  gate.collapseToCenter();
  ASSERT_EQ(CENTER, gate.getLeftPole());
  ASSERT_EQ(CENTER, gate.getRightPole());
}

TEST_F(GateTest, shrinkByWidth) {
  Gate gate = createGate();
  gate.shrinkBy(gate.getWidth());
  ASSERT_EQ(CENTER, gate.getLeftPole());
  ASSERT_EQ(CENTER, gate.getRightPole());
  ASSERT_EQ(0, gate.getWidth());
}

TEST_F(GateTest, shrinkFromLeftByWidth) {
  Gate gate = createGate();
  gate.shrinkFromLeftBy(gate.getWidth());
  ASSERT_EQ(RIGHT_POLE, gate.getLeftPole());
  ASSERT_EQ(RIGHT_POLE, gate.getRightPole());
  ASSERT_EQ(0, gate.getWidth());
}

TEST_F(GateTest, shrinkFromRightByWidth) {
  Gate gate = createGate();
  gate.shrinkFromRightBy(gate.getWidth());
  ASSERT_EQ(LEFT_POLE, gate.getLeftPole());
  ASSERT_EQ(LEFT_POLE, gate.getRightPole());
  ASSERT_EQ(0, gate.getWidth());
}

TEST_F(GateTest, shrinkByWidthDoubled) {
  Gate gate = createGate();
  gate.shrinkBy(2 * gate.getWidth());
  ASSERT_EQ(CENTER, gate.getLeftPole());
  ASSERT_EQ(CENTER, gate.getRightPole());
  ASSERT_EQ(0, gate.getWidth());
}

TEST_F(GateTest, shrinkFromLeftByWidthDoubled) {
  Gate gate = createGate();
  gate.shrinkFromLeftBy(2 * gate.getWidth());
  ASSERT_EQ(RIGHT_POLE, gate.getLeftPole());
  ASSERT_EQ(RIGHT_POLE, gate.getRightPole());
  ASSERT_EQ(0, gate.getWidth());
}

TEST_F(GateTest, shrinkFromRightByWidthDoubled) {
  Gate gate = createGate();
  gate.shrinkFromRightBy(2 * gate.getWidth());
  ASSERT_EQ(LEFT_POLE, gate.getLeftPole());
  ASSERT_EQ(LEFT_POLE, gate.getRightPole());
  ASSERT_EQ(0, gate.getWidth());
}

TEST_F(GateTest, shrinkByHalfWidth) {
  Gate gate = createGate();
  gate.shrinkBy(0.5 * gate.getWidth());
  ASSERT_EQ(0.5 * (LEFT_POLE + CENTER), gate.getLeftPole());
  ASSERT_EQ(0.5 * (RIGHT_POLE + CENTER), gate.getRightPole());
  ASSERT_EQ(0.5 * (LEFT_POLE - RIGHT_POLE).norm(), gate.getWidth());
}

TEST_F(GateTest, shrinkFromLeftByHalfWidth) {
  Gate gate = createGate();
  gate.shrinkFromLeftBy(0.5 * gate.getWidth());
  ASSERT_EQ(CENTER, gate.getLeftPole());
  ASSERT_EQ(RIGHT_POLE, gate.getRightPole());
  ASSERT_EQ(0.5 * (LEFT_POLE - RIGHT_POLE).norm(), gate.getWidth());
}

TEST_F(GateTest, shrinkFromRightByHalfWidth) {
  Gate gate = createGate();
  gate.shrinkFromRightBy(0.5 * gate.getWidth());
  ASSERT_EQ(LEFT_POLE, gate.getLeftPole());
  ASSERT_EQ(CENTER, gate.getRightPole());
  ASSERT_EQ(0.5 * (LEFT_POLE - RIGHT_POLE).norm(), gate.getWidth());
}

TEST_F(GateTest, shrinkDegeneratedGate) {
  Gate gate = createDegeneratedGate();
  gate.shrinkBy(0);
  ASSERT_EQ(CENTER, gate.getLeftPole());
  ASSERT_EQ(CENTER, gate.getRightPole());
  ASSERT_EQ(0, gate.getWidth());
}

TEST_F(GateTest, shrinkDegeneratedGateFromLeft) {
  Gate gate = createDegeneratedGate();
  gate.shrinkFromLeftBy(0);
  ASSERT_EQ(CENTER, gate.getLeftPole());
  ASSERT_EQ(CENTER, gate.getRightPole());
  ASSERT_EQ(0, gate.getWidth());
}

TEST_F(GateTest, shrinkDegeneratedGateFromRight) {
  Gate gate = createDegeneratedGate();
  gate.shrinkFromRightBy(0);
  ASSERT_EQ(CENTER, gate.getLeftPole());
  ASSERT_EQ(CENTER, gate.getRightPole());
  ASSERT_EQ(0, gate.getWidth());
}

TEST_F(GateTest, transform) {
  Gate gate = createGate();
  const Eigen::Affine3d transformation = gate.getTransformationToGateFrame();
  gate.transform(transformation);
  ASSERT_PRED2(isApproxVector, LEFT_POLE_GATE_FRAME, gate.getLeftPole());
  ASSERT_PRED2(isApproxVector, RIGHT_POLE_GATE_FRAME, gate.getRightPole());
}

TEST_F(GateTest, fromMessage) {
  navigation_msgs::Gate gate_msg;
  gate_msg.id = 42;
  gate_msg.left_lane_boundary = tf2::toMsg(LEFT_POLE);
  gate_msg.right_lane_boundary = tf2::toMsg(RIGHT_POLE);
  gate_msg.lane_center_point = 0.5;
  gate_msg.left_pole = 0.0;
  gate_msg.right_pole = 1.0;
  const Gate gate = Gate::fromMessage(gate_msg);
  const Gate expected = createGate();
  ASSERT_EQ(expected.getId(), gate.getId());
  ASSERT_EQ(expected.getLeftLaneBoundary(), gate.getLeftLaneBoundary());
  ASSERT_EQ(expected.getRightLaneBoundary(), gate.getRightLaneBoundary());
  ASSERT_EQ(expected.getLaneCenter(), gate.getLaneCenter());
  ASSERT_EQ(expected.getLeftPole(), gate.getLeftPole());
  ASSERT_EQ(expected.getRightPole(), gate.getRightPole());
  ASSERT_EQ(expected.getCenter(), gate.getCenter());
}

TEST_F(GateTest, toMessage) {
  navigation_msgs::Gate original_gate_msg;
  original_gate_msg.id = 42;
  original_gate_msg.left_lane_boundary = tf2::toMsg(Eigen::Vector3d(1, 2, 3));
  original_gate_msg.right_lane_boundary = tf2::toMsg(Eigen::Vector3d(4, 5, 6));
  original_gate_msg.lane_center_point = 0.5;
  original_gate_msg.left_pole = 0.0;
  original_gate_msg.right_pole = 1.0;
  const Gate gate = createGate();
  navigation_msgs::Gate gate_msg = gate.toMessage();
  ASSERT_EQ(tf2::toMsg(LEFT_POLE).x, gate_msg.left_lane_boundary.x);
  ASSERT_EQ(tf2::toMsg(LEFT_POLE).y, gate_msg.left_lane_boundary.y);
  ASSERT_EQ(tf2::toMsg(RIGHT_POLE).x, gate_msg.right_lane_boundary.x);
  ASSERT_EQ(tf2::toMsg(RIGHT_POLE).y, gate_msg.right_lane_boundary.y);
  ASSERT_EQ(gate_msg.id, original_gate_msg.id);
  ASSERT_EQ(gate_msg.lane_center_point, original_gate_msg.lane_center_point);
  ASSERT_EQ(gate_msg.left_pole, original_gate_msg.left_pole);
  ASSERT_EQ(gate_msg.right_pole, original_gate_msg.right_pole);
}

TEST_F(GateTest, lessThanPositive) {
  const Gate gate = createGate();
  const Gate second_gate = Gate(0, POINT_A, POINT_B);
  ASSERT_TRUE(gate < second_gate);
}

TEST_F(GateTest, lessThanNegative) {
  const Gate gate = createGate();
  const Gate second_gate = Gate(0, POINT_A, POINT_B);
  ASSERT_FALSE(second_gate < gate);
}

TEST_F(GateTest, largerPositive) {
  const Gate gate = createGate();
  const Gate second_gate = Gate(0, POINT_A, POINT_B);
  ASSERT_TRUE(second_gate > gate);
}

TEST_F(GateTest, largerNegative) {
  const Gate gate = createGate();
  const Gate second_gate = Gate(0, POINT_A, POINT_B);
  ASSERT_FALSE(gate > second_gate);
}

TEST_F(GateTest, lessThanIdentical) {
  const Gate gate = createGate();
  ASSERT_FALSE(gate < gate);
}

TEST_F(GateTest, largerIdentical) {
  const Gate gate = createGate();
  ASSERT_FALSE(gate > gate);
}

TEST_F(GateTest, lessThanNoOrder) {
  const Gate gate = createGate();
  const Gate second_gate = Gate(0, POINT_C, POINT_B);
  ASSERT_FALSE(gate < second_gate);
}

TEST_F(GateTest, largerNoOrder) {
  const Gate gate = createGate();
  const Gate second_gate = Gate(0, POINT_C, POINT_B);
  ASSERT_FALSE(gate > second_gate);
}

TEST_F(GateTest, lessThanContradictionSecondGate) {
  const Gate gate = createGate();
  const Gate second_gate = Gate(0, POINT_B, POINT_A);
  ASSERT_FALSE(gate < second_gate);
  ASSERT_FALSE(second_gate < gate);
}

TEST_F(GateTest, greaterThanContradictionSecondGate) {
  const Gate gate = createGate();
  const Gate second_gate = Gate(0, POINT_B, POINT_A);
  ASSERT_FALSE(gate > second_gate);
  ASSERT_FALSE(second_gate > gate);
}

TEST_F(GateTest, lessThanContradictionFirstGate) {
  const Gate gate_inversed = Gate(0, RIGHT_POLE, LEFT_POLE);
  const Gate second_gate = Gate(0, POINT_A, POINT_B);
  ASSERT_FALSE(gate_inversed < second_gate);
  ASSERT_FALSE(second_gate < gate_inversed);
}

TEST_F(GateTest, largerContradictionFirstGate) {
  const Gate gate_inversed = Gate(0, RIGHT_POLE, LEFT_POLE);
  const Gate second_gate = Gate(0, POINT_A, POINT_B);
  ASSERT_FALSE(gate_inversed > second_gate);
  ASSERT_FALSE(second_gate > gate_inversed);
}

TEST_F(GateTest, poleOutsideLeftLaneBoundary) {
  const Eigen::Vector3d outside_left_pole(5, 7, 0);
  Gate gate = createGate();
  gate.setLeftPole(outside_left_pole);
  ASSERT_EQ(LEFT_POLE, gate.getLeftPole());
  ASSERT_EQ(RIGHT_POLE, gate.getRightPole());
}

TEST_F(GateTest, poleOutsideRightLaneBoundary) {
  const Eigen::Vector3d outside_right_pole(5, 1, 0);
  Gate gate = createGate();
  gate.setRightPole(outside_right_pole);
  ASSERT_EQ(LEFT_POLE, gate.getLeftPole());
  ASSERT_EQ(RIGHT_POLE, gate.getRightPole());
}

TEST_F(GateTest, projectPointOutsideLineToGate) {
  Gate gate = createGate();
  gate.setLeftPole(POINT_B);
  ASSERT_EQ(gate.getLaneCenter(), gate.getLeftPole());
  ASSERT_EQ(RIGHT_POLE, gate.getRightPole());
}

TEST_F(GateTest, interchangedLeftRightPole) {
  Gate gate = createGate();
  gate.setLeftPole(RIGHT_POLE);
  ASSERT_EQ(RIGHT_POLE, gate.getLeftPole());
  ASSERT_EQ(RIGHT_POLE, gate.getRightPole());
  gate.setRightPole(LEFT_POLE);
  ASSERT_EQ(RIGHT_POLE, gate.getLeftPole());
  ASSERT_EQ(RIGHT_POLE, gate.getRightPole());
}

}  // namespace

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
