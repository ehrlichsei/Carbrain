#include <common/macros.h>

THIRD_PARTY_HEADERS_BEGIN
#include <gtest/gtest.h>
#include <boost/make_shared.hpp>
#include <tf2_eigen/tf2_eigen.h>
#include "navigation_msgs/DrivingCorridor.h"
THIRD_PARTY_HEADERS_END

#include "navigation/driving_corridor.h"

DISABLE_SUGGEST_OVERRIDE_WARNING

// execute tests with one of the following command:
// catkin_make run_tests_navigation
// catkin_make run_tests_navigation_gtest_driving_corridor_test

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

class DrivingCorridorTest : public ::testing::Test {
 public:
  static const Eigen::Vector3d FIRST_GATE_LEFT_POLE;
  static const Eigen::Vector3d FIRST_GATE_RIGHT_POLE;
  static const Eigen::Vector3d SECOND_GATE_LEFT_POLE;
  static const Eigen::Vector3d SECOND_GATE_RIGHT_POLE;
  static const Eigen::Vector3d THIRD_GATE_LEFT_POLE;
  static const Eigen::Vector3d THIRD_GATE_RIGHT_POLE;
  static const Eigen::Vector3d FOURTH_GATE_LEFT_POLE;
  static const Eigen::Vector3d FOURTH_GATE_RIGHT_POLE;
  static const Eigen::Vector3d POINT_A;
  static const Eigen::Vector3d POINT_B;
  static const Eigen::Vector3d POINT_C;
  static const Eigen::Vector3d POINT_D;
  static const Eigen::Vector3d POINT_E;
  static const Eigen::Vector3d POINT_F;
  static const Eigen::Vector3d POINT_G;

 protected:
  static DrivingCorridor createDrivingCorridor() {
    Gate::GateList gates;
    gates.push_back(createFirstGate());
    return DrivingCorridor(gates);
  }

  static DrivingCorridor createDrivingCorridorTwoCloseGates() {
    Gate::GateList gates;
    gates.push_back(createFirstGate());
    gates.push_back(createSecondGate());
    return DrivingCorridor(gates);
  }

  static DrivingCorridor createDrivingCorridorTwoFarGates() {
    Gate::GateList gates;
    gates.push_back(createFirstGate());
    gates.push_back(createThirdGate());
    return DrivingCorridor(gates);
  }

  static DrivingCorridor createDrivingCorridorThreeGates() {
    Gate::GateList gates;
    gates.push_back(createFirstGate());
    gates.push_back(createSecondGate());
    gates.push_back(createThirdGate());
    return DrivingCorridor(gates);
  }

  static DrivingCorridor createDrivingCorridorFourGates() {
    Gate::GateList gates;
    gates.push_back(createFirstGate());
    gates.push_back(createSecondGate());
    gates.push_back(createThirdGate());
    gates.push_back(createFourthGate());
    return DrivingCorridor(gates);
  }

  static Gate createFirstGate() {
    return Gate(0, FIRST_GATE_LEFT_POLE, FIRST_GATE_RIGHT_POLE);
  }

  static Gate createSecondGate() {
    return Gate(0, SECOND_GATE_LEFT_POLE, SECOND_GATE_RIGHT_POLE);
  }

  static Gate createThirdGate() {
    return Gate(0, THIRD_GATE_LEFT_POLE, THIRD_GATE_RIGHT_POLE);
  }

  static Gate createFourthGate() {
    return Gate(0, FOURTH_GATE_LEFT_POLE, FOURTH_GATE_RIGHT_POLE);
  }
};

//
//     ^
//  8 -|     A+      B+
//     |
//  6 -|  o left  o left  o       o
//     |  |       |    G+ |      /
//  4 -| C+  D+   +       +  E+ /
//     |  |       |       |    /+F
//  2 -|  o right o right o   o
//     |
//     |--+---+---+---+---+---+---+-->
//        |   '   |   '   |   '   |
//        1   3   5   7   9   11  13
//
const Eigen::Vector3d DrivingCorridorTest::FIRST_GATE_LEFT_POLE(1, 6, 0);
const Eigen::Vector3d DrivingCorridorTest::FIRST_GATE_RIGHT_POLE(1, 2, 0);
const Eigen::Vector3d DrivingCorridorTest::SECOND_GATE_LEFT_POLE(5, 6, 0);
const Eigen::Vector3d DrivingCorridorTest::SECOND_GATE_RIGHT_POLE(5, 2, 0);
const Eigen::Vector3d DrivingCorridorTest::THIRD_GATE_LEFT_POLE(9, 6, 0);
const Eigen::Vector3d DrivingCorridorTest::THIRD_GATE_RIGHT_POLE(9, 2, 0);
const Eigen::Vector3d DrivingCorridorTest::FOURTH_GATE_LEFT_POLE(13, 6, 0);
const Eigen::Vector3d DrivingCorridorTest::FOURTH_GATE_RIGHT_POLE(11, 2, 0);
const Eigen::Vector3d DrivingCorridorTest::POINT_A(3, 8, 0);
const Eigen::Vector3d DrivingCorridorTest::POINT_B(7, 8, 0);
const Eigen::Vector3d DrivingCorridorTest::POINT_C(1, 4, 0);
const Eigen::Vector3d DrivingCorridorTest::POINT_D(3, 4, 0);
const Eigen::Vector3d DrivingCorridorTest::POINT_E(11, 4, 0);
const Eigen::Vector3d DrivingCorridorTest::POINT_F(12, 3, 0);
const Eigen::Vector3d DrivingCorridorTest::POINT_G(8, 5, 0);

TEST_F(DrivingCorridorTest, fromMessage) {
  const navigation_msgs::DrivingCorridor::Ptr driving_corridor_msg =
      boost::make_shared<navigation_msgs::DrivingCorridor>();
  driving_corridor_msg->gates.push_back(createFirstGate().toMessage());
  const DrivingCorridor driving_corridor =
      DrivingCorridor::fromMessage(driving_corridor_msg);

  ASSERT_EQ(1, driving_corridor.size());
  ASSERT_EQ(FIRST_GATE_LEFT_POLE, driving_corridor.at(0).getLeftPole());
  ASSERT_EQ(FIRST_GATE_RIGHT_POLE, driving_corridor.at(0).getRightPole());
}

TEST_F(DrivingCorridorTest, toMessage) {
  navigation_msgs::DrivingCorridor original_driving_corridor_msg;
  original_driving_corridor_msg.header.frame_id = "world";

  const DrivingCorridor driving_corridor = createDrivingCorridor();
  const navigation_msgs::DrivingCorridor driving_corridor_msg =
      driving_corridor.toMessage();

  ASSERT_EQ(1, driving_corridor_msg.gates.size());
  ASSERT_EQ(tf2::toMsg(FIRST_GATE_LEFT_POLE).x,
            driving_corridor_msg.gates.at(0).left_lane_boundary.x);
  ASSERT_EQ(tf2::toMsg(FIRST_GATE_LEFT_POLE).y,
            driving_corridor_msg.gates.at(0).left_lane_boundary.y);
  ASSERT_EQ(tf2::toMsg(FIRST_GATE_RIGHT_POLE).x,
            driving_corridor_msg.gates.at(0).right_lane_boundary.x);
  ASSERT_EQ(tf2::toMsg(FIRST_GATE_RIGHT_POLE).y,
            driving_corridor_msg.gates.at(0).right_lane_boundary.y);
}

TEST_F(DrivingCorridorTest, simplifiedOneGate) {
  const DrivingCorridor driving_corridor = createDrivingCorridor();
  const DrivingCorridor simplified_corridor = driving_corridor.simplified(10);
  ASSERT_EQ(1, simplified_corridor.size());
  ASSERT_EQ(FIRST_GATE_LEFT_POLE, simplified_corridor.at(0).getLeftPole());
  ASSERT_EQ(FIRST_GATE_RIGHT_POLE, simplified_corridor.at(0).getRightPole());
}

TEST_F(DrivingCorridorTest, simplifiedTwoCloseGatesSmallDistanceParameter) {
  const DrivingCorridor driving_corridor = createDrivingCorridorTwoCloseGates();
  const DrivingCorridor simplified_corridor = driving_corridor.simplified(1);
  ASSERT_EQ(2, simplified_corridor.size());
  ASSERT_EQ(FIRST_GATE_LEFT_POLE, simplified_corridor.at(0).getLeftPole());
  ASSERT_EQ(FIRST_GATE_RIGHT_POLE, simplified_corridor.at(0).getRightPole());
  ASSERT_EQ(SECOND_GATE_LEFT_POLE, simplified_corridor.at(1).getLeftPole());
  ASSERT_EQ(SECOND_GATE_RIGHT_POLE, simplified_corridor.at(1).getRightPole());
}

/* assumption no longer appropriate
TEST_F(DrivingCorridorTest, simplifiedTwoCloseGatesMatchingDistanceParameter) {
  const DrivingCorridor driving_corridor = createDrivingCorridorTwoCloseGates();
  const DrivingCorridor simplified_corridor = driving_corridor.simplified(4);
  const Eigen::Vector3d expected_left_pole = 0.5 * (FIRST_GATE_LEFT_POLE +
SECOND_GATE_LEFT_POLE);
  const Eigen::Vector3d expected_right_pole = 0.5 * (FIRST_GATE_RIGHT_POLE +
SECOND_GATE_RIGHT_POLE);
  ASSERT_EQ(1, simplified_corridor.size());
  ASSERT_EQ(expected_left_pole, simplified_corridor.at(0).getLeftPole());
  ASSERT_EQ(expected_right_pole, simplified_corridor.at(0).getRightPole());
}
*/

TEST_F(DrivingCorridorTest, simplifiedTwoFarGatesSmallDistanceParameter) {
  const DrivingCorridor driving_corridor = createDrivingCorridorTwoFarGates();
  const DrivingCorridor simplified_corridor = driving_corridor.simplified(4);
  ASSERT_EQ(2, simplified_corridor.size());
  ASSERT_EQ(FIRST_GATE_LEFT_POLE, simplified_corridor.at(0).getLeftPole());
  ASSERT_EQ(FIRST_GATE_RIGHT_POLE, simplified_corridor.at(0).getRightPole());
  ASSERT_EQ(THIRD_GATE_LEFT_POLE, simplified_corridor.at(1).getLeftPole());
  ASSERT_EQ(THIRD_GATE_RIGHT_POLE, simplified_corridor.at(1).getRightPole());
}

TEST_F(DrivingCorridorTest, simplifiedThreeGatesSmallDistanceParameter) {
  const DrivingCorridor driving_corridor = createDrivingCorridorThreeGates();
  const DrivingCorridor simplified_corridor = driving_corridor.simplified(1);
  ASSERT_EQ(3, simplified_corridor.size());
  ASSERT_EQ(FIRST_GATE_LEFT_POLE, simplified_corridor.at(0).getLeftPole());
  ASSERT_EQ(FIRST_GATE_RIGHT_POLE, simplified_corridor.at(0).getRightPole());
  ASSERT_EQ(SECOND_GATE_LEFT_POLE, simplified_corridor.at(1).getLeftPole());
  ASSERT_EQ(SECOND_GATE_RIGHT_POLE, simplified_corridor.at(1).getRightPole());
  ASSERT_EQ(THIRD_GATE_LEFT_POLE, simplified_corridor.at(2).getLeftPole());
  ASSERT_EQ(THIRD_GATE_RIGHT_POLE, simplified_corridor.at(2).getRightPole());
}

/* assumption no longer appropriate
TEST_F(DrivingCorridorTest, simplifiedThreeGatesMediumDistanceParameter) {
  const DrivingCorridor driving_corridor = createDrivingCorridorThreeGates();
  const DrivingCorridor simplified_corridor = driving_corridor.simplified(5);
  const Eigen::Vector3d expected_left_pole = 0.5 * (FIRST_GATE_LEFT_POLE +
SECOND_GATE_LEFT_POLE);
  const Eigen::Vector3d expected_right_pole = 0.5 * (FIRST_GATE_RIGHT_POLE +
SECOND_GATE_RIGHT_POLE);
  ASSERT_EQ(2, simplified_corridor.size());
  ASSERT_EQ(expected_left_pole, simplified_corridor.at(0).getLeftPole());
  ASSERT_EQ(expected_right_pole, simplified_corridor.at(0).getRightPole());
  ASSERT_EQ(THIRD_GATE_LEFT_POLE, simplified_corridor.at(1).getLeftPole());
  ASSERT_EQ(THIRD_GATE_RIGHT_POLE, simplified_corridor.at(1).getRightPole());
}
*/

/* assumption no longer appropriate
TEST_F(DrivingCorridorTest, simplifiedThreeGatesLargeDistanceParameter) {
  const DrivingCorridor driving_corridor = createDrivingCorridorThreeGates();
  const DrivingCorridor simplified_corridor = driving_corridor.simplified(10);
  ASSERT_EQ(1, simplified_corridor.size());
  ASSERT_EQ(SECOND_GATE_LEFT_POLE, simplified_corridor.at(0).getLeftPole());
  ASSERT_EQ(SECOND_GATE_RIGHT_POLE, simplified_corridor.at(0).getRightPole());
}
*/

TEST_F(DrivingCorridorTest, isPointApproximateContainedPointA) {
  const DrivingCorridor corridor = createDrivingCorridorThreeGates();
  ASSERT_FALSE(corridor.isPointApproximateContained(POINT_A));
}

TEST_F(DrivingCorridorTest, isPointApproximateContainedPointB) {
  const DrivingCorridor corridor = createDrivingCorridorThreeGates();
  ASSERT_FALSE(corridor.isPointApproximateContained(POINT_B));
}

TEST_F(DrivingCorridorTest, isPointApproximateContainedPointC) {
  const DrivingCorridor corridor = createDrivingCorridorThreeGates();
  ASSERT_TRUE(corridor.isPointApproximateContained(POINT_C));
}

TEST_F(DrivingCorridorTest, isPointApproximateContainedPointD) {
  const DrivingCorridor corridor = createDrivingCorridorThreeGates();
  ASSERT_TRUE(corridor.isPointApproximateContained(POINT_D));
}

TEST_F(DrivingCorridorTest, isPointApproximateContainedPointE) {
  const DrivingCorridor corridor = createDrivingCorridorThreeGates();
  ASSERT_FALSE(corridor.isPointApproximateContained(POINT_E));
}

TEST_F(DrivingCorridorTest, isPointApproximateContainedPointF) {
  const DrivingCorridor corridor = createDrivingCorridorThreeGates();
  ASSERT_FALSE(corridor.isPointApproximateContained(POINT_F));
}

TEST_F(DrivingCorridorTest, isPointApproximateContainedFourGatesPointE) {
  const DrivingCorridor corridor = createDrivingCorridorFourGates();
  ASSERT_TRUE(corridor.isPointApproximateContained(POINT_E));
}

TEST_F(DrivingCorridorTest, isPointApproximateContainedFourGatesPointF) {
  const DrivingCorridor corridor = createDrivingCorridorFourGates();
  ASSERT_TRUE(corridor.isPointApproximateContained(POINT_F));
}

TEST_F(DrivingCorridorTest, isPointContainedPointA) {
  const DrivingCorridor corridor = createDrivingCorridorThreeGates();
  ASSERT_FALSE(corridor.isPointContained(POINT_A));
}

TEST_F(DrivingCorridorTest, isPointContainedPointB) {
  const DrivingCorridor corridor = createDrivingCorridorThreeGates();
  ASSERT_FALSE(corridor.isPointContained(POINT_B));
}

TEST_F(DrivingCorridorTest, isPointContainedPointC) {
  const DrivingCorridor corridor = createDrivingCorridorThreeGates();
  ASSERT_TRUE(corridor.isPointContained(POINT_C));
}

TEST_F(DrivingCorridorTest, isPointContainedPointD) {
  const DrivingCorridor corridor = createDrivingCorridorThreeGates();
  ASSERT_TRUE(corridor.isPointContained(POINT_D));
}

TEST_F(DrivingCorridorTest, isPointContainedPointE) {
  const DrivingCorridor corridor = createDrivingCorridorThreeGates();
  ASSERT_FALSE(corridor.isPointContained(POINT_E));
}

TEST_F(DrivingCorridorTest, isPointContainedPoles) {
  const DrivingCorridor corridor = createDrivingCorridorThreeGates();
  ASSERT_TRUE(corridor.isPointContained(FIRST_GATE_LEFT_POLE));
  ASSERT_TRUE(corridor.isPointContained(FIRST_GATE_RIGHT_POLE));
  ASSERT_TRUE(corridor.isPointContained(SECOND_GATE_LEFT_POLE));
  ASSERT_TRUE(corridor.isPointContained(SECOND_GATE_RIGHT_POLE));
  ASSERT_TRUE(corridor.isPointContained(THIRD_GATE_LEFT_POLE));
  ASSERT_TRUE(corridor.isPointContained(THIRD_GATE_RIGHT_POLE));
}

TEST_F(DrivingCorridorTest, isPointContainedPointF) {
  const DrivingCorridor corridor = createDrivingCorridorThreeGates();
  ASSERT_FALSE(corridor.isPointContained(POINT_F));
}

TEST_F(DrivingCorridorTest, isPointContainedFourGatesPointE) {
  const DrivingCorridor corridor = createDrivingCorridorFourGates();
  ASSERT_TRUE(corridor.isPointContained(POINT_E));
}

TEST_F(DrivingCorridorTest, isPointContainedFourGatesPointF) {
  const DrivingCorridor corridor = createDrivingCorridorFourGates();
  ASSERT_FALSE(corridor.isPointContained(POINT_F));
}

TEST_F(DrivingCorridorTest, isPointOnRightLaneFourGatesPointF) {
  const DrivingCorridor corridor = createDrivingCorridorFourGates();
  ASSERT_TRUE(corridor.isPointOnRightLane(POINT_F));
}

TEST_F(DrivingCorridorTest, isPointOnRightLaneFourGatesPointG) {
  const DrivingCorridor corridor = createDrivingCorridorFourGates();
  ASSERT_FALSE(corridor.isPointOnRightLane(POINT_G));
}

}  // namespace

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
