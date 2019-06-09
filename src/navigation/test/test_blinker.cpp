#include <common/macros.h>

THIRD_PARTY_HEADERS_BEGIN
#include <gtest/gtest.h>
#include <cmath>
THIRD_PARTY_HEADERS_END

#include "../src/blinker/blinker.h"
#include "common/test/dummy_parameter_handler.h"
#include "navigation/driving_corridor.h"
#include "navigation/gate.h"

DISABLE_SUGGEST_OVERRIDE_WARNING

static ParameterString<double> VEHICLE_WIDTH("/car_specs/vehicle_width");
static ParameterString<double> VEHICLE_WIDTH_SUBSTRACTION(
    "vehicle_width_subtraction");

DrivingCorridor generateCorridor(const std::vector<double>& left_poles,
                                 const std::vector<double>& right_poles) {
  Eigen::Vector3d left_lane_bondary = Eigen::Vector3d(0, 1, 0);
  Eigen::Vector3d right_lane_bondary = Eigen::Vector3d(0, 0, 0);
  const Eigen::Vector3d add_x = Eigen::Vector3d(1, 0, 0);
  const double center = 0.5;
  DrivingCorridor safe_corridor;
  for (unsigned int i = 0; i < left_poles.size(); i++) {
    left_lane_bondary += add_x;
    right_lane_bondary += add_x;
    Gate gate = Gate(
        i, left_poles.at(i), right_poles.at(i), left_lane_bondary, right_lane_bondary, center);
    safe_corridor.push_back(gate);
  }
  return safe_corridor;
}

TEST(TestBlinker, testSwitchFromRightToLeftLane) {
  DrivingCorridor safe_corridor =
      generateCorridor({0.5, 0, 0, 0, 0, 0, 0}, {1, 1, 1, 1, 0.7, 0.7, 0.7});

  BlinkerDecision blinker_decision =
      Blinker().decideBlink(safe_corridor, Eigen::Affine3d::Identity());

  ASSERT_EQ(BlinkerDecision::BLINK_LEFT, blinker_decision);
}

TEST(TestBlinker, testSwitchFromLeftToRightLane) {
  DrivingCorridor safe_corridor =
      generateCorridor({0, 0, 0, 0, 0.3, 0.3, 0.3}, {0.7, 1, 1, 1, 1, 1, 1});

  BlinkerDecision blinker_decision =
      Blinker().decideBlink(safe_corridor, Eigen::Affine3d::Identity());

  ASSERT_EQ(BlinkerDecision::BLINK_RIGHT, blinker_decision);
}

TEST(TestBlinker, testStayOnRightLane) {
  DrivingCorridor safe_corridor =
      generateCorridor({0.5, 0.5, 0.5, 0.5, 0.5}, {1, 1, 1, 1, 1});

  BlinkerDecision blinker_decision =
      Blinker().decideBlink(safe_corridor, Eigen::Affine3d::Identity());

  ASSERT_EQ(BlinkerDecision::ABORT_BLINKING, blinker_decision);
}

TEST(TestBlinker, testStayOnLeftLane) {
  DrivingCorridor safe_corridor =
      generateCorridor({0, 0, 0, 0, 0}, {0.5, 0.5, 0.5, 0.5, 0.5});

  BlinkerDecision blinker_decision =
      Blinker().decideBlink(safe_corridor, Eigen::Affine3d::Identity());

  ASSERT_EQ(BlinkerDecision::ABORT_BLINKING, blinker_decision);
}

TEST(TestBlinker, testEmptyCorridor) {
  DrivingCorridor safe_corridor = generateCorridor({}, {});

  BlinkerDecision blinker_decision =
      Blinker().decideBlink(safe_corridor, Eigen::Affine3d::Identity());

  ASSERT_EQ(BlinkerDecision::NO_DECISION, blinker_decision);
}

// Run all the tests that were declared with TEST()
int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
