#include <common/macros.h>

THIRD_PARTY_HEADERS_BEGIN
#include <gtest/gtest.h>
#include "navigation_msgs/Gate.h"
THIRD_PARTY_HEADERS_END

#include "navigation/gate.h"
#include "../src/path_preprocessing/path_preprocessing.h"
#include "../src/path_preprocessing/gate_utils.h"

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

class GateUtilsTest : public ::testing::Test {
 public:
  static const Eigen::Vector3d LEFT_POLE;
  static const Eigen::Vector3d RIGHT_POLE;
  static const Eigen::Vector3d LEFT_POLE_GATE_FRAME;
  static const Eigen::Vector3d RIGHT_POLE_GATE_FRAME;
  static const Eigen::Vector3d CENTER;
  static const Eigen::Vector3d POINT_A;
  static const Eigen::Vector3d POINT_B;
  static const Eigen::Vector3d POINT_C;

 protected:
  static Gate createGate() { return Gate(42, LEFT_POLE, RIGHT_POLE); }

  static common::EigenAlignedVector<Eigen::Vector3d> createLeftLane() {
    common::EigenAlignedVector<Eigen::Vector3d> lane;
    lane.reserve(10);
    for (int i = 0; i < 10; i++) {
      lane.emplace_back(i, 6, 0);
    }
    return lane;
  }

  static common::EigenAlignedVector<Eigen::Vector3d> createRightLane() {
    common::EigenAlignedVector<Eigen::Vector3d> lane;
    lane.reserve(5);
    for (int i = 0; i < 10; i += 2) {
      lane.emplace_back(i, 0 + 0.01 * i, 0);
    }
    return lane;
  }

  static common::EigenAlignedVector<Eigen::Vector3d> createSmallLane() {
    common::EigenAlignedVector<Eigen::Vector3d> lane;
    lane.reserve(2);
    for (int i = 0; i < 2; i++) {
      lane.emplace_back(4 + 2 * i, 4, 0);
    }
    return lane;
  }

  static common::EigenAlignedVector<Eigen::Vector3d> createSmallLaneBefore() {
    common::EigenAlignedVector<Eigen::Vector3d> lane;
    lane.reserve(4);
    for (int i = 0; i < 4; i++) {
      lane.emplace_back(i, 4, 0);
    }
    return lane;
  }

  static common::EigenAlignedVector<Eigen::Vector3d> createSmallLaneAfter() {
    common::EigenAlignedVector<Eigen::Vector3d> lane;
    lane.reserve(4);
    for (int i = 0; i < 4; i++) {
      lane.emplace_back(i + 6, 4, 0);
    }
    return lane;
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
const Eigen::Vector3d GateUtilsTest::LEFT_POLE(5, 6, 0);
const Eigen::Vector3d GateUtilsTest::RIGHT_POLE(5, 2, 0);
const Eigen::Vector3d GateUtilsTest::LEFT_POLE_GATE_FRAME(0, 0, 0);
const Eigen::Vector3d GateUtilsTest::RIGHT_POLE_GATE_FRAME(4, 0, 0);
const Eigen::Vector3d GateUtilsTest::CENTER(0.5 * (LEFT_POLE + RIGHT_POLE));
const Eigen::Vector3d GateUtilsTest::POINT_A(9, 8, 0);
const Eigen::Vector3d GateUtilsTest::POINT_B(9, 4, 0);
const Eigen::Vector3d GateUtilsTest::POINT_C(1, 5, 0);

TEST_F(GateUtilsTest, intersectLeftLane) {
  const Gate gate = createGate();
  const auto lane = createLeftLane();
  GateUtils::Intersection intersection;
  intersection = GateUtils::getIntersection(gate, lane);
  ASSERT_TRUE(intersection.has_intersection);
  ASSERT_EQ(LEFT_POLE, intersection.point);
}

TEST_F(GateUtilsTest, intersectRightLane) {
  const Gate gate = createGate();
  const auto lane = createRightLane();
  GateUtils::Intersection intersection;
  intersection = GateUtils::getIntersection(gate, lane);
  ASSERT_TRUE(intersection.has_intersection);
  double distance = (Eigen::Vector3d(5, 0.05, 0) - intersection.point).norm();
  ASSERT_TRUE(distance < 0.001);
}

TEST_F(GateUtilsTest, intersectSmallLane) {
  const Gate gate = createGate();
  const auto lane = createSmallLane();
  GateUtils::Intersection intersection;
  intersection = GateUtils::getIntersection(gate, lane);
  ASSERT_TRUE(intersection.has_intersection);
  ASSERT_EQ(Eigen::Vector3d(5, 4, 0), intersection.point);
}

TEST_F(GateUtilsTest, intersectSmallLaneBeforeGate) {
  const Gate gate = createGate();
  const auto lane = createSmallLaneBefore();
  GateUtils::Intersection intersection;
  intersection = GateUtils::getIntersection(gate, lane);
  ASSERT_FALSE(intersection.has_intersection);
}

TEST_F(GateUtilsTest, intersectSmallLaneAfterGate) {
  const Gate gate = createGate();
  const auto lane = createSmallLaneAfter();
  GateUtils::Intersection intersection;
  intersection = GateUtils::getIntersection(gate, lane);
  ASSERT_FALSE(intersection.has_intersection);
}

}  // namespace

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
