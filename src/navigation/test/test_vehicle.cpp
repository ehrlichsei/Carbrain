#include <common/macros.h>

THIRD_PARTY_HEADERS_BEGIN
#include <gtest/gtest.h>
THIRD_PARTY_HEADERS_END

#include "../src/collision_detection/vehicle.h"

DISABLE_SUGGEST_OVERRIDE_WARNING

// execute tests with one of the following command:
// catkin_make run_tests_navigation
// catkin_make run_tests_navigation_gtest_vehicle_test

namespace Eigen {

void PrintTo(const Eigen::Vector2d& vector, ::std::ostream* os) {
  *os << "\n" << vector;
}

}  // namespace Eigen

namespace {

class VehicleTest : public ::testing::Test {
 public:
  static const Eigen::Vector2d REAR_LEFT_;
  static const Eigen::Vector2d REAR_RIGHT_;
  static const Eigen::Vector2d FRONT_LEFT_;
  static const Eigen::Vector2d FRONT_RIGHT_;
  static const double WIDTH;
  static const double DISTANCE_TO_REAR_BUMPER;
  static const double DISTANCE_TO_FRONT_BUMPER;
  static const Eigen::Affine3d TRANSFORMATION_3D;
  static const Eigen::Affine2d TRANSFORMATION_2D;

  static Vehicle createVehicle() {
    return Vehicle(WIDTH, DISTANCE_TO_REAR_BUMPER, DISTANCE_TO_FRONT_BUMPER);
  }
};

//
//     ^
//     |             Left
//  1 -|       +--===------===-+
//     |       |   |        |\ |
//  0 -|  Rear |   + Vehicle| ]| Front
//     |       |   |        |/ |
// -1 -|       +--===------===-+
//     |             Right
// -2 -|---+---+---+---+---+---+-->
//     '   |   '   |   '   |   '   |
//        -2       0       2       4
//

const Eigen::Vector2d VehicleTest::REAR_LEFT_(-1, 1);
const Eigen::Vector2d VehicleTest::REAR_RIGHT_(-1, -1);
const Eigen::Vector2d VehicleTest::FRONT_LEFT_(3, 1);
const Eigen::Vector2d VehicleTest::FRONT_RIGHT_(3, -1);
const double VehicleTest::WIDTH(2);
const double VehicleTest::DISTANCE_TO_REAR_BUMPER(1);
const double VehicleTest::DISTANCE_TO_FRONT_BUMPER(3);
const Eigen::Affine3d VehicleTest::TRANSFORMATION_3D(
    Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitZ()));
const Eigen::Affine2d VehicleTest::TRANSFORMATION_2D(Eigen::Rotation2D<double>(M_PI));

TEST_F(VehicleTest, getLeftSide) {
  const Vehicle vehicle = createVehicle();
  const LineSegment line_segment = vehicle.getLeftSide();
  ASSERT_EQ(REAR_LEFT_, line_segment.getStartPoint());
  ASSERT_EQ(FRONT_LEFT_, line_segment.getEndPoint());
}

TEST_F(VehicleTest, getRightSide) {
  const Vehicle vehicle = createVehicle();
  const LineSegment line_segment = vehicle.getRightSide();
  ASSERT_EQ(REAR_RIGHT_, line_segment.getStartPoint());
  ASSERT_EQ(FRONT_RIGHT_, line_segment.getEndPoint());
}

TEST_F(VehicleTest, getRearBumper) {
  const Vehicle vehicle = createVehicle();
  const LineSegment line_segment = vehicle.getRearBumper();
  ASSERT_EQ(REAR_LEFT_, line_segment.getStartPoint());
  ASSERT_EQ(REAR_RIGHT_, line_segment.getEndPoint());
}

TEST_F(VehicleTest, getFrontBumper) {
  const Vehicle vehicle = createVehicle();
  const LineSegment line_segment = vehicle.getFrontBumper();
  ASSERT_EQ(FRONT_LEFT_, line_segment.getStartPoint());
  ASSERT_EQ(FRONT_RIGHT_, line_segment.getEndPoint());
}

TEST_F(VehicleTest, transform) {
  Vehicle vehicle = createVehicle();
  vehicle.transform(TRANSFORMATION_3D);
  const LineSegment line_segment = vehicle.getLeftSide();
  ASSERT_EQ(TRANSFORMATION_2D * REAR_LEFT_, line_segment.getStartPoint());
  ASSERT_EQ(TRANSFORMATION_2D * FRONT_LEFT_, line_segment.getEndPoint());
}

TEST_F(VehicleTest, transformReturnThis) {
  Vehicle vehicle = createVehicle();
  ASSERT_EQ(&vehicle, &(vehicle.transform(TRANSFORMATION_3D)));
}

TEST_F(VehicleTest, transformed) {
  const Vehicle vehicle = createVehicle();
  ASSERT_EQ(TRANSFORMATION_2D * REAR_LEFT_,
            vehicle.transformed(TRANSFORMATION_3D).getLeftSide().getStartPoint());
  ASSERT_EQ(TRANSFORMATION_2D * FRONT_LEFT_,
            vehicle.transformed(TRANSFORMATION_3D).getLeftSide().getEndPoint());
}

}  // namespace

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
