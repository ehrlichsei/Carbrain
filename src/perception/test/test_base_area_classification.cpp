#include "common/macros.h"

THIRD_PARTY_HEADERS_BEGIN
#include <gtest/gtest.h>
#include <tf2_ros/buffer.h>
#include <boost/range/adaptors.hpp>
#include <eigen3/Eigen/Dense>
THIRD_PARTY_HEADERS_END

#include "../src/road_watcher/classification.h"
#include "common/basic_statistics.h"
#include "common/basic_statistics_eigen.h"

DISABLE_SUGGEST_OVERRIDE_WARNING

// test fixture
class BaseAreaClassificationTest : public ::testing::Test {
 public:
  BaseAreaClassificationTest() {}

 protected:
  road_object_detection::RoadObjects test_classifications;

  const VehiclePoints test_hull_polygon = {{VehiclePoint{0, 0, 0},
                                            VehiclePoint{1, 0, 0},
                                            VehiclePoint{1, 1, 0},
                                            VehiclePoint{0, 1, 0}}};


  const VehiclePose fake_pose = Eigen::Translation3d{common::mean(test_hull_polygon)} *
                                Eigen::AngleAxisd{0, VehiclePoint::UnitX()};
};

using Direction = VehiclePoint;
using namespace boost::adaptors;

VehiclePoints shiftPolygon(const VehiclePoints& in,
                           const Direction& shift_direction,
                           const double length) {
  assert(shift_direction.norm() == 1);
  const auto shift = [&shift_direction, length](const auto& p) {
    return p + length * shift_direction;
  };
  const auto out = in | transformed(shift);
  return VehiclePoints(out.begin(), out.end());
}

VehiclePoints rotatePolygon(const VehiclePoints& in, const double angle) {
  const auto rotation =
      Eigen::AngleAxisd{angle, VehiclePoint::UnitX()}.toRotationMatrix();

  const auto rotate = [&rotation](const auto& p) { return rotation * p; };

  const auto out = in | transformed(rotate);
  return VehiclePoints(out.begin(), out.end());
}



TEST_F(BaseAreaClassificationTest, singleRoadObject) {
  //  ClassificationMockObject classification;
  const double score = 1;
  const VehiclePose fake_pose = Eigen::Translation3d{common::mean(test_hull_polygon)} *
                                Eigen::AngleAxisd{0, VehiclePoint::UnitX()};
  test_classifications.push_back(std::make_unique<road_object_detection::Obstacle>(
      ros::Time(0), score, fake_pose, test_hull_polygon));
  voting::voteOnIntersectingRoadObjects(test_classifications);
  EXPECT_EQ(1, test_classifications.size());
  EXPECT_EQ(score, test_classifications.front()->score);
  EXPECT_EQ("Obstacle", test_classifications.front()->type);
}

TEST_F(BaseAreaClassificationTest, twoObjectsNoIntersection) {
  test_classifications.push_back(std::make_unique<road_object_detection::ArrowMarking>(
      ros::Time(0), 1, fake_pose, test_hull_polygon, road_object_detection::ArrowMarking::Type::TURN_LEFT));
  const VehiclePoints test_hull_polygon2 =
      shiftPolygon(test_hull_polygon, Direction::UnitX(), 2.0);
  const VehiclePose fake_pose2 = Eigen::Translation3d{common::mean(test_hull_polygon2)} *
                                 Eigen::AngleAxisd{0, VehiclePoint::UnitX()};
  test_classifications.push_back(std::make_unique<road_object_detection::Crosswalk>(
      ros::Time(0), 0.5, fake_pose2, test_hull_polygon2));

  voting::voteOnIntersectingRoadObjects(test_classifications);
  EXPECT_EQ(2, test_classifications.size());
}

TEST_F(BaseAreaClassificationTest, twoIntersectingObjects) {
  test_classifications.push_back(std::make_unique<road_object_detection::ArrowMarking>(
      ros::Time(0), 1, fake_pose, test_hull_polygon, road_object_detection::ArrowMarking::Type::TURN_LEFT));
  const VehiclePoints test_hull_polygon2 =
      shiftPolygon(test_hull_polygon, Direction::UnitX(), 0.5);
  const VehiclePose fake_pose2 = Eigen::Translation3d{common::mean(test_hull_polygon2)} *
                                 Eigen::AngleAxisd{0, VehiclePoint::UnitX()};
  test_classifications.push_back(std::make_unique<road_object_detection::Crosswalk>(
      ros::Time(0), 0.5, fake_pose2, test_hull_polygon2));

  voting::voteOnIntersectingRoadObjects(test_classifications);
  EXPECT_EQ(1, test_classifications.size());
  EXPECT_EQ(1, test_classifications.front()->score);
  EXPECT_EQ("Arrow Marking", test_classifications.front()->type);
  EXPECT_EQ(0,
            (test_classifications.front()->pose_in_vehicle.translation() -
             fake_pose.translation())
                .norm());
  EXPECT_EQ(0,
            (common::mean(test_classifications.front()->base_hull_polygon_in_vehicle) -
             common::mean(test_hull_polygon))
                .norm());
}

TEST_F(BaseAreaClassificationTest, threeIntersectingObjects) {
  test_classifications.push_back(std::make_unique<road_object_detection::ArrowMarking>(
      ros::Time(0), 0.8, fake_pose, test_hull_polygon, road_object_detection::ArrowMarking::Type::TURN_LEFT));
  const VehiclePoints test_hull_polygon2 =
      shiftPolygon(test_hull_polygon, Direction::UnitX(), 0.5);
  const VehiclePose fake_pose2 = Eigen::Translation3d{common::mean(test_hull_polygon2)} *
                                 Eigen::AngleAxisd{0, VehiclePoint::UnitX()};
  test_classifications.push_back(std::make_unique<road_object_detection::Crosswalk>(
      ros::Time(0), 0.2, fake_pose2, test_hull_polygon2));
  const VehiclePoints test_hull_polygon3 =
      shiftPolygon(rotatePolygon(test_hull_polygon, 0.2), VehiclePoint::UnitX(), 0.2);
  const VehiclePose fake_pose3 = Eigen::Translation3d{common::mean(test_hull_polygon3)} *
                                 Eigen::AngleAxisd{0, VehiclePoint::UnitX()};
  test_classifications.push_back(std::make_unique<road_object_detection::Obstacle>(
      ros::Time(0), 0.6, fake_pose3, test_hull_polygon3));

  voting::voteOnIntersectingRoadObjects(test_classifications);
  EXPECT_EQ(1, test_classifications.size());
  EXPECT_EQ(0.8, test_classifications.front()->score);
  EXPECT_EQ("Arrow Marking", test_classifications.front()->type);
  EXPECT_EQ(0,
            (test_classifications.front()->pose_in_vehicle.translation() -
             fake_pose.translation())
                .norm());
  EXPECT_EQ(0,
            (common::mean(test_classifications.front()->base_hull_polygon_in_vehicle) -
             common::mean(test_hull_polygon))
                .norm());
}

TEST_F(BaseAreaClassificationTest, threeIntersectionsOneRemoved) {
  test_classifications.push_back(std::make_unique<road_object_detection::ArrowMarking>(
      ros::Time(0), 0.8, fake_pose, test_hull_polygon, road_object_detection::ArrowMarking::Type::TURN_LEFT));
  const VehiclePoints test_hull_polygon2 =
      shiftPolygon(test_hull_polygon, Direction::UnitX(), 0.5);
  const VehiclePose fake_pose2 = Eigen::Translation3d{common::mean(test_hull_polygon2)} *
                                 Eigen::AngleAxisd{0, VehiclePoint::UnitX()};
  test_classifications.push_back(std::make_unique<road_object_detection::Crosswalk>(
      ros::Time(0), 0.2, fake_pose2, test_hull_polygon2));
  const VehiclePoints test_hull_polygon3 =
      shiftPolygon(rotatePolygon(test_hull_polygon, 0.1), VehiclePoint::UnitX(), 1.2);
  const VehiclePose fake_pose3 = Eigen::Translation3d{common::mean(test_hull_polygon3)} *
                                 Eigen::AngleAxisd{0, VehiclePoint::UnitX()};
  test_classifications.push_back(std::make_unique<road_object_detection::Obstacle>(
      ros::Time(0), 0.6, fake_pose3, test_hull_polygon3));

  voting::voteOnIntersectingRoadObjects(test_classifications);
  EXPECT_EQ(2, test_classifications.size());
  EXPECT_EQ(0.8, test_classifications.front()->score);
  EXPECT_EQ(0.6, test_classifications.back()->score);
  EXPECT_EQ("Arrow Marking", test_classifications.front()->type);
  EXPECT_EQ("Obstacle", test_classifications.back()->type);
  EXPECT_EQ(0,
            (test_classifications.front()->pose_in_vehicle.translation() -
             fake_pose.translation())
                .norm());
  EXPECT_EQ(0,
            (test_classifications.back()->pose_in_vehicle.translation() -
             fake_pose3.translation())
                .norm());
  EXPECT_EQ(0,
            (common::mean(test_classifications.front()->base_hull_polygon_in_vehicle) -
             common::mean(test_hull_polygon))
                .norm());
  EXPECT_EQ(0,
            (common::mean(test_classifications.back()->base_hull_polygon_in_vehicle) -
             common::mean(test_hull_polygon3))
                .norm());
}


int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
