#include <common/macros.h>

THIRD_PARTY_HEADERS_BEGIN
#include <gtest/gtest.h>
#include <Eigen/Dense>
#include <limits>
THIRD_PARTY_HEADERS_END

#include "../src/road_object_detection/road_objects/road_object.h"
#include "../src/road_watcher/object_tracking/object_tracker.h"

DISABLE_SUGGEST_OVERRIDE_WARNING

TEST(RoadWatcherObjectTrackerTest, noRoadObjects) {

  ObjectTracker tracker;
  road_object_detection::RoadObjects road_objects;
  tracker.trackRoadObjects(road_objects, ros::Time(1));
}

TEST(RoadWatcherObjectTrackerTest, trackJunction) {

  ObjectTracker tracker;

  VehiclePose vehicle_pose = Eigen::Affine3d::Identity();
  road_object_detection::RoadObjects road_objects;
  road_objects.push_back(std::make_unique<road_object_detection::Junction>(
      road_object_detection::Junction::stopline_right, ros::Time(1), 1.0, vehicle_pose, VehiclePoints()));
  road_objects.front()->pose_in_world = road_objects.front()->pose_in_vehicle;
  tracker.trackRoadObjects(road_objects, ros::Time(1));
  EXPECT_NE(road_objects.front()->id, 0);
  EXPECT_NE(road_objects.front()->id, HungarianAlgorithm::UNASSINGED);
  const int id = road_objects.front()->id;

  road_objects.clear();
  road_objects.push_back(std::make_unique<road_object_detection::Junction>(
      road_object_detection::Junction::stopline_right, ros::Time(2), 1.0, vehicle_pose, VehiclePoints()));
  road_objects.front()->pose_in_world = road_objects.front()->pose_in_vehicle;
  tracker.trackRoadObjects(road_objects, ros::Time(2));
  EXPECT_NE(road_objects.front()->id, HungarianAlgorithm::UNASSINGED);
  EXPECT_EQ(road_objects.front()->id, id);

  // still keeping track of junction
  road_objects.clear();
  tracker.trackRoadObjects(road_objects, ros::Time(5));
  EXPECT_EQ(tracker.size(), 1);

  // not keeping track of junction
  road_objects.clear();
  tracker.trackRoadObjects(road_objects, ros::Time(7));
  EXPECT_EQ(tracker.size(), 0);

  // "new" junction with new id
  road_objects.clear();
  road_objects.push_back(std::make_unique<road_object_detection::Junction>(
      road_object_detection::Junction::stopline_right, ros::Time(8), 1.0, vehicle_pose, VehiclePoints()));
  road_objects.front()->pose_in_world = road_objects.front()->pose_in_vehicle;
  tracker.trackRoadObjects(road_objects, ros::Time(8));
  EXPECT_NE(road_objects.front()->id, HungarianAlgorithm::UNASSINGED);
  EXPECT_NE(road_objects.front()->id, id);
}


TEST(RoadWatcherObjectTrackerTest, tracktwoJunctions) {
  ObjectTracker tracker;
  VehiclePose vehicle_pose = Eigen::Affine3d::Identity();

  road_object_detection::RoadObjects road_objects;
  road_objects.push_back(std::make_unique<road_object_detection::Junction>(
      road_object_detection::Junction::stopline_right, ros::Time(1), 1.0, vehicle_pose, VehiclePoints()));
  road_objects.front()->pose_in_world = road_objects.front()->pose_in_vehicle;
  tracker.trackRoadObjects(road_objects, ros::Time(1));
  EXPECT_NE(road_objects.front()->id, 0);
  EXPECT_NE(road_objects.front()->id, HungarianAlgorithm::UNASSINGED);
  const int id_junction_1 = road_objects.front()->id;

  // mesurement too far away => create new track/id
  road_objects.clear();
  vehicle_pose.translation() = Eigen::Vector3d(1, 1, 0);
  road_objects.push_back(std::make_unique<road_object_detection::Junction>(
      road_object_detection::Junction::stopline_right, ros::Time(3), 1.0, vehicle_pose, VehiclePoints()));
  road_objects.front()->pose_in_world = road_objects.front()->pose_in_vehicle;
  tracker.trackRoadObjects(road_objects, ros::Time(3));
  EXPECT_NE(road_objects.front()->id, HungarianAlgorithm::UNASSINGED);
  EXPECT_NE(road_objects.front()->id, id_junction_1);
  const int id_junction_2 = road_objects.front()->id;

  // mesurement of junction_1
  road_objects.clear();
  vehicle_pose.translation() = Eigen::Vector3d(0, 0.1, 0);
  road_objects.push_back(std::make_unique<road_object_detection::Junction>(
      road_object_detection::Junction::stopline_right, ros::Time(4), 1.0, vehicle_pose, VehiclePoints()));
  road_objects.front()->pose_in_world = road_objects.front()->pose_in_vehicle;
  tracker.trackRoadObjects(road_objects, ros::Time(4));
  EXPECT_NE(road_objects.front()->id, HungarianAlgorithm::UNASSINGED);
  EXPECT_EQ(road_objects.front()->id, id_junction_1);

  // mesurement of both
  road_objects.clear();
  vehicle_pose.translation() = Eigen::Vector3d(1.1, 1.1, 0);
  road_objects.push_back(std::make_unique<road_object_detection::Junction>(
      road_object_detection::Junction::stopline_right, ros::Time(5), 1.0, vehicle_pose, VehiclePoints()));
  road_objects.front()->pose_in_world = road_objects.front()->pose_in_vehicle;
  tracker.trackRoadObjects(road_objects, ros::Time(5));
  EXPECT_NE(road_objects.front()->id, HungarianAlgorithm::UNASSINGED);
  EXPECT_EQ(road_objects.front()->id, id_junction_2);

  // mesurement of junction_2
  road_objects.clear();
  vehicle_pose.translation() = Eigen::Vector3d(0.1, 0.1, 0);
  road_objects.push_back(std::make_unique<road_object_detection::Junction>(
      road_object_detection::Junction::stopline_right, ros::Time(6), 1.0, vehicle_pose, VehiclePoints()));
  road_objects.front()->pose_in_world = road_objects.front()->pose_in_vehicle;
  vehicle_pose.translation() = Eigen::Vector3d(1.0, 0.9, 0);
  road_objects.push_back(std::make_unique<road_object_detection::Junction>(
      road_object_detection::Junction::stopline_right, ros::Time(6), 1.0, vehicle_pose, VehiclePoints()));
  road_objects.back()->pose_in_world = road_objects.back()->pose_in_vehicle;
  tracker.trackRoadObjects(road_objects, ros::Time(6));
  EXPECT_NE(road_objects.front()->id, HungarianAlgorithm::UNASSINGED);
  EXPECT_EQ(road_objects.front()->id, id_junction_1);
  EXPECT_NE(road_objects.back()->id, HungarianAlgorithm::UNASSINGED);
  EXPECT_EQ(road_objects.back()->id, id_junction_2);

  // loose track of junction_1 but not junction_2
  road_objects.clear();
  vehicle_pose.translation() = Eigen::Vector3d(1.0, 0.9, 0);
  road_objects.push_back(std::make_unique<road_object_detection::Junction>(
      road_object_detection::Junction::stopline_right, ros::Time(11), 1.0, vehicle_pose, VehiclePoints()));
  road_objects.back()->pose_in_world = road_objects.back()->pose_in_vehicle;
  tracker.trackRoadObjects(road_objects, ros::Time(11));
  EXPECT_NE(road_objects.back()->id, HungarianAlgorithm::UNASSINGED);
  EXPECT_EQ(road_objects.back()->id, id_junction_2);
  EXPECT_EQ(tracker.size(), 1);
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
