#include <common/macros.h>

THIRD_PARTY_HEADERS_BEGIN
#include <gtest/gtest.h>
#include <ros/callback_queue.h>
#include <ros/ros.h>
#include <rosconsole/macros_generated.h>
#include <tf2_eigen/tf2_eigen.h>
#include <memory>
#include <opencv2/imgproc.hpp>

#include "perception_msgs/NoPassingZones.h"
THIRD_PARTY_HEADERS_END

#include "common/angle_conversions.h"
#include "common/testee_module.h"
#include "perception_types.h"

DISABLE_SUGGEST_OVERRIDE_WARNING

class NoPassingZoneTest : public ::testing::Test {
 protected:
  ros::NodeHandle nh;

 private:
  ros::Subscriber no_passing_zones_subscriber;

 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  NoPassingZoneTest()
      : nh("~"),
        timeout(nh.param("timeout", 1.0)),
        expected_start(Eigen::Vector3d(
            nh.param("expected_x_start", 0.0), nh.param("expected_y_start", 0.0), 0)),
        expected_end(Eigen::Vector3d(
            nh.param("expected_x_end", 0.0), nh.param("expected_y_end", 0.0), 0)),
        expected_yaw(nh.param("expected_yaw", 0.0)),
        expect_np_zone(nh.param("expect_np_zone", true)) {}

  // expected yaw ist der Winkel zwischen RoadClosure und dem Auto
  // (features.cluster_center_lane_orientation),
  // expected_x_rect_center, expected_y_rect_center sind die Koordinaten der
  // Mittelpunkte des bounding rects um hull_polygon
  // expected size x und y sind die x- und y-Werte des bounding rects um die
  // RoadClosure
 protected:
  const ros::Duration timeout;
  bool finished = false;
  const Eigen::Vector3d expected_start;
  const Eigen::Vector3d expected_end;
  const double expected_yaw;
  const bool expect_np_zone;

  // Test interface
  void SetUp() override {
    no_passing_zones_subscriber = nh.subscribe(
        "no_passing_zones", 1, &NoPassingZoneTest::handleNoPassingZones, this);
  }

 private:
  void handleNoPassingZones(const perception_msgs::NoPassingZonesConstPtr &msg) {
    ROS_INFO("expect no passing zone is %d", expect_np_zone);
    if (expect_np_zone) {
      ROS_INFO("Received RoadClosure!");
      ASSERT_TRUE(!msg->sub_messages.empty());
      const auto &np_zone = msg->sub_messages.front();


      Eigen::Affine3d np_zone_pose;
      tf2::fromMsg(np_zone.pose, np_zone_pose);

      Eigen::Vector3d start, end;
      tf2::fromMsg(np_zone.start, start);
      tf2::fromMsg(np_zone.end, end);

      double yaw = common::toYaw(np_zone_pose.rotation());

      ROS_INFO_STREAM("start: " << start.x() << " " << start.y());
      ROS_INFO_STREAM("end: " << end.x() << " " << end.y());
      ROS_INFO_STREAM("yaw: " << yaw);

      const double eps = 0.02;
      const double eps_angle = 0.1;
      EXPECT_LE((start - expected_start).norm(), eps);
      EXPECT_LE(std::fabs(yaw - expected_yaw), eps_angle);
      EXPECT_LE((end - expected_end).norm(), eps);

    } else {
      for (const auto &m : msg->sub_messages) {
        EXPECT_LE(m.certainty, 0.2);
      }
    }

    finished = true;
  }
};

TEST_F(NoPassingZoneTest, testNoPassingZoneClassifier) {
  TesteeModule UNUSED lane_detection(nh, "/perception/lane_detection");
  TesteeModule UNUSED road_watcher(nh, "/perception/road_watcher");
  TesteeModule UNUSED preprocessing(nh, "/perception/preprocessing");
  TesteeModule UNUSED rosbag_nodebase(nh, "/common/rosbag_nodebase");

  const ros::Time start = ros::Time::now();
  do {
    ros::getGlobalCallbackQueue()->callAvailable(ros::WallDuration(0.1));
  } while (!finished && (ros::Time::now() - start) < timeout);
  EXPECT_TRUE(finished);
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "test_no_passing_zone");
  return RUN_ALL_TESTS();
}
