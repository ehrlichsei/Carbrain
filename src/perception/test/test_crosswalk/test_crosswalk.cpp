#include <common/macros.h>

THIRD_PARTY_HEADERS_BEGIN
#include <gtest/gtest.h>
#include <ros/callback_queue.h>
#include <ros/ros.h>
#include <rosconsole/macros_generated.h>
#include <tf2_eigen/tf2_eigen.h>
#include <memory>
#include <opencv2/imgproc.hpp>

#include "perception_msgs/Crosswalks.h"
THIRD_PARTY_HEADERS_END

#include "common/angle_conversions.h"
#include "common/testee_module.h"
#include "perception_types.h"

DISABLE_SUGGEST_OVERRIDE_WARNING

class CrosswalkTest : public testing::Test {

 protected:
  ros::NodeHandle nh;

 private:
  ros::Subscriber crosswalk_subscriber;

 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  CrosswalkTest()
      : nh("~"),
        timeout(nh.param("timeout", 1.0)),
        expected_pose(
            Eigen::Translation3d(
                nh.param("expected_x_pose", 0.0), nh.param("expected_y_pose", 0.0), 0) *
            Eigen::AngleAxisd(nh.param("expected_yaw", 0.0), Eigen::Vector3d::UnitZ())),
        nr_of_frames(nh.param("nr_of_frames", 1)),
        frames_received(0),
        true_positive_test(nh.param("true_positive_test", true)) {}


 protected:
  const ros::Duration timeout;
  bool finished = false;
  const Eigen::Affine3d expected_pose;
  const uint nr_of_frames;
  uint frames_received;
  bool true_positive_test;

  // Test interface
  void SetUp() override {
    crosswalk_subscriber =
        nh.subscribe("crosswalks", 1, &CrosswalkTest::handleCrosswalk, this);
  }

 private:
  void handleCrosswalk(const perception_msgs::CrosswalksConstPtr& msg) {
    if (true_positive_test) {
      ROS_INFO("Received Crosswalk!");
      ASSERT_TRUE(!msg->sub_messages.empty());

      frames_received++;

      if (frames_received >= nr_of_frames) {
        const auto& crosswalk = msg->sub_messages.front();

        Eigen::Affine3d crosswalk_pose;

        tf2::fromMsg(crosswalk.pose, crosswalk_pose);


        double yaw = common::toYaw(crosswalk_pose.rotation());
        double expected_yaw = nh.param("expected_yaw", 0.0);


        ROS_INFO_STREAM("position: " << crosswalk_pose.translation().x() << " "
                                     << crosswalk_pose.translation().y());
        ROS_INFO_STREAM("yaw: " << yaw);
        const double eps_position = 0.035;
        const double eps_yaw = 0.08;

        EXPECT_LE(
            (crosswalk_pose.translation() - expected_pose.translation()).array().abs().sum(),
            eps_position);
        EXPECT_LE(std::fabs(yaw - expected_yaw), eps_yaw);
        finished = true;
      } else {
        EXPECT_FALSE(finished);
      }
    } else {
      for (const auto& m : msg->sub_messages) {
        EXPECT_LE(m.certainty, 0.2);
      }
      finished = true;
    }
  }
};

TEST_F(CrosswalkTest, testCrosswalkClassifier) {
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

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "test_crosswalk");
  return RUN_ALL_TESTS();
}
