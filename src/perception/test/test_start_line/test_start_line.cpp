#include <common/macros.h>

THIRD_PARTY_HEADERS_BEGIN
#include <gtest/gtest.h>
#include <ros/callback_queue.h>
#include <ros/ros.h>
#include <tf2_eigen/tf2_eigen.h>
#include <boost/optional.hpp>
#include <memory>
#include <opencv2/imgproc.hpp>

#include "perception_msgs/StartLines.h"
THIRD_PARTY_HEADERS_END

#include "perception_types.h"
#include "common/testee_module.h"

DISABLE_SUGGEST_OVERRIDE_WARNING

class StartLineTest : public testing::Test {

 protected:
  ros::NodeHandle nh;

 private:
  ros::Subscriber start_line_subscriber;
  const bool expect_start_line;

  boost::optional<Eigen::Affine3d> getExpectedPoseIf(bool cond) {
    if (!cond) {
      return boost::none;
    }

    return Eigen::Affine3d(
        Eigen::Translation3d(nh.param("expected_x", 0.0), nh.param("expected_y", 0.0), 0) *
        Eigen::AngleAxisd(nh.param("expected_yaw", 0.0), Eigen::Vector3d::UnitZ()));
  }

 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  StartLineTest()
      : nh("~"),
        expect_start_line(nh.param("expect_start_line", true)),
        timeout(nh.param("timeout", 1.0)),
        expected_pose(getExpectedPoseIf(expect_start_line)) {}

 protected:
  const ros::Duration timeout;
  bool finished = false;
  const boost::optional<Eigen::Affine3d> expected_pose;

  // Test interface
  void SetUp() override {
    start_line_subscriber =
        nh.subscribe("startlines", 1, &StartLineTest::handleStartLine, this);
  }

 private:
  void handleStartLine(const perception_msgs::StartLinesConstPtr& msg) {
    ROS_INFO("Recieved Startline!");
    finished = true;

    if (!expect_start_line) {
      for (const auto& m : msg->sub_messages) {
        EXPECT_LE(m.certainty, 0.2);
      }
      return;
    }

    ASSERT_FALSE(msg->sub_messages.empty());
    const auto start_line = msg->sub_messages.front();

    Eigen::Affine3d start_line_pose;
    tf2::fromMsg(start_line.pose, start_line_pose);
    const Eigen::Vector3d angles = start_line_pose.rotation().eulerAngles(0, 1, 2);
    ROS_INFO_STREAM("position: " << start_line_pose.translation().x() << " "
                                 << start_line_pose.translation().y());
    ROS_INFO_STREAM("yaw: " << angles[2]);

    const double eps = 0.02;
    EXPECT_LE((start_line_pose.translation() - expected_pose->translation())
                  .array()
                  .abs()
                  .sum(),
              eps);
    EXPECT_LE(
        (start_line_pose.rotation() - expected_pose->rotation()).array().abs().sum(), eps);
  }
};

TEST_F(StartLineTest, testStartLineClassifier) {
  TesteeModule UNUSED lane_detection(nh, "/perception/lane_detection");
  TesteeModule UNUSED road_watcher(nh, "/perception/road_watcher");
  TesteeModule UNUSED preprocessing(nh, "/perception/preprocessing");
  TesteeModule UNUSED rosbag_nodebase(nh, "/common/rosbag_nodebase");

  const ros::Time start = ros::Time::now();
  do {
    ros::getGlobalCallbackQueue()->callAvailable(ros::WallDuration(0.1));
  } while (!finished && (ros::Time::now() - start) < timeout);
  EXPECT_EQ(true, finished);
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "test_start_line");
  return RUN_ALL_TESTS();
}
