#include <common/macros.h>

THIRD_PARTY_HEADERS_BEGIN
#include <gtest/gtest.h>
#include <ros/callback_queue.h>
#include <ros/ros.h>
#include <tf2_eigen/tf2_eigen.h>
#include <boost/optional.hpp>
#include <memory>
#include <opencv2/imgproc.hpp>

#include "perception_msgs/Junctions.h"
THIRD_PARTY_HEADERS_END

#include "perception_types.h"
#include "common/testee_module.h"

DISABLE_SUGGEST_OVERRIDE_WARNING

class JunctionTest : public testing::Test {

 protected:
  ros::NodeHandle nh;

 private:
  ros::Subscriber junction_subscriber;
  const bool expect_junction;

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
  JunctionTest()
      : nh("~"),
        expect_junction(nh.param("expect_junction", true)),
        timeout(nh.param("timeout", 1.0)),
        expected_pose(getExpectedPoseIf(expect_junction)) {}

 protected:
  const ros::Duration timeout;
  bool finished = false;
  const boost::optional<Eigen::Affine3d> expected_pose;

  // Test interface
  void SetUp() override {
    junction_subscriber = nh.subscribe("junction", 1, &JunctionTest::handleJunction, this);
  }

 private:
  void handleJunction(const perception_msgs::JunctionsConstPtr& msg) {
    ROS_INFO("Received Junction!");
    finished = true;

    if (!expect_junction) {
      for (const auto& m : msg->sub_messages) {
        EXPECT_LE(m.certainty, 0.2);
      }
      return;
    }

    ASSERT_FALSE(msg->sub_messages.empty());
    const auto junction = msg->sub_messages.front();

    Eigen::Affine3d junction_pose;
    tf2::fromMsg(junction.pose, junction_pose);
    const Eigen::Vector3d angles = junction_pose.rotation().eulerAngles(0, 1, 2);
    ROS_INFO_STREAM("position: " << junction_pose.translation().x() << " "
                                 << junction_pose.translation().y());
    ROS_INFO_STREAM("yaw: " << angles[2]);

    const double eps = 0.02;
    EXPECT_LE(
        (junction_pose.translation() - expected_pose->translation()).array().abs().sum(), eps);
    EXPECT_LE((junction_pose.rotation() - expected_pose->rotation()).array().abs().sum(), eps);
  }
};

TEST_F(JunctionTest, testStartLineClassifier) {
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
