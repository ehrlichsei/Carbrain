#include <common/macros.h>

THIRD_PARTY_HEADERS_BEGIN
#include <gtest/gtest.h>
#include <boost/optional.hpp>
#include <ros/ros.h>
#include <ros/callback_queue.h>

#include <memory>
#include <opencv2/imgproc.hpp>

#include "perception_msgs/Obstacles.h"
#include "perception_msgs/LookAt.h"
THIRD_PARTY_HEADERS_END

#include "perception_types.h"
#include "common/testee_module.h"

DISABLE_SUGGEST_OVERRIDE_WARNING

class RoadWatcherTest : public testing::Test {

 protected:
  ros::NodeHandle nh;

 private:
  ros::Subscriber road_object_subscriber;
  const bool expect_obstacle;

  boost::optional<Eigen::Affine3d> getExpectedPoseIf(bool cond) {
    if (!cond) {
      return boost::none;
    }

    return Eigen::Affine3d(
        Eigen::Translation3d(nh.param("expected_x", 0.0), nh.param("expected_y", 0.0), 0) *
        Eigen::AngleAxisd(nh.param("expected_yaw", 0.0), Eigen::Vector3d::UnitZ()));
  }

  boost::optional<Eigen::Vector2d> getExpectedSizeIf(bool cond) {
    if (!cond) {
      return boost::none;
    }

    return Eigen::Vector2d(nh.param("expected_size_x", 0.0),
                           nh.param("expected_size_y", 0.0));
  }

 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  RoadWatcherTest()
      : nh("~"),
        expect_obstacle(nh.param("expect_obstacle", true)),
        timeout(nh.param("timeout", 1.0)),
        expected_pose(getExpectedPoseIf(expect_obstacle)),
        expected_size(getExpectedSizeIf(expect_obstacle)) {}

 protected:
  const ros::Duration timeout;
  bool finished = false;
  const boost::optional<Eigen::Affine3d> expected_pose;
  const boost::optional<Eigen::Vector2d> expected_size;

  // Test interface
  void SetUp() override {
    road_object_subscriber =
        nh.subscribe("obstacles", 1, &RoadWatcherTest::handleObstacle, this);
  }

 private:
  void handleObstacle(const perception_msgs::ObstaclesConstPtr& msg) {
    ROS_INFO("Recieved Obstacle!");
    finished = true;

    if (!expect_obstacle) {
      ASSERT_TRUE(msg->sub_messages.empty());
      return;
    }

    ASSERT_FALSE(msg->sub_messages.empty());
    const auto& obstacle = msg->sub_messages.front();

    std::vector<cv::Point2f> vertices;
    for (const auto& p_msg : obstacle.vertices) {
      vertices.emplace_back(p_msg.x, p_msg.y);
    }

    const cv::RotatedRect rotated_rect = cv::minAreaRect(vertices);

    const Eigen::Vector2d obst_size(rotated_rect.size.width, rotated_rect.size.height);

    const Eigen::Affine3d obstacle_pose = Eigen::Affine3d(
        Eigen::Translation3d(rotated_rect.center.x, rotated_rect.center.y, 0.0) *
        Eigen::AngleAxisd(rotated_rect.angle * 2.f * M_PI / 360.f, Eigen::Vector3d::UnitZ()));

    // EXPECT_LE((obstacle_pose - expected_pose).abs().sum(), 0.001);
    ROS_INFO_STREAM("position: " << obstacle_pose.translation().x() << " "
                                 << obstacle_pose.translation().y());
    ROS_INFO_STREAM("yaw: " << rotated_rect.angle * 2.f * M_PI / 360.f);
    ROS_INFO_STREAM("size: " << obst_size(0) << " " << obst_size(1));
    const double eps = 0.045;
    const double yaw_eps = 0.085;
    EXPECT_LE(
        (obstacle_pose.translation() - expected_pose->translation()).array().abs().sum(), eps);
    EXPECT_LE((obstacle_pose.rotation() - expected_pose->rotation()).array().abs().sum(), yaw_eps);
    EXPECT_LE((obst_size - *expected_size).array().abs().sum(), eps);
  }
};

TEST_F(RoadWatcherTest, testRoadWatcherObstacleDetection) {
  TesteeModule UNUSED lane_detection(nh, "/perception/lane_detection");
  TesteeModule UNUSED road_watcher(nh, "/perception/road_watcher");
  TesteeModule UNUSED preprocessing(nh, "/perception/preprocessing");

  if (nh.param("use_look_at", false)) {
    ros::ServiceClient s = nh.serviceClient<perception_msgs::LookAt>(
        "/perception/road_watcher/look_at");
    perception_msgs::LookAt::Request rq;
    rq.do_look = true;
    rq.pose.header.frame_id = "vehicle";
    rq.pose.pose.position.x = nh.param("roi/x", 0.0);
    rq.pose.pose.position.y = nh.param("roi/y", 0.0);
    rq.pose.pose.orientation.w = 1;
    rq.rect.x = nh.param("roi/rect/x", 0.0);
    rq.rect.y = nh.param("roi/rect/y", 0.0);
    perception_msgs::LookAt::Response rs;
    ASSERT_TRUE(s.waitForExistence(ros::Duration(2)));
    ASSERT_TRUE(s.call(rq, rs));
  }

  TesteeModule UNUSED rosbag_nodebase(nh, "/common/rosbag_nodebase");

  const ros::Time start = ros::Time::now();
  do {
    ros::getGlobalCallbackQueue()->callAvailable(ros::WallDuration(0.1));
  } while (!finished && (ros::Time::now() - start) < timeout);
  EXPECT_EQ(true, finished);
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "test_road_watcher");
  return RUN_ALL_TESTS();
}
