#include <common/macros.h>

THIRD_PARTY_HEADERS_BEGIN
#include <actionlib/client/simple_action_client.h>
#include <gtest/gtest.h>
#include <perception_msgs/LookForObstaclesAction.h>
#include <perception_msgs/Obstacles.h>
#include <ros/ros.h>
#include <tf2_eigen/tf2_eigen.h>
#include <opencv2/imgproc.hpp>
THIRD_PARTY_HEADERS_END

#include "common/angle_conversions.h"
#include "common/testee_module.h"
#include "perception_types.h"

DISABLE_SUGGEST_OVERRIDE_WARNING

class LookAtObstaclesTest : public ::testing::Test {

 public:
  LookAtObstaclesTest()
      : nh("~"),
        timeout(nh.param("timeout", 1.0)),
        nr_frames(nh.param("frames", 0)),
        roi_position_x(nh.param("roi_position_x", 0.0)),
        roi_position_y(nh.param("roi_position_y", 0.0)),
        roi_orientation(nh.param("roi_orientation", 0.0)),
        roi_width(nh.param("roi_width", 0.0)),
        roi_height(nh.param("roi_height", 0.0)),
        expected_position_x(nh.param("expected_position_x", 0.0)),
        expected_position_y(nh.param("expected_position_y", 0.0)),
        expected_orientation(nh.param("expected_orientation", 0.0)),
        obstacles_look_at_client("look_for_obstacles", true) {}

 protected:
  ros::NodeHandle nh;
  const ros::Time start = ros::Time::now();
  const ros::Duration timeout;
  const std::size_t nr_frames;
  bool finished = false;
  std::size_t nr_empty_results = 0;
  std::size_t frame_count = 0;

  const double roi_position_x, roi_position_y;
  const double roi_orientation;

  const double roi_width, roi_height;

  const double expected_position_x, expected_position_y;
  const double expected_orientation;

  actionlib::SimpleActionClient<perception_msgs::LookForObstaclesAction> obstacles_look_at_client;

  perception_msgs::LookForObstaclesGoal rois;

  void sendGoal() {

    ROS_INFO("Waiting for server to connect.");
    obstacles_look_at_client.waitForServer();

    ROS_INFO("Connected to server!");

    // construct roi
    WorldPose cw_pose_ = Eigen::Translation3d{roi_position_x, roi_position_y, 0} *
                         Eigen::AngleAxisd{roi_orientation, WorldPoint::UnitZ()};

    perception_msgs::LookAtRegion roi;

    roi.pose = tf2::toMsg(cw_pose_);
    roi.rect.x = roi_height;
    roi.rect.y = roi_width;
    roi.id = 0;

    rois.obstacle_regions.regions.push_back(roi);

    // send goal
    obstacles_look_at_client.sendGoal(
        rois,
        boost::bind(&LookAtObstaclesTest::checkResult, this, _1, _2),
        boost::bind(&LookAtObstaclesTest::lookAtActive, this),
        boost::bind(&LookAtObstaclesTest::lookAtFeedback, this, _1));
  }

 private:
  void checkResult(const actionlib::SimpleClientGoalState & /*state*/,
                   const perception_msgs::LookForObstaclesResult::ConstPtr &result) {


    EXPECT_NE(result, nullptr);

    frame_count++;

    // check result
    ROS_INFO("recieved result!");

    ASSERT_FALSE(result->obstacles.sub_messages.empty());

    if (!result->obstacles.sub_messages.empty()) {
      const auto obstacle = result->obstacles.sub_messages.front();
      ASSERT_GT(obstacle.vertices.size(), 0);
      std::vector<cv::Point2f> vertices;
      for (const auto &p_msg : obstacle.vertices) {
        vertices.emplace_back(p_msg.x, p_msg.y);
      }
      std::cout << "vertices: " << vertices.size() << std::endl;
      const cv::RotatedRect rotated_rect = cv::minAreaRect(vertices);

      const Eigen::Vector2d obst_size(rotated_rect.size.width, rotated_rect.size.height);

      const Eigen::Affine3d obstacle_pose = Eigen::Affine3d(
          Eigen::Translation3d(rotated_rect.center.x, rotated_rect.center.y, 0.0) *
          Eigen::AngleAxisd(rotated_rect.angle * 2.f * M_PI / 360.f,
                            Eigen::Vector3d::UnitZ()));

      const auto rotation = common::toYaw(obstacle_pose.rotation());

      const auto position_x = obstacle_pose.translation().x();
      const auto position_y = obstacle_pose.translation().y();

      ROS_INFO("recieved obstacle at position (%f,%f), with orientation %f",
               position_x,
               position_y,
               rotation);
      const auto eps_pos = 0.02;
      const auto eps_angle = 0.05;

      EXPECT_LE(std::fabs(expected_position_x - position_x), eps_pos);
      EXPECT_LE(std::fabs(expected_position_y - position_y), eps_pos);
      EXPECT_LE(std::fabs(expected_orientation - rotation), eps_angle);
    }

    if (frame_count >= nr_frames) {
      finished = true;
    }
  }

  void lookAtActive() {}

  void lookAtFeedback(const perception_msgs::LookForObstaclesFeedback::ConstPtr & /*feedback*/) {
  }
};

TEST_F(LookAtObstaclesTest, testLookAtObstacles) {
  TesteeModule UNUSED look_at(nh, "/perception/look_at");
  TesteeModule UNUSED lane_detection(nh, "/perception/lane_detection");
  TesteeModule UNUSED preprocessing(nh, "/perception/preprocessing");
  TesteeModule UNUSED rosbag_nodebase(nh, "/common/rosbag_nodebase");

  sendGoal();

  do {
    ros::getGlobalCallbackQueue()->callAvailable(ros::WallDuration(0.1));
  } while (!finished && (ros::Time::now() - start) < timeout);

  EXPECT_EQ(true, finished);
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "test_look_at_obstacles");
  return RUN_ALL_TESTS();
}
