#include <common/macros.h>

THIRD_PARTY_HEADERS_BEGIN
#include <gtest/gtest.h>
#include <ros/callback_queue.h>
#include <ros/ros.h>
#include <rosconsole/macros_generated.h>
#include <memory>
#include <opencv2/imgproc.hpp>

#include "perception_msgs/RoadClosures.h"
THIRD_PARTY_HEADERS_END

#include "perception_types.h"
#include "common/testee_module.h"

DISABLE_SUGGEST_OVERRIDE_WARNING

class RoadClosureTest : public testing::Test {

 protected:
  ros::NodeHandle nh;

 private:
  ros::Subscriber road_closure_subscriber;

 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  RoadClosureTest()
      : nh("~"),
        timeout(nh.param("timeout", 1.0)),
        expected_pose(Eigen::Translation3d(nh.param("expected_x_rect_center", 0.0),
                                           nh.param("expected_y_rect_center", 0.0),
                                           0) *
                      Eigen::AngleAxisd(nh.param("expected_yaw", 0.0),
                                        Eigen::Vector3d::UnitZ())),
        expected_size(nh.param("expected_size_x", 0.0),
                      nh.param("expected_size_y", 0.0)),
        expect_road_closure(nh.param("expect_road_closure", true)),
        message_count(0),
        min_number_msgs(nh.param("min_number_msgs", 1)) {}


  // expected yaw ist der Winkel zwischen RoadClosure und dem Auto
  // (features.cluster_center_lane_orientation),
  // expected_x_rect_center, expected_y_rect_center sind die Koordinaten der
  // Mittelpunkte des bounding rects um hull_polygon
  // expected size x und y sind die x- und y-Werte des bounding rects um die
  // RoadClosure
 protected:
  const ros::Duration timeout;
  bool finished = false;
  const Eigen::Affine3d expected_pose;
  const Eigen::Vector2d expected_size;
  const bool expect_road_closure;
  int message_count;
  const int min_number_msgs;

  // Test interface
  void SetUp() override {
    road_closure_subscriber =
        nh.subscribe("road_closures", 1, &RoadClosureTest::handleRoadClosure, this);
  }

 private:
  void handleRoadClosure(const perception_msgs::RoadClosuresConstPtr& msg) {
    ROS_INFO("expect road closure is %d", expect_road_closure);
    if (expect_road_closure) {
      ROS_INFO("Received RoadClosure!");
      ASSERT_TRUE(!msg->sub_messages.empty());
      const auto& road_closure = msg->sub_messages.front();

      std::vector<cv::Point2f> hull_polygon;
      for (const auto& corner : road_closure.hull_polygon) {
        hull_polygon.emplace_back(corner.x, corner.y);
      }
      const cv::RotatedRect bounding_rect = cv::minAreaRect(hull_polygon);

      const Eigen::Vector2d road_closure_size(bounding_rect.size.width,
                                              bounding_rect.size.height);

      const Eigen::Affine3d road_closure_pose = Eigen::Affine3d(
          Eigen::Translation3d(bounding_rect.center.x, bounding_rect.center.y, 0.0) *
          Eigen::AngleAxisd(bounding_rect.angle * 2.f * M_PI / 360.f,
                            Eigen::Vector3d::UnitZ()));


      ROS_INFO_STREAM("position: " << road_closure_pose.translation().x() << " "
                                   << road_closure_pose.translation().y());
      ROS_INFO_STREAM("yaw: " << bounding_rect.angle * 2.f * M_PI / 360.f);
      ROS_INFO_STREAM("size: " << road_closure_size(0) << " " << road_closure_size(1));
      const double eps = 0.03;
      const double eps_angle = 0.1;
      EXPECT_LE(
          (road_closure_pose.translation() - expected_pose.translation()).array().abs().sum(),
          eps);
      EXPECT_LE((road_closure_pose.rotation() - expected_pose.rotation()).array().abs().sum(),
                eps_angle);
      EXPECT_LE((road_closure_size - expected_size).array().abs().sum(), eps);
      finished = true;
    } else {
      for (const auto& m : msg->sub_messages) {
        EXPECT_LE(m.certainty, 0.2);
      }
      message_count++;
      ROS_INFO("%d messages received", message_count);
      if (message_count >= min_number_msgs) {
        finished = true;
      }
    }
  }
};

TEST_F(RoadClosureTest, testRoadClosureClassifier) {
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
  ros::init(argc, argv, "test_road_closure");
  return RUN_ALL_TESTS();
}
// Vorgehen: Ein Rosbag mit ner RoadClosure aufnehmen, deren Pose ausmessen, das
// in ein yaml-File speichern und in
// /launch/tests ablegen. Dann ein generelles test-file schreiben, für das man
// dann noch in einem launch file die konkreten werte spezifiziert
