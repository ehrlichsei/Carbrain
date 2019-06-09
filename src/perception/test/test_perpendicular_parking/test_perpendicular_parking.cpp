#include <common/macros.h>

THIRD_PARTY_HEADERS_BEGIN
#include <gtest/gtest.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <ros/callback_queue.h>
#include <ros/ros.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_ros/transform_listener.h>

#include <boost/algorithm/cxx11/any_of.hpp>

#include "perception_msgs/ParkingLotFound.h"
#include "perception_msgs/PerpendicularParkingSpots.h"
#include "perception_msgs/SearchParkingSpots.h"
THIRD_PARTY_HEADERS_END

#include "common/testee_module.h"
#include "perception_types.h"  //Custom Perception data structures

DISABLE_SUGGEST_OVERRIDE_WARNING

class ParkingTest : public testing::Test {

 public:
  ParkingTest()
      : nh("~"),
        timeout(nh.param("timeout", 1.0)),
        true_positive_test(nh.param("true_positive_test", true)),
        expected_nr_of_spots(nh.param("expected_nr_of_spots", 0)),
        target_frames(nh.param("frames", 0)),
        expected_detections(nh.param("expected_detections", 0)),
        time_sync_(parking_spots_sub_, indicator_sub_, 20) {

    time_sync_.registerCallback(boost::bind(&ParkingTest::handleParkingSpots, this, _1, _2));

    nh.getParam("expected_x_values", expected_poses_[0]);
    nh.getParam("expected_y_values", expected_poses_[1]);
    nh.getParam("expected_orientations", expected_poses_[2]);
    nh.getParam("expected_distances", expected_poses_[3]);
  }

 protected:
  std::vector<int> seen_ids_;
  ros::NodeHandle nh;
  const ros::Time start = ros::Time::now();
  std::size_t act_frames = 0, detections = 0;
  const ros::Duration timeout;
  const bool true_positive_test;
  const std::size_t expected_nr_of_spots;
  const std::size_t target_frames;
  const std::size_t expected_detections;
  // for the information about the location of a parking spot in a true positive
  // test
  std::array<std::vector<double>, 4> expected_poses_;
  bool finished = false;

  void SetUp() override {
    parking_spots_sub_.subscribe(nh, "free_parking_spots", 20);
    indicator_sub_.subscribe(nh, "parking_lot_found", 20);

    parking_lot_searcher_client_ = nh.serviceClient<perception_msgs::SearchParkingSpots>(
        "parking_service");
  }

  void searchForParkingSpots() {
    perception_msgs::SearchParkingSpots srv;
    srv.request.search = true;
    EXPECT_TRUE(parking_lot_searcher_client_.waitForExistence(ros::Duration(1.0)));
    if (parking_lot_searcher_client_.call(srv)) {
      ROS_INFO_THROTTLE(1, "Called search parking spot service!");
    } else {
      ROS_WARN_THROTTLE(1, "Failed to call parking service!");
    }
  }

 private:
  void handleParkingSpots(const perception_msgs::PerpendicularParkingSpotsConstPtr &parking_spots,
                          const perception_msgs::ParkingLotFoundConstPtr &indicator_msg) {
    ASSERT_EQ(act_frames, parking_spots->header.seq);
    ASSERT_EQ(act_frames, indicator_msg->header.seq);
    act_frames++;
    ROS_INFO("frame nr %zu", act_frames);
    if (true_positive_test) {
      ROS_INFO("True Test");
      if (!parking_spots->sub_messages.empty()) {
        ASSERT_TRUE(indicator_msg->found.data);
        std::size_t received_spots = 0;
        detections++;
        for (const auto &parking_spot : parking_spots->sub_messages) {

          const int actual_id = parking_spot.id.data;

          if (!boost::algorithm::any_of_equal(seen_ids_, actual_id)) {
            seen_ids_.emplace_back(actual_id);
          }
          const Eigen::Affine3d expected_left_entrance_pose =
              Eigen::Affine3d{Eigen::Translation3d{expected_poses_[0][received_spots],
                                                   expected_poses_[1][received_spots],
                                                   0.0} *
                              Eigen::AngleAxisd{expected_poses_[2][received_spots],
                                                -Eigen::Vector3d::UnitZ()}};

          Eigen::Affine3d left_entrance;
          Eigen::Affine3d right_entrance;
          // get pose
          tf2::fromMsg(parking_spot.entrance_pose_left, left_entrance);
          tf2::fromMsg(parking_spot.entrance_pose_right, right_entrance);

          ROS_INFO_STREAM("position left: (" << left_entrance.translation().x() << ","
                                             << left_entrance.translation().y() << ")");

          ROS_INFO_STREAM("orientation left: "
                          << left_entrance.linear().eulerAngles(0, 2, 0).y());
          ROS_INFO_STREAM("orientation right: "
                          << right_entrance.linear().eulerAngles(0, 2, 0).y());
          ROS_INFO_STREAM("distance: " << (left_entrance.translation() -
                                           right_entrance.translation())
                                              .norm());

          const double eps = 0.025;
          EXPECT_LE((left_entrance.translation() - expected_left_entrance_pose.translation())
                        .array()
                        .abs()
                        .sum(),
                    eps);

          EXPECT_LE(
              std::fabs(
                  (left_entrance.translation() - right_entrance.translation()).norm() -
                  expected_poses_[3][received_spots]),
              eps);
          const double angle_eps = 0.12;
          EXPECT_LE((left_entrance.rotation() - expected_left_entrance_pose.rotation())
                        .array()
                        .abs()
                        .sum(),
                    angle_eps);

          EXPECT_LE((right_entrance.rotation() - expected_left_entrance_pose.rotation())
                        .array()
                        .abs()
                        .sum(),
                    angle_eps);

          received_spots++;
        }
      }
    } else {
      ROS_INFO("False Test");
      ASSERT_FALSE(indicator_msg->found.data);
      ASSERT_TRUE(parking_spots->sub_messages.empty());
      if (act_frames >= target_frames) {
        finished = true;
      }
    }

    ROS_INFO("nr of complete spots is %zu", seen_ids_.size());
    if (act_frames >= target_frames && true_positive_test) {
      finished = true;
      ROS_INFO("whole nr of detections is %zu", detections);
      EXPECT_GE(detections, expected_detections);
      EXPECT_GE(seen_ids_.size(), expected_nr_of_spots);
    }
  }

  // member attributes
  message_filters::Subscriber<perception_msgs::PerpendicularParkingSpots> parking_spots_sub_;
  message_filters::Subscriber<perception_msgs::ParkingLotFound> indicator_sub_;
  message_filters::TimeSynchronizer<perception_msgs::PerpendicularParkingSpots, perception_msgs::ParkingLotFound> time_sync_;
  ros::ServiceClient parking_lot_searcher_client_;
};

TEST_F(ParkingTest, testPerpendicularParking) {
  TesteeModule UNUSED perpendicular_parking(
      nh, "/perception/perpendicular_parking");
  TesteeModule UNUSED lane_detection(nh, "/perception/lane_detection");
  TesteeModule UNUSED preprocessing(nh, "/perception/preprocessing");

  searchForParkingSpots();
  TesteeModule UNUSED rosbag_nodebase(nh, "/common/rosbag_nodebase");

  do {
    // activate parking
    // wait for callback
    ros::getGlobalCallbackQueue()->callAvailable(ros::WallDuration(0.1));
  } while (!finished && (ros::Time::now() - start) < timeout);
  EXPECT_TRUE(finished) << "expected frames: " << target_frames
                        << "  received frames: " << act_frames;
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "test_parkinglot_start_classifier");
  return RUN_ALL_TESTS();
}
