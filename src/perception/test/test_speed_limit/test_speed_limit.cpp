#include <common/macros.h>

THIRD_PARTY_HEADERS_BEGIN
#include <gtest/gtest.h>
#include <ros/callback_queue.h>
#include <ros/ros.h>
#include <rosconsole/macros_generated.h>
#include <boost/algorithm/cxx11/any_of.hpp>
#include <memory>
#include <opencv2/imgproc.hpp>

#include "perception_msgs/ArrowMarkings.h"
#include "perception_msgs/Crosswalks.h"
#include "perception_msgs/Junctions.h"
#include "perception_msgs/Obstacles.h"
#include "perception_msgs/RoadClosures.h"
#include "perception_msgs/SpeedLimitMarkings.h"
THIRD_PARTY_HEADERS_END

#include "common/testee_module.h"
#include "perception_types.h"

DISABLE_SUGGEST_OVERRIDE_WARNING

class SpeedLimitTest : public testing::Test {

 protected:
  ros::NodeHandle nh;

 public:
  std::vector<ros::Subscriber> road_objects_subscribers;

 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  SpeedLimitTest()
      : nh("~"),
        timeout(nh.param("timeout", 1.0)),
        speed_limit(nh.param("expected_speed_limit", 100)),
        is_relieved(nh.param("expected_is_relieved", true)),
        min_number_positives(nh.param("min_number_positives", 10)),
        max_allowed_outliers(nh.param("max_allowed_outliers", 0)),
        minimal_score(nh.param("minimal_score", 0.5)) {
    EXPECT_TRUE(nh.hasParam("timeout"));
    EXPECT_TRUE(nh.hasParam("expected_speed_limit"));
    EXPECT_TRUE(nh.hasParam("expected_is_relieved"));
    EXPECT_TRUE(nh.hasParam("min_number_positives"));
    EXPECT_TRUE(nh.hasParam("max_allowed_outliers"));
  }

  void checkPublishers() {
    // otherwise the wiring is incorrect
    for (const auto &sub : road_objects_subscribers) {
      if (sub.getNumPublishers() == 0) {
        ROS_ERROR_STREAM("Nobody is publishing on topic " << sub.getTopic());
      }
      EXPECT_GT(sub.getNumPublishers(), 0);
    }
  }

  void handleSpeedLimit(const perception_msgs::SpeedLimitMarkingsConstPtr &speed_limits) {
    checkPublishers();

    if (speed_limits->sub_messages.empty()) {
      ROS_INFO("Message is empty");
      return;
    }

    const auto check = [this](const auto &sm) {
      return sm.speed_limit == static_cast<int>(this->speed_limit) &&
             sm.certainty > this->minimal_score;
    };

    if (speed_limit != 0) {  // 0 means arbitrary
      if (!boost::algorithm::any_of(speed_limits->sub_messages, check)) {
        ++received_outliers;
        return;
      }
    }

    if (speed_limits->sub_messages[0].limit_relieved != is_relieved) {
      ++received_outliers;
      return;
    }
    ++n_received;

    if (n_received > min_number_positives) {
      finished = true;
    }
  }

  template <typename T>
  void handleFailCallback(const boost::shared_ptr<const T> &message) {
    for (const auto &m : message->sub_messages) {
      EXPECT_LE(m.certainty, 0.2);
    }
  }

 protected:
  const ros::Duration timeout;

  // expected values
  const unsigned int speed_limit;
  const bool is_relieved;
  const unsigned int min_number_positives;
  const unsigned int max_allowed_outliers;
  const float minimal_score;

  bool finished = false;

  // flag indicating received message (should occur at least once)
  unsigned int n_received = 0;

  unsigned int received_outliers = 0;

  // Test interface
  void SetUp() override {
    using namespace perception_msgs;

    road_objects_subscribers = {
        nh.subscribe("speed_limit_marking", 1, &SpeedLimitTest::handleSpeedLimit, this),
        // those that should fail
        nh.subscribe("arrow_marking", 1, &SpeedLimitTest::handleFailCallback<ArrowMarkings>, this),
        nh.subscribe("crosswalk", 1, &SpeedLimitTest::handleFailCallback<Crosswalks>, this),
        nh.subscribe("junction", 1, &SpeedLimitTest::handleFailCallback<Junctions>, this),
        nh.subscribe("obstacle", 1, &SpeedLimitTest::handleFailCallback<Obstacles>, this),
    };
  }
};

TEST_F(SpeedLimitTest, testContourClassifier) {
  TesteeModule UNUSED lane_detection(nh, "/perception/lane_detection");
  TesteeModule UNUSED road_watcher(nh, "/perception/road_watcher");
  TesteeModule UNUSED preprocessing(nh, "/perception/preprocessing");
  TesteeModule UNUSED rosbag_nodebase(nh, "/common/rosbag_nodebase");

  const ros::Time start = ros::Time::now();
  do {
    ros::getGlobalCallbackQueue()->callAvailable(ros::WallDuration(0.1));

    ROS_INFO_THROTTLE(1, "Received %d / %d speed limits", n_received, min_number_positives);

  } while ((ros::Time::now() - start) < timeout && !finished);

  ROS_INFO("Received %i speed limit detections", n_received);
  ROS_INFO("Received %i outliers", received_outliers);

  EXPECT_GE(n_received, min_number_positives);
  EXPECT_LE(received_outliers, max_allowed_outliers);
  EXPECT_TRUE(finished);
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "test_speed_limit");
  return RUN_ALL_TESTS();
}
