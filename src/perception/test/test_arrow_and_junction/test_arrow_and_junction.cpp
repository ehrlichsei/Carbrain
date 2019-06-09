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

#include "perception_types.h"
#include "common/testee_module.h"

DISABLE_SUGGEST_OVERRIDE_WARNING

class ArrowAndJunctionTest : public testing::Test {

 protected:
  ros::NodeHandle nh;

 public:
  std::vector<ros::Subscriber> road_objects_subscribers;

 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  ArrowAndJunctionTest()
      : nh("~"),
        timeout(nh.param("timeout", 1.0)),
        arrow_direction(nh.param("arrow_direction", -1)),
        arrow_min_num_positives(nh.param("arrow_min_num_positives", 10)),
        junction_min_num_positives(nh.param("junction_min_num_positives", 10)),
        junction_type(nh.param("junction_type", -1)),
        n_images(nh.param("n_images", -1)),
        max_allowed_outliers(nh.param("max_allowed_outliers", 0)),
        minimal_score(nh.param("minimal_score", 0.2)) {
    EXPECT_TRUE(nh.hasParam("timeout"));
    EXPECT_TRUE(nh.hasParam("arrow_direction"));
    EXPECT_TRUE(nh.hasParam("arrow_min_num_positives"));
    EXPECT_TRUE(nh.hasParam("junction_min_num_positives"));
    EXPECT_TRUE(nh.hasParam("junction_type"));
    EXPECT_TRUE(nh.hasParam("n_images"));
    EXPECT_TRUE(nh.hasParam("max_allowed_outliers"));
  }

  void checkPubishers() {
    // otherwise the wiring is incorrect
    for (const auto &sub : road_objects_subscribers) {
      if (sub.getNumPublishers() == 0) {
        ROS_ERROR_STREAM("Nobody is publishing on topic " << sub.getTopic());
      }
      EXPECT_GT(sub.getNumPublishers(), 0);
    }
  }

  void checkArrow(const perception_msgs::ArrowMarkingsConstPtr &arrow_marking) {

    if (arrow_marking->sub_messages.empty())
      return;

    const auto checkMarking = [this](const auto &sm) {
      return sm.direction == static_cast<int>(this->arrow_direction) &&
             sm.certainty > this->minimal_score;
    };
    if (!boost::algorithm::any_of(arrow_marking->sub_messages, checkMarking)) {
      ++received_outliers;
      ROS_INFO("arrow outlayer;");
      return;
    }
    ++n_arrows_received;
  }

  void handleArrowMessage(const perception_msgs::ArrowMarkingsConstPtr &arrow_marking) {
    checkPubishers();

    checkArrow(arrow_marking);

    ++n_messages_received;

    ROS_INFO("%d images received so far.", n_messages_received);

    if (n_messages_received >= n_images) {
      finished = true;
    }
  }

  void handleJunction(const perception_msgs::JunctionsConstPtr &junction_msg) {
    checkPubishers();

    if (junction_msg->sub_messages.empty())
      return;

    const auto checkJunction = [this](const auto &sm) {
      return sm.junction_type == static_cast<int>(this->junction_type) &&
             sm.certainty > this->minimal_score;
    };
    if (!boost::algorithm::any_of(junction_msg->sub_messages, checkJunction)) {
      ++received_outliers;
      return;
    }

    ++n_junctions_received;

    //    if (n_junctions_received > junction_min_num_positives) {
    //      finished = true;
    //    }
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
  const unsigned int arrow_direction;
  const unsigned int arrow_min_num_positives;
  const unsigned int junction_min_num_positives;
  const unsigned int junction_type;
  const unsigned int n_images;
  const unsigned int max_allowed_outliers;
  const float minimal_score;

  bool finished = false;

  // flag indicating received message (should occur at least once)
  unsigned int n_arrows_received = 0;
  unsigned int n_junctions_received = 0;

  unsigned int n_messages_received = 0;

  unsigned int received_outliers = 0;

  // Test interface
  void SetUp() override {
    using namespace perception_msgs;

    road_objects_subscribers = {
        nh.subscribe("arrow_marking", 1, &ArrowAndJunctionTest::handleArrowMessage, this),
        // those that should fail
        nh.subscribe("speed_limit_marking",
                     1,
                     &ArrowAndJunctionTest::handleFailCallback<SpeedLimitMarkings>,
                     this),
        nh.subscribe("crosswalk", 1, &ArrowAndJunctionTest::handleFailCallback<Crosswalks>, this),
        // junction is allowed directly after the arrow
        nh.subscribe("junction", 1, &ArrowAndJunctionTest::handleJunction, this),
        nh.subscribe("obstacle", 1, &ArrowAndJunctionTest::handleFailCallback<Obstacles>, this)};
  }
};

TEST_F(ArrowAndJunctionTest, testContourClassifier) {
  TesteeModule UNUSED lane_detection(nh, "/perception/lane_detection");
  TesteeModule UNUSED road_watcher(nh, "/perception/road_watcher");
  TesteeModule UNUSED preprocessing(nh, "/perception/preprocessing");
  TesteeModule UNUSED rosbag_nodebase(nh, "/common/rosbag_nodebase");

  const ros::Time start = ros::Time::now();
  do {
    ros::getGlobalCallbackQueue()->callAvailable(ros::WallDuration(0.1));
  } while ((ros::Time::now() - start) < timeout && !finished);
  ROS_INFO("Received %i arrow detections", n_arrows_received);
  ROS_INFO("Received %i junction detections", n_junctions_received);
  ROS_INFO("Received %i outliers", received_outliers);

  EXPECT_GE(n_arrows_received, arrow_min_num_positives);
  EXPECT_GE(n_junctions_received, junction_min_num_positives);

  EXPECT_LE(received_outliers, max_allowed_outliers);
  EXPECT_TRUE(finished);
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "test_arrow");
  return RUN_ALL_TESTS();
}
// Vorgehen: Ein Rosbag mit ner RoadClosure aufnehmen, deren Pose ausmessen, das
// in ein yaml-File speichern und in
// /launch/tests ablegen. Dann ein generelles test-file schreiben, für das man
// dann noch in einem launch file die konkreten werte spezifiziert
