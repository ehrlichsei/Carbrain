#include <common/macros.h>

THIRD_PARTY_HEADERS_BEGIN
#include <gtest/gtest.h>
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <numeric>
#include <iostream>
#include <fstream>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <eigen3/Eigen/Core>
#include <state_estimation_msgs/ProcessSensorData.h>

#include <controller_msgs/StateMeasure.h>
THIRD_PARTY_HEADERS_END

#include <common/realtime_channel_ids.h>
#include <common/realtimeipc.h>
#include <controller_interface/sensormeasurements.h>
#include "common/math.h"

DISABLE_SUGGEST_OVERRIDE_WARNING

using namespace Eigen;
using common::wrapAngleMinusPiToPi;

#define SPEED_OUTLIER_THRESHOLD 0.15f
#define YAW_RATE_OUTLIER_THRESHOLD 0.4f
#define YAW_RATE_OUTLIER_THRESHOLD_FAST 0.8f

typedef std::vector<controller_msgs::StateMeasure> Measures;
typedef std::vector<common_msgs::Float32Stamped> ServoCommands;
typedef std::vector<state_estimation_msgs::State> States;

class RosbagTest : public testing::Test {
 public:
  RosbagTest(std::string rosbag_name, int update_rate)
      : update_rate(update_rate), rosbag_name(rosbag_name) {}

 protected:
  virtual void SetUp() override {
    ros::NodeHandle nh("~");

    state_estimation_msgs::ProcessSensorDataRequest request;
    std::tie(request.measures,
             request.front_steering_servo_commands,
             request.back_steering_servo_commands) = readRosbag();
    request.rate = update_rate;

    ros::ServiceClient state_estimation_client =
        nh.serviceClient<state_estimation_msgs::ProcessSensorData>(
            "/state_estimation/state_estimation/process_sensor_data");
    bool state_estimation_service_exists =
        state_estimation_client.waitForExistence(ros::Duration(5));
    ASSERT_TRUE(state_estimation_service_exists);

    state_estimation_msgs::ProcessSensorDataResponse response;
    bool call_return_value = state_estimation_client.call(request, response);
    ASSERT_TRUE(call_return_value);
    intermediate_states = response.states;
  }

  States intermediate_states;
  int update_rate;

  // Helper methods
  std::vector<float> getSpeedEstimations(States& states) {
    std::vector<float> speed_estimations;
    speed_estimations.reserve(states.size());
    for (state_estimation_msgs::State state : states) {
      speed_estimations.push_back(state.speed_x);
    }
    return speed_estimations;
  }

  std::vector<float> getYawRateEstimations(States& states) {
    std::vector<float> yaw_rate_estimations;
    yaw_rate_estimations.reserve(states.size());
    for (state_estimation_msgs::State state : states) {
      yaw_rate_estimations.push_back(state.yaw_rate);
    }
    return yaw_rate_estimations;
  }

  void checkForOutliers(std::vector<float>& estimations, float threshold) {
    for (std::vector<float>::iterator estimation_iterator = estimations.begin() + 1;
         estimation_iterator != estimations.end();
         estimation_iterator++) {
      const float estimation = *estimation_iterator;
      const float last_estimation = *(estimation_iterator - 1);
      EXPECT_NEAR(estimation, last_estimation, threshold);
    }
  }

  void integrateIntermediateStatesToEndPose(const States& states,
                                            Eigen::Vector2d& end_position,
                                            double& end_orientation) {
    Eigen::Vector2d position = Eigen::Vector2d(0, 0);
    double orientation = 0.0;
    for (auto state : states) {
      position += Eigen::Vector2d(std::cos(orientation), std::sin(orientation)) *
                  state.speed_x * 1. / 120.0;
      orientation = wrapAngleMinusPiToPi(orientation + state.yaw_rate * 1. / 120.0);
    }
    end_position = position;
    end_orientation = orientation;
  }

  void outputCSV(std::string name, std::vector<float> data, std::string filename) {
    std::string path(".");
    ros::NodeHandle("~").getParam("csv_path", path);
    std::ofstream out(path + "/" + filename);
    out << name << std::endl;
    for_each(data.begin(),
             data.end(),
             [&out](float data_point) { out << data_point << std::endl; });
    out.close();
  }

 private:
  std::tuple<Measures, ServoCommands, ServoCommands> readRosbag() {
    ros::NodeHandle nh("~");
    rosbag::Bag bag;
    std::string rosbag_path;
    nh.getParam("rosbag_path", rosbag_path);
    bag.open(rosbag_path + rosbag_name, rosbag::bagmode::Read);

    const std::string front_servo_topic =
        "/control/front_steering_controller/debug/servo_set_value";
    const std::string back_servo_topic =
        "/control/back_steering_controller/debug/servo_set_value";
    rosbag::View view(
        bag,
        rosbag::TopicQuery(
            {"/controller_interface/controller_interface/state_measure", front_servo_topic, back_servo_topic}));

    Measures measures;
    ServoCommands front_servo_commands;
    ServoCommands back_servo_commands;
    for (auto message_iterator = view.begin(); message_iterator != view.end();
         message_iterator++) {
      rosbag::MessageInstance m = *message_iterator;

      controller_msgs::StateMeasurePtr state_measure =
          m.instantiate<controller_msgs::StateMeasure>();
      if (state_measure != std::nullptr_t()) {
        measures.push_back(*state_measure);
      } else {
        common_msgs::Float32StampedPtr servo_command =
            m.instantiate<common_msgs::Float32Stamped>();
        if (servo_command != std::nullptr_t()) {
          if (m.getTopic() == front_servo_topic) {
            front_servo_commands.push_back(*servo_command);
          } else if (m.getTopic() == back_servo_topic) {
            back_servo_commands.push_back(*servo_command);
          }
        }
      }
    }

    bag.close();
    if (measures.empty()) {
      ROS_ERROR("Empty measures!");
    } else {
      ros::Time start_time = measures.front().header.stamp;
      ros::Time end_time = measures.back().header.stamp;
      float average_rate = measures.size() / (end_time - start_time).toSec();
      EXPECT_NEAR(average_rate, update_rate, 2);
    }
    return std::make_tuple(measures, front_servo_commands, back_servo_commands);
  }

  std::string rosbag_name;
};

class SlowParkingRosbagTest : public RosbagTest {
 public:
  SlowParkingRosbagTest()
      : RosbagTest("/test_state_estimation/new_message_test_parking.bag", 120) {}
};

class StraightDriving05RosbagTest : public RosbagTest {
 public:
  StraightDriving05RosbagTest()
      : RosbagTest("/test_state_estimation/new_message_test_straight_0_5.bag", 120) {}
};

class StraightDriving10RosbagTest : public RosbagTest {
 public:
  StraightDriving10RosbagTest()
      : RosbagTest("/test_state_estimation/new_message_test_straight_1_0.bag", 120) {}
};

class StraightDriving15RosbagTest : public RosbagTest {
 public:
  StraightDriving15RosbagTest()
      : RosbagTest("/test_state_estimation/new_message_test_straight_1_5.bag", 120) {}
};

class StraightDriving20RosbagTest : public RosbagTest {
 public:
  StraightDriving20RosbagTest()
      : RosbagTest("/test_state_estimation/new_message_test_straight_2_0.bag", 120) {}
};

class StraightDriving25RosbagTest : public RosbagTest {
 public:
  StraightDriving25RosbagTest()
      : RosbagTest("/test_state_estimation/new_message_test_straight_2_5.bag", 120) {}
};

class StraightDriving30RosbagTest : public RosbagTest {
 public:
  StraightDriving30RosbagTest()
      : RosbagTest("/test_state_estimation/new_message_test_straight_3_0.bag", 120) {}
};

class OvalDrivingSlowRosbagTest : public RosbagTest {
 public:
  OvalDrivingSlowRosbagTest()
      : RosbagTest("/test_state_estimation/new_message_test_oval_slow.bag", 120) {}
};

class OvalDrivingMediumRosbagTest : public RosbagTest {
 public:
  OvalDrivingMediumRosbagTest()
      : RosbagTest("/test_state_estimation/new_message_test_oval_medium.bag", 120) {}
};

class OvalDrivingFastRosbagTest : public RosbagTest {
 public:
  OvalDrivingFastRosbagTest()
      : RosbagTest("/test_state_estimation/new_message_test_oval_fast.bag", 120) {}
};


TEST_F(SlowParkingRosbagTest, testSpeedEstimationOutliers) {
  std::vector<float> speed_estimations = getSpeedEstimations(intermediate_states);
  checkForOutliers(speed_estimations, SPEED_OUTLIER_THRESHOLD);
  outputCSV(
      "Estimated Speed", speed_estimations, "SlowParkingRosbagTest_speed.csv");
}

TEST_F(OvalDrivingSlowRosbagTest, testSpeedEstimationOutliers) {
  std::vector<float> speed_estimations = getSpeedEstimations(intermediate_states);
  checkForOutliers(speed_estimations, SPEED_OUTLIER_THRESHOLD);
  outputCSV("Estimated Speed",
            speed_estimations,
            "OvalDrivingSlowRosbagTest_speed.csv");
}

TEST_F(OvalDrivingMediumRosbagTest, testSpeedEstimationOutliers) {
  std::vector<float> speed_estimations = getSpeedEstimations(intermediate_states);
  checkForOutliers(speed_estimations, SPEED_OUTLIER_THRESHOLD);
  outputCSV("Estimated Speed",
            speed_estimations,
            "OvalDrivingMediumRosbagTest_speed.csv");
}

TEST_F(OvalDrivingFastRosbagTest, testSpeedEstimationOutliers) {
  std::vector<float> speed_estimations = getSpeedEstimations(intermediate_states);
  checkForOutliers(speed_estimations, SPEED_OUTLIER_THRESHOLD);
  outputCSV("Estimated Speed",
            speed_estimations,
            "OvalDrivingFastRosbagTest_speed.csv");
}


TEST_F(SlowParkingRosbagTest, testYawRateEstimationOutliers) {
  std::vector<float> yaw_rate_estimations = getYawRateEstimations(intermediate_states);
  checkForOutliers(yaw_rate_estimations, YAW_RATE_OUTLIER_THRESHOLD);
  outputCSV("Estimated yaw rate",
            yaw_rate_estimations,
            "SlowParkingRosbagTest_yaw_rate.csv");
}

TEST_F(OvalDrivingSlowRosbagTest, testYawRateEstimationOutliers) {
  std::vector<float> yaw_rate_estimations = getYawRateEstimations(intermediate_states);
  checkForOutliers(yaw_rate_estimations, YAW_RATE_OUTLIER_THRESHOLD);
  outputCSV("Estimated yaw rate",
            yaw_rate_estimations,
            "OvalDrivingSlowRosbagTest_yaw_rate.csv");
}

TEST_F(OvalDrivingMediumRosbagTest, testYawRateEstimationOutliers) {
  std::vector<float> yaw_rate_estimations = getYawRateEstimations(intermediate_states);
  checkForOutliers(yaw_rate_estimations, YAW_RATE_OUTLIER_THRESHOLD);
  outputCSV("Estimated yaw rate",
            yaw_rate_estimations,
            "OvalDrivingMediumRosbagTest_yaw_rate.csv");
}

TEST_F(OvalDrivingFastRosbagTest, testYawRateEstimationOutliers) {
  std::vector<float> yaw_rate_estimations = getYawRateEstimations(intermediate_states);
  checkForOutliers(yaw_rate_estimations, YAW_RATE_OUTLIER_THRESHOLD_FAST);
  outputCSV("Estimated yaw rate",
            yaw_rate_estimations,
            "OvalDrivingFastRosbagTest_yaw_rate.csv");
}


TEST_F(StraightDriving05RosbagTest, testAverageSpeed) {  // 355 - 1208
  std::vector<float> speed_estimations = getSpeedEstimations(intermediate_states);
  const int count = 1208 - 355;
  float sum = 0.f;
  for (int i = 355; i < 1208; i++) {
    sum += speed_estimations[i];
  }
  const float average_speed = sum / count;
  EXPECT_NEAR(average_speed, 0.5f, 0.02f);
}


TEST_F(StraightDriving15RosbagTest, testAverageSpeed) {  // 400 - 585
  std::vector<float> speed_estimations = getSpeedEstimations(intermediate_states);
  const int count = 585 - 400;
  float sum = 0.f;
  for (int i = 400; i < 585; i++) {
    sum += speed_estimations[i];
  }
  const float average_speed = sum / count;
  EXPECT_NEAR(average_speed, 1.5f, 0.05f);
}

TEST_F(StraightDriving05RosbagTest, testDrivenDistance) {
  std::vector<float> speed_estimations = getSpeedEstimations(intermediate_states);
  const float driven_distance =
      (1 / 120.f) *
      std::accumulate(speed_estimations.begin(), speed_estimations.end(), 0.f);
  EXPECT_NEAR(driven_distance, 4.09f, 0.02f);
  outputCSV("Estimated Speed",
            speed_estimations,
            "StraightDriving05RosbagTest_speed.csv");
}

TEST_F(StraightDriving10RosbagTest, testDrivenDistance) {
  std::vector<float> speed_estimations = getSpeedEstimations(intermediate_states);
  const float driven_distance =
      (1 / 120.f) *
      std::accumulate(speed_estimations.begin(), speed_estimations.end(), 0.f);
  EXPECT_NEAR(driven_distance, 4.36f, 0.02f);
  outputCSV("Estimated Speed",
            speed_estimations,
            "StraightDriving10RosbagTest_speed.csv");
}

TEST_F(StraightDriving15RosbagTest, testDrivenDistance) {
  std::vector<float> speed_estimations = getSpeedEstimations(intermediate_states);
  const float driven_distance =
      (1 / 120.f) *
      std::accumulate(speed_estimations.begin(), speed_estimations.end(), 0.f);
  EXPECT_NEAR(driven_distance, 4.34f, 0.02f);
  outputCSV("Estimated Speed",
            speed_estimations,
            "StraightDriving15RosbagTest_speed.csv");
}

TEST_F(StraightDriving20RosbagTest, testDrivenDistance) {
  std::vector<float> speed_estimations = getSpeedEstimations(intermediate_states);
  const float driven_distance =
      (1 / 120.f) *
      std::accumulate(speed_estimations.begin(), speed_estimations.end(), 0.f);
  EXPECT_NEAR(driven_distance, 4.28f, 0.05f);
  outputCSV("Estimated Speed",
            speed_estimations,
            "StraightDriving20RosbagTest_speed.csv");
}

TEST_F(StraightDriving25RosbagTest, testDrivenDistance) {
  std::vector<float> speed_estimations = getSpeedEstimations(intermediate_states);
  const float driven_distance =
      (1 / 120.f) *
      std::accumulate(speed_estimations.begin(), speed_estimations.end(), 0.f);
  EXPECT_NEAR(driven_distance, 4.44f, 0.13f);
  outputCSV("Estimated Speed",
            speed_estimations,
            "StraightDriving25RosbagTest_speed.csv");
}

TEST_F(StraightDriving30RosbagTest, testDrivenDistance) {
  std::vector<float> speed_estimations = getSpeedEstimations(intermediate_states);
  const float driven_distance =
      (1 / 120.f) *
      std::accumulate(speed_estimations.begin(), speed_estimations.end(), 0.f);
  EXPECT_NEAR(driven_distance, 4.6f, 0.23f);
  outputCSV("Estimated Speed",
            speed_estimations,
            "StraightDriving30RosbagTest_speed.csv");
}

TEST_F(OvalDrivingSlowRosbagTest, testPoseDeviationBeginToEnd) {
  Eigen::Vector2d end_pose = Eigen::Vector2d(0, 0);
  double end_orientation = 0.0;
  integrateIntermediateStatesToEndPose(intermediate_states, end_pose, end_orientation);
  const double distance_to_start_position = end_pose.norm();
  EXPECT_NEAR(distance_to_start_position, 0.0, 0.1);
  EXPECT_NEAR(end_orientation, 0.0, 0.07);
}

TEST_F(OvalDrivingMediumRosbagTest, testPoseDeviationBeginToEnd) {
  Eigen::Vector2d end_pose = Eigen::Vector2d(0, 0);
  double end_orientation = 0.0;
  integrateIntermediateStatesToEndPose(intermediate_states, end_pose, end_orientation);
  const double distance_to_start_position = end_pose.norm();
  EXPECT_NEAR(distance_to_start_position, 0.0, 0.08);
  EXPECT_NEAR(end_orientation, 0.0, 0.04);
}

TEST_F(OvalDrivingFastRosbagTest, testPoseDeviationBeginToEnd) {
  Eigen::Vector2d end_pose = Eigen::Vector2d(0, 0);
  double end_orientation = 0.0;
  integrateIntermediateStatesToEndPose(intermediate_states, end_pose, end_orientation);
  const double distance_to_start_position = end_pose.norm();
  EXPECT_NEAR(distance_to_start_position, 0.0, 0.1);
  EXPECT_NEAR(end_orientation, 0.0, 0.07);
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "test_state_estimation");
  return RUN_ALL_TESTS();
}
