#include <gtest/gtest.h>
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <common/realtime_channel_ids.h>
#include <common/realtimeipc.h>
#include <controller_interface/sensormeasurements.h>
#include <controller_msgs/StateMeasure.h>
#include <state_estimation_msgs/ProcessSensorData.h>
#include <numeric>
#include <iostream>
#include <fstream>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <eigen3/Eigen/Core>

using namespace Eigen;


#define OUTLIER_THRESHOLD 0.15f

typedef std::vector<controller_msgs::StateMeasure> Measures;
typedef std::vector<state_estimation_msgs::State> States;

class RosbagTest : public testing::Test {
 public:
  RosbagTest(std::string rosbag_name, int update_rate)
      : rosbag_name(rosbag_name), update_rate(update_rate) {}

 protected:
  virtual void SetUp() {
    ros::NodeHandle nh("~");

    Measures measures = readRosbag();
    ros::ServiceClient state_estimation_client =
        nh.serviceClient<state_estimation_msgs::ProcessSensorData>(
            "/state_estimation/state_estimation/process_sensor_data");
    bool state_estimation_service_exists =
        state_estimation_client.waitForExistence(ros::Duration(5));
    ASSERT_TRUE(state_estimation_service_exists);

    state_estimation_msgs::ProcessSensorDataRequest request;
    state_estimation_msgs::ProcessSensorDataResponse response;
    request.measures = measures;
    request.rate = update_rate;
    bool call_return_value = state_estimation_client.call(request, response);
    ASSERT_TRUE(call_return_value);
    intermediate_states = response.states;
  }

  States intermediate_states;
  int update_rate;



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
  Measures readRosbag() {
    ros::NodeHandle nh("~");
    rosbag::Bag bag;
    std::string rosbag_path;
    nh.getParam("rosbag_path", rosbag_path);
    bag.open(rosbag_path + rosbag_name, rosbag::bagmode::Read);

    std::vector<std::string> topics;
    topics.push_back(std::string(
        "/controller_interface/controller_interface/state_measure"));

    rosbag::View view(bag, rosbag::TopicQuery(topics));

    Measures measures;
    for (rosbag::MessageInstance m : view) {
      controller_msgs::StateMeasurePtr state_measure =
          m.instantiate<controller_msgs::StateMeasure>();
      if (state_measure != std::nullptr_t()) {
        measures.push_back(*state_measure);
      }
    }
    bag.close();
    ros::Time start_time = measures.front().header.stamp;
    ros::Time end_time = measures.back().header.stamp;
    float average_rate = measures.size() / (end_time - start_time).toSec();
    EXPECT_NEAR(average_rate, update_rate, 2);
    return measures;
  }

  std::string rosbag_name;
};

class SlowParkingRosbagTest : public RosbagTest {
 public:
  SlowParkingRosbagTest()
      : RosbagTest("/test_state_estimation/test_parking.bag", 120) {}
};

TEST_F(SlowParkingRosbagTest, testSpeedEstimationOutliers) {

  std::vector<float> speed_estimations = getSpeedEstimations(intermediate_states);
  for (std::vector<float>::iterator speed_iterator = speed_estimations.begin() + 1;
       speed_iterator != speed_estimations.end();
       speed_iterator++) {
    const float estimated_speed = *speed_iterator;
    const float last_estimated_speed = *(speed_iterator - 1);
    EXPECT_NEAR(estimated_speed, last_estimated_speed, OUTLIER_THRESHOLD);
  }
  outputCSV(
      "Estimated Speed", speed_estimations, "SlowParkingRosbagTest_speed.csv");
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "test_state_estimation");
  return RUN_ALL_TESTS();
}
