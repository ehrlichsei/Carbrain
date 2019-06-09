#include <common/macros.h>

THIRD_PARTY_HEADERS_BEGIN
#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include "sensor_msgs/Range.h"
#include "std_msgs/Int32.h"
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <iostream>
#include <string>
#include <cmath>
#include <ros/callback_queue.h>
#include "controller_msgs/BlinkerCommand.h"
#include "lateral_controller_msgs/DrivingSteeringAngle.h"
#include "longitudinal_controller_msgs/DrivingSpeed.h"
#include <memory>
THIRD_PARTY_HEADERS_END

#include "parkingparameters.h"
#include "parkingorchestrator.h"
#include "parkingcarinterface.h"
#include "parkingservice.h"


static ros::CallbackQueue service_callback_queue;

static std::unique_ptr<ParkingOrchestrator> parking_sequence;
static std::unique_ptr<tf::TransformListener> transform_listener;

void locationCallback(const geometry_msgs::PoseStamped::ConstPtr &) {
  tf::StampedTransform transform;
  try {
    transform_listener->lookupTransform("world", "vehicle", ros::Time(0), transform);
  } catch (const tf::LookupException &) {
    ROS_WARN("Transform lookup failed");
  }
  parking_sequence->setOrientationEstimation(transform.getRotation());
}


void distanceSensorCallback(const sensor_msgs::Range::ConstPtr &range) {
  if (range->header.frame_id == "sensors/distance_sensor_2") {
    parking_sequence->setRangeSensors(range);
  }
}

void spin() {
  while (ros::ok()) {
    ros::spinOnce();
    service_callback_queue.callAvailable(ros::WallDuration(0.1));
  }
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "parking");
  ros::NodeHandle node("~");

  // ros::Subscriber location_subscriber = node.subscribe("pose_estimation", 10,
  // locationCallback);
  // ros::Subscriber state_subscriber = node.subscribe("state_estimation", 1000,
  // locationCallback);

  ros::Publisher drive_command_publisher_speed =
      node.advertise<longitudinal_controller_msgs::DrivingSpeed>(
          "velocity_command", 10);
  ros::Publisher drive_command_publisher_steering_angle =
      node.advertise<lateral_controller_msgs::DrivingSteeringAngle>(
          "steering_angle_command_front", 10);

  ros::Publisher blinker_publisher =
      node.advertise<controller_msgs::LightsCommand>("blinker_command", 1);

  ParkingParameters parameters;
  parameters.readParameters(node);

  RosParkingDebugUtil parking_debug(node);

  ParkingCarInterface car(
      parameters, drive_command_publisher_speed, drive_command_publisher_steering_angle);
  parking_sequence = std::make_unique<ParkingOrchestrator>(
      parameters, car, parking_debug, blinker_publisher);

  ros::Subscriber location_subscriber =
      node.subscribe("pose_estimation", 1000, locationCallback);

  ros::Subscriber distance_sensor_subscriber =
      node.subscribe("distance_sensors", 1000, distanceSensorCallback);

  std::unique_ptr<ParkingService> UNUSED parking_service = std::make_unique<ParkingService>(
      &node, "drive_into_parking_slot", parking_sequence.get(), &service_callback_queue);

  transform_listener = std::make_unique<tf::TransformListener>();

  ROS_INFO("parking node started");
  spin();
  return 0;
}
