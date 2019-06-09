#include <common/macros.h>

THIRD_PARTY_HEADERS_BEGIN
#include <iostream>
#include <string>
#include <cmath>
#include <dynamic_reconfigure/Reconfigure.h>
#include <ros/callback_queue.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <ros/ros.h>

//#include <ackermann_msgs/AckermannDriveStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/Range.h>
#include "navigation_msgs/FindParkingSlot.h"
THIRD_PARTY_HEADERS_END

#include "parkingslotfinder.h"

static ros::CallbackQueue service_callback_queue;
static ros::ServiceServer service;

bool serviceCallback(navigation_msgs::FindParkingSlotRequest &,
                     navigation_msgs::FindParkingSlotResponse &) {
  ROS_INFO("FindParkingSlot Service was called");
  ParkingSlotFinder slot_finder;
  return slot_finder.findParkingSlot();
}

void exposeService(ros::NodeHandle &node, const std::string& serviceTopic) {
  ros::AdvertiseServiceOptions opts =
      ros::AdvertiseServiceOptions::create<navigation_msgs::FindParkingSlot>(
          serviceTopic, serviceCallback, ros::VoidPtr(), &service_callback_queue);
  service = node.advertiseService(opts);
}

/**
* polls the message queues
* in addition to the default queue,
* the service queue is also polled.
* A separate queue is necessary because the service
* invocation blocks until the action is finished
* and other callbacks would not be serviced otherwise
* in this interval.
*/
void spin() {
  while (ros::ok()) {
    ros::spinOnce();
    service_callback_queue.callOne(ros::WallDuration(0.1));
  }
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "find_parking_slot");
  ros::NodeHandle node;

  sleep(2);
  ROS_INFO("Exposing ROS Service /find_slot");

  exposeService(node, "find_slot");
  spin();
  return 0;
}
