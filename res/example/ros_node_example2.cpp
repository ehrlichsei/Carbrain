/**
 * @file ros_node_example.cpp
 *
 * @brief This file is an small example of a ROS node with a publisher and subscriber.
 *
 * The chatter_subscriber subscribes to the topic of the chatter_publisher.
 * Usually a node does not subscribe to its own topic, but to topics of other nodes.
 */


#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Int32.h"

#include <sstream>


ros::Publisher extracted_number_publisher;


void chatterCallback(const std_msgs::String::ConstPtr& msg_in)
{
  ROS_INFO("I heard: [%s]", msg_in->data.c_str());

  std_msgs::Int32 msg_out;

  sscanf(msg_in->data.c_str(), "hello world %d", &msg_out.data);

  extracted_number_publisher.publish(msg_out);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "ros_node_example2");

  ros::NodeHandle node;

  extracted_number_publisher = node.advertise<std_msgs::Int32>("extracted_number", 1000);

  ros::Subscriber chatter_subscriber = node.subscribe("chatter", 1000, chatterCallback);

  ros::spin();

  return 0;
}
