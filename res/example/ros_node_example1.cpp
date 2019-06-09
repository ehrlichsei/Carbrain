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

#include <sstream>


ros::Publisher chatter_publisher;
int count = 0;


void publishMessage(const ros::TimerEvent& event)
{
  std_msgs::String msg;

  std::stringstream ss;
  ss << "hello world " << count++;
  msg.data = ss.str();

  ROS_INFO("%s", msg.data.c_str());

  chatter_publisher.publish(msg);
}

void chatterCallback(const std_msgs::String::ConstPtr& msg)
{
  ROS_INFO("I heard: [%s]", msg->data.c_str());
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "ros_node_example1");

  ros::NodeHandle node;

  chatter_publisher = node.advertise<std_msgs::String>("chatter", 1000);

  ros::Subscriber chatter_subscriber = node.subscribe("chatter", 1000, chatterCallback);

  ros::Timer publish_timer = node.createTimer(ros::Duration(0.1), publishMessage);

  ros::spin();

  return 0;
}
