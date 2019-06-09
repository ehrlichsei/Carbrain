#include "parkinglot_mapping_node.h"
#include <common/macros.h>

THIRD_PARTY_HEADERS_BEGIN
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <boost/bind.hpp>

#include "perception/ParkinglotMappingNodeConfig.h"
THIRD_PARTY_HEADERS_END

#include "common/node_creation_makros.h"


ParkinglotMappingNode::ParkinglotMappingNode(ros::NodeHandle& node_handle)
    : NodeBase(node_handle),
      tf2_listener_(tf2_buffer_),
      parkinglot_mapping_(nullptr) {
  ParkinglotMapping::registerParams(parameter_handler_);
  parameter_handler_.addDynamicReconfigureServer<perception::ParkinglotMappingNodeConfig>(
     node_handle);
}

void ParkinglotMappingNode::startModule() {
  parkinglot_mapping_ = ParkinglotMapping::create(&parameter_handler_, tf2_buffer_);
  infrared_sensor_1_subscriber_ = node_handle_.subscribe(
      "infrared_sensor_front", 100, &ParkinglotMappingNode::infraredSensorCallback, this);
  infrared_sensor_2_subscriber_ = node_handle_.subscribe(
      "infrared_sensor_back", 100, &ParkinglotMappingNode::infraredSensorCallback, this);
  lane_subscriber_ = std::make_unique<message_filters::Subscriber<nav_msgs::Path>>(
      node_handle_, "road_lane_right", 1);
  map_publisher_ = node_handle_.advertise<nav_msgs::OccupancyGrid>("map", 1);
  publish_map_timer_ = node_handle_.createTimer(
      ros::Duration(0.1), &ParkinglotMappingNode::publishMap, this);
}

void ParkinglotMappingNode::stopModule() {
  infrared_sensor_1_subscriber_.shutdown();
  infrared_sensor_2_subscriber_.shutdown();
  map_publisher_.shutdown();
  publish_map_timer_.stop();
}

void ParkinglotMappingNode::infraredSensorCallback(const sensor_msgs::Range::ConstPtr& range_measurement) {
  parkinglot_mapping_->mapRangeMeasurement(range_measurement);
}

void ParkinglotMappingNode::publishMap(const ros::TimerEvent&) {
  map_publisher_.publish(parkinglot_mapping_->getMap());
}

std::string ParkinglotMappingNode::getName() {
  return std::string("parkinglot_mapping");
}

CREATE_NODE(ParkinglotMappingNode)
