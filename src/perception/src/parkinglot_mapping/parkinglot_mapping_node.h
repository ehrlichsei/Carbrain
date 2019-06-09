#ifndef PARKINGLOTMAPPINGNODE_H
#define PARKINGLOTMAPPINGNODE_H
#include <common/macros.h>

THIRD_PARTY_HEADERS_BEGIN
#include <memory>
#include <dynamic_reconfigure/server.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <nav_msgs/OccupancyGrid.h>
#include <sensor_msgs/Range.h>
#include <nav_msgs/Path.h>
#include <message_filters/subscriber.h>
THIRD_PARTY_HEADERS_END

#include "common/node_base.h"
#include "parkinglot_mapping.h"

class ParkinglotMappingNode : public NodeBase {
 public:
  ParkinglotMappingNode(ros::NodeHandle& node_handle);

  static std::string getName();

 private:
  // NodeBase interface
  void startModule() override;
  void stopModule() override;

  void infraredSensorCallback(const sensor_msgs::Range::ConstPtr& range_measurement);
  void publishMap(const ros::TimerEvent&);

  static const ParameterString<int> PARKINGLOT_MAP_WIDTH;
  static const ParameterString<int> PARKINGLOT_MAP_HEIGHT;
  static const ParameterString<int> PARKINGLOT_MAP_RESOLUTION;

  ros::Subscriber infrared_sensor_1_subscriber_;
  ros::Subscriber infrared_sensor_2_subscriber_;
  std::unique_ptr<message_filters::Subscriber<nav_msgs::Path>> lane_subscriber_;
  ros::Publisher map_publisher_;
  ros::Timer publish_map_timer_;

  tf2_ros::Buffer tf2_buffer_;
  tf2_ros::TransformListener tf2_listener_;

  std::unique_ptr<ParkinglotMapping> parkinglot_mapping_;
};

#endif  // PARKINGLOTMAPPINGNODE_H
