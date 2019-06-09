#ifndef PARKINGLOTMAPPING_H
#define PARKINGLOTMAPPING_H
#include <common/macros.h>

THIRD_PARTY_HEADERS_BEGIN
#include <memory>
#include <ros/console.h>
#include <nav_msgs/OccupancyGrid.h>
#include <sensor_msgs/Range.h>
#include <sensor_msgs/Image.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2_ros/buffer.h>
#include <nav_msgs/Path.h>
THIRD_PARTY_HEADERS_END

#include "common/parameter_interface.h"
#include "roi_birds_view_transformation.h"
#include "common/camera_transformation.h"
#include "parkinglot_map.h"

class ParkinglotMapping {
 public:
  static const ParameterString<double> PARKINGLOT_MAP_WIDTH;
  static const ParameterString<double> PARKINGLOT_MAP_HEIGHT;
  static const ParameterString<double> PARKINGLOT_MAP_RESOLUTION;

  static void registerParams(ParameterInterface& parameters);
  static std::unique_ptr<ParkinglotMapping> create(ParameterInterface* parameters, const tf2_ros::Buffer &tf2_buffer);
  void mapRangeMeasurement(const sensor_msgs::Range::ConstPtr &range_measurement);
  const nav_msgs::OccupancyGrid getMap();

  ParkinglotMapping(ParameterInterface *parameters, const tf2_ros::Buffer &tf2_buffer);

 private:
  void mapRangeCell(const int x_map, const int y_map, const bool is_start, const bool is_end);
  void roadLane(const int x_map, const int y_map, const bool is_start, const bool is_end);
  int bayes(const int actual_value, const int update_value);

  ParameterInterface *parameters_ptr_;

  ros::Time last_map_update_;
  ParkinglotMap parkinglot_map_;
  ROIBirdsViewTransformation birdsview_transformation_;

  const tf2_ros::Buffer &tf2_buffer_;
};

#endif  // PARKINGLOTMAPPING_H
