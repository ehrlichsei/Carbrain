#ifndef PARKINGLOTMAP_H
#define PARKINGLOTMAP_H
#include <common/macros.h>

THIRD_PARTY_HEADERS_BEGIN
#include <cstdlib>
#include <tf2/LinearMath/Vector3.h>
#include <ros/time.h>
#include <nav_msgs/OccupancyGrid.h>
THIRD_PARTY_HEADERS_END

#include "common/bresenham.h"

class ParkinglotMap {
 public:
  ParkinglotMap(const float width_in_parkinglot, const float height_in_parkinglot, const float resolution, const float default_value);
  nav_msgs::OccupancyGrid createMessage(const ros::Time load_time);
  float getData(const float x_in_parkinglot, const float y_in_parkinglot);
  bool updateData(const float x_in_parkinglot, const float y_in_parkinglot, const float update_value);
  bool updateDataOnLine(const float x_start_in_parkinglot, const float y_start_in_parkinglot, const float x_end_in_parkinglot, const float y_end_in_parkinglot, const float line_update_value, const float end_update_value);
  bool updateDataOnRay(const float x_start_in_parkinglot, const float y_start_in_parkinglot, const float x_switch_in_parkinglot, const float y_switch_in_parkinglot, const float first_update_value, const float last_update_value);
  nav_msgs::OccupancyGrid createMessage(const std::string& frame_id, const ros::Time& time_stamp, const double orientation_w);

private:
  static constexpr float EPSILON = 0.0001f;
  const float width_in_parkinglot;
  const float height_in_parkinglot;
  const std::size_t width_in_map;
  const std::size_t height_in_map;
  const float resolution;
  const float default_value;
  std::vector<float> data_in_map;

  bool updateDataInMap(const int x_in_map, const int y_in_map, const float update_value);
  void resetAllData(const float reset_value);
  int convertFromParkinglotToMap(const float coordinate_component);
  int convertFromMapToIndexOrNegative(const int x_in_map, const int y_in_map);
  int convertFromParkinglotToMapIndexOrNegative(const float x_in_parkinglot, const float y_in_parkinglot);
  bool floatsEqual(float f1, float f2);
};

#endif  // PARKINGLOTMAP_H
