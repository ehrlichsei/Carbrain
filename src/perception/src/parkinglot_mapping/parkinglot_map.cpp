#include "parkinglot_map.h"

#include <cmath>
#include <boost/range/algorithm/transform.hpp>
#include <boost/range/algorithm/fill.hpp>

ParkinglotMap::ParkinglotMap(const float width_in_parkinglot,
                             const float height_in_parkinglot,
                             const float resolution,
                             const float default_value)
    : width_in_parkinglot(width_in_parkinglot),
      height_in_parkinglot(height_in_parkinglot),
      width_in_map(static_cast<std::size_t>(width_in_parkinglot / resolution)),
      height_in_map(static_cast<std::size_t>(height_in_parkinglot / resolution)),
      resolution(resolution),
      default_value(default_value) {
  // Divide the area given by width and height into a grid using the given
  // resolution.
  data_in_map.resize(width_in_map * height_in_map);
  resetAllData(default_value);
}

float ParkinglotMap::getData(const float x_in_parkinglot, const float y_in_parkinglot) {
  int map_index = convertFromParkinglotToMapIndexOrNegative(x_in_parkinglot, y_in_parkinglot);
  bool in_range = map_index >= 0;

  // Return default value if position lies outside map.
  float value = default_value;
  if (in_range) {
    value = data_in_map[map_index];
  }

  return value;
}

bool ParkinglotMap::updateData(const float x_in_parkinglot,
                               const float y_in_parkinglot,
                               const float update_value) {
  int x_in_map = convertFromParkinglotToMap(x_in_parkinglot);
  int y_in_map = convertFromParkinglotToMap(y_in_parkinglot);

  return updateDataInMap(x_in_map, y_in_map, update_value);
}

bool ParkinglotMap::updateDataInMap(const int x_in_map, const int y_in_map, const float update_value) {
  const int map_index = convertFromMapToIndexOrNegative(x_in_map, y_in_map);
  const bool in_range = map_index >= 0;
  if (in_range) {
    const float old_value = data_in_map[map_index];
    const float s = (update_value / (1.0f - update_value)) *
                    std::min((old_value / (1.0f - old_value)), 100000.f);

    data_in_map[map_index] = (s / (1.0f + s)) * 1.0f;
  }

  return in_range;
}

bool ParkinglotMap::updateDataOnLine(const float x_start_in_parkinglot,
                                     const float y_start_in_parkinglot,
                                     const float x_end_in_parkinglot,
                                     const float y_end_in_parkinglot,
                                     const float line_update_value,
                                     const float end_update_value) {
  int x_start_in_map = convertFromParkinglotToMap(x_start_in_parkinglot);
  int y_start_in_map = convertFromParkinglotToMap(y_start_in_parkinglot);
  int x_end_in_map = convertFromParkinglotToMap(x_end_in_parkinglot);
  int y_end_in_map = convertFromParkinglotToMap(y_end_in_parkinglot);

  bool success = true;
  common::bresenhamFunctional(
      x_start_in_map,
      y_start_in_map,
      x_end_in_map,
      y_end_in_map,
      [this, line_update_value, end_update_value, &success](
          const int x_in_map, const int y_in_map, const bool, const bool is_end) {
        success &= updateDataInMap(
            x_in_map, y_in_map, is_end ? end_update_value : line_update_value);
      });

  return success;
}

bool ParkinglotMap::updateDataOnRay(const float x_start_in_parkinglot,
                                    const float y_start_in_parkinglot,
                                    const float x_switch_in_parkinglot,
                                    const float y_switch_in_parkinglot,
                                    const float first_update_value,
                                    const float last_update_value) {
  tf2::Vector3 v_start_in_parkinglot(x_start_in_parkinglot, y_start_in_parkinglot, 0);
  tf2::Vector3 v_switch_in_parkinglot(x_switch_in_parkinglot, y_switch_in_parkinglot, 0);
  tf2::Vector3 v_dir_in_parkinglot = v_switch_in_parkinglot - v_start_in_parkinglot;

  float additional_length_in_parkinglot = std::numeric_limits<float>::infinity();
  // Check x direction.
  if (!floatsEqual(v_dir_in_parkinglot.getX(), 0)) {
    float x_additional_length_in_parkinglot;
    if (v_dir_in_parkinglot.getX() > EPSILON) {
      // Check positive x direction.
      x_additional_length_in_parkinglot =
          static_cast<float>((width_in_parkinglot - v_switch_in_parkinglot.getX()) /
                             v_dir_in_parkinglot.getX());
    } else {
      // Check negative x direction.
      x_additional_length_in_parkinglot = static_cast<float>(
          (-v_switch_in_parkinglot.getX()) / v_dir_in_parkinglot.getX());
    }
    if (x_additional_length_in_parkinglot > 0)
      additional_length_in_parkinglot =
          std::min(additional_length_in_parkinglot, x_additional_length_in_parkinglot);
    else
      additional_length_in_parkinglot = 0;
  }
  // Check y direction.
  if (!floatsEqual(v_dir_in_parkinglot.getY(), 0)) {
    float y_additional_length_in_parkinglot;
    if (v_dir_in_parkinglot.getY() > EPSILON) {
      y_additional_length_in_parkinglot =
          static_cast<float>((height_in_parkinglot - v_switch_in_parkinglot.getY()) /
                             v_dir_in_parkinglot.getY());
    } else {
      y_additional_length_in_parkinglot = static_cast<float>(
          (-v_switch_in_parkinglot.getY()) / v_dir_in_parkinglot.getY());
    }
    if (y_additional_length_in_parkinglot > 0)
      additional_length_in_parkinglot =
          std::min(additional_length_in_parkinglot, y_additional_length_in_parkinglot);
    else
      additional_length_in_parkinglot = 0;
  }

  tf2::Vector3 v_end_in_parkinglot =
      v_switch_in_parkinglot + additional_length_in_parkinglot * v_dir_in_parkinglot;

  float x_end_in_parkinglot = static_cast<float>(v_end_in_parkinglot.getX());
  float y_end_in_parkinglot = static_cast<float>(v_end_in_parkinglot.getY());

  int x_start_in_map = convertFromParkinglotToMap(x_start_in_parkinglot);
  int y_start_in_map = convertFromParkinglotToMap(y_start_in_parkinglot);
  int x_switch_in_map = convertFromParkinglotToMap(x_switch_in_parkinglot);
  int y_switch_in_map = convertFromParkinglotToMap(y_switch_in_parkinglot);

  bool success = true;
  common::bresenhamFunctional(
      x_start_in_map,
      y_start_in_map,
      x_switch_in_map,
      y_switch_in_map,
      [this, first_update_value, &success](
          const int x_in_map, const int y_in_map, const bool, const bool is_end) {
        // Skip end to avoid updating twice which would be wrong.
        if (!is_end) {
          success &= updateDataInMap(x_in_map, y_in_map, first_update_value);
        }
      });
  success &= updateDataOnLine(x_switch_in_parkinglot,
                              y_switch_in_parkinglot,
                              x_end_in_parkinglot,
                              y_end_in_parkinglot,
                              last_update_value,
                              last_update_value);

  return success;
}

nav_msgs::OccupancyGrid ParkinglotMap::createMessage(const std::string& frame_id,
                                                     const ros::Time& time_stamp,
                                                     const double orientation_w) {
  nav_msgs::OccupancyGrid msg;
  msg.header.frame_id = frame_id;
  msg.header.stamp = time_stamp;
  msg.info.origin.orientation.w = orientation_w;
  msg.info.resolution = resolution;
  msg.info.width = width_in_map;
  msg.info.height = height_in_map;
  msg.data.resize(data_in_map.size());
  // OccupancyGrid expects integer values from 0 to 100.
  // Internal representation is float however to avoid underflows.
  boost::transform(
      data_in_map, msg.data.begin(), [](const auto& x) { return x * 100; });
  return msg;
}

void ParkinglotMap::resetAllData(const float reset_value) {
  boost::fill(data_in_map, reset_value);
}

int ParkinglotMap::convertFromParkinglotToMap(const float coordinate_component) {
  return static_cast<int>(coordinate_component / resolution);
}

int ParkinglotMap::convertFromMapToIndexOrNegative(const int x_in_map, const int y_in_map) {
  int index_or_negative_one = -1;
  if (x_in_map >= 0 && static_cast<std::size_t>(x_in_map) < width_in_map &&
      y_in_map >= 0 && static_cast<std::size_t>(y_in_map) < height_in_map)
    index_or_negative_one = y_in_map * width_in_map + x_in_map;

  return index_or_negative_one;
}

int ParkinglotMap::convertFromParkinglotToMapIndexOrNegative(const float x_in_parkinglot,
                                                             const float y_in_parkinglot) {
  int x_in_map = convertFromParkinglotToMap(x_in_parkinglot);
  int y_in_map = convertFromParkinglotToMap(y_in_parkinglot);

  return convertFromMapToIndexOrNegative(x_in_map, y_in_map);
}

bool ParkinglotMap::floatsEqual(float f1, float f2) {
  return std::abs(f1 - f2) < EPSILON;
}
