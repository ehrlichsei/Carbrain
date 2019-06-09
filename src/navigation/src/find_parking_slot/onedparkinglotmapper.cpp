#include "onedparkinglotmapper.h"

OneDParkinglotMapper::OneDParkinglotMapper(float min_obstacle_len, float max_ramp_y_diff)
    : max_ramp_y_diff(max_ramp_y_diff), min_obstacle_len(min_obstacle_len) {}

static inline bool isMapEntryOccupied(const int8_t value) { return value > 85; }

nav_msgs::OccupancyGrid OneDParkinglotMapper::mapToOneD(const nav_msgs::OccupancyGrid &map) const {
  nav_msgs::OccupancyGrid out = map;
  out.info.width = 1;
  out.data.resize(out.info.height);
  for (size_t row_index = 0; row_index < map.info.height; row_index++) {
    float distance_in_m = 0.0;
    bool occupied = checkOccupied(map, row_index, distance_in_m);
    out.data[row_index] = occupied * VALUE_OCCUPIED;
  }

  // Check for ramp at beginning of the obstacle.
  for (size_t row_index = 0; row_index < map.info.height - 1; row_index++) {
    if (out.data[row_index] == VALUE_OCCUPIED) {
      // Measure the distace to the obstacle.
      float distance_in_m = 0.0;
      checkOccupied(map, row_index, distance_in_m);

      // Check y diff to next row distance.
      size_t next_row_index = row_index + 1;
      float next_distance_in_m = distance_in_m;
      bool next_occupied = checkOccupied(map, next_row_index, next_distance_in_m);
      if (next_occupied && (distance_in_m - next_distance_in_m) > max_ramp_y_diff) {
        out.data[row_index] = VALUE_FILTERED_OUT;
      }
    }
  }

  // Check for ramp at end of the obstacle.
  for (size_t row_index = map.info.height; row_index-- > 1;) {
    if (out.data[row_index] == VALUE_OCCUPIED) {
      // Measure the distace to the obstacle.
      float distance_in_m = 0.0;
      checkOccupied(map, row_index, distance_in_m);

      // Check y diff to next row distance.
      size_t prev_row_index = row_index - 1;
      float prev_distance_in_m = distance_in_m;
      bool prev_occupied = checkOccupied(map, prev_row_index, prev_distance_in_m);
      if (prev_occupied && (distance_in_m - prev_distance_in_m) > max_ramp_y_diff) {
        out.data[row_index] = VALUE_FILTERED_OUT;
      }
    }
  }

  // Filter out too small obstacles.
  bool inside_obj = false;
  size_t obj_start_index = 0;
  for (size_t row_index = 0; row_index <= map.info.height; row_index++) {
    if (row_index != map.info.height && out.data[row_index] == VALUE_OCCUPIED) {
      if (!inside_obj) {
        inside_obj = true;
        obj_start_index = row_index;
      }
    } else {
      if (inside_obj) {
        inside_obj = false;
        float obj_size = map.info.resolution * (row_index - obj_start_index);
        if (obj_size < min_obstacle_len) {
          for (size_t del_row_index = obj_start_index; del_row_index < row_index; del_row_index++) {
            out.data[del_row_index] = VALUE_FILTERED_OUT;
          }
        }
      }
    }
  }

  return out;
}

bool OneDParkinglotMapper::checkOccupied(const nav_msgs::OccupancyGrid &map,
                                         size_t row_index,
                                         float &out_distance_in_m) const {
  auto row_start = map.data.begin() + (row_index * map.info.width);
  auto row_end = row_start + map.info.width;
  bool occupied = false;
  size_t depthIndex = 0;
  for (auto col = row_start; col != row_end && !occupied; ++col) {
    occupied |= isMapEntryOccupied(*col);
    if (occupied) {
      depthIndex = col - row_start;
      out_distance_in_m = map.info.resolution * depthIndex;
    }
    ++depthIndex;
  }

  return occupied;
}
