#ifndef ONEDPARKINGLOTMAPPER_H
#define ONEDPARKINGLOTMAPPER_H
#include <common/macros.h>

THIRD_PARTY_HEADERS_BEGIN
#include <nav_msgs/OccupancyGrid.h>
THIRD_PARTY_HEADERS_END

class OneDParkinglotMapper
{
public:
  static constexpr int VALUE_OCCUPIED = 100;
  static constexpr int VALUE_FILTERED_OUT = 20;
  static constexpr int VALUE_EMPTY = 0;

  OneDParkinglotMapper(float min_obstacle_len, float max_ramp_y_diff);
  nav_msgs::OccupancyGrid mapToOneD(const nav_msgs::OccupancyGrid& map) const;

private:
  const float max_ramp_y_diff;
  const float min_obstacle_len;

  bool checkOccupied(const nav_msgs::OccupancyGrid& map, size_t row_index, float& out_distance_in_m) const;
};

#endif // ONEDPARKINGLOTMAPPER_H
