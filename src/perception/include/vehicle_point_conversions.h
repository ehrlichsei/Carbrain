#ifndef VEHICLE_POINT_CONVERSIONS_H
#define VEHICLE_POINT_CONVERSIONS_H
#include <common/macros.h>

THIRD_PARTY_HEADERS_BEGIN
#include <nav_msgs/Path.h>
THIRD_PARTY_HEADERS_END

#include "common/path_conversion.h"
#include "perception_types.h"

inline LineVehiclePoints createLineVehiclePoints(const nav_msgs::PathConstPtr right_points,
                                                 const nav_msgs::PathConstPtr middle_points,
                                                 const nav_msgs::PathConstPtr left_points,
                                                 const nav_msgs::PathConstPtr no_passing_points) {

  LineVehiclePoints points;
  tf2::fromMsg(*right_points, points[LINESPEC_RIGHT]);
  tf2::fromMsg(*middle_points, points[LINESPEC_MIDDLE]);
  tf2::fromMsg(*left_points, points[LINESPEC_LEFT]);
  tf2::fromMsg(*no_passing_points, points[LINESPEC_NO_PASSING]);
  return points;
}



#endif  // VEHICLE_POINT_CONVERSIONS_H
