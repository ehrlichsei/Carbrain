#ifndef LINE_VEHICLE_POINTS_H
#define LINE_VEHICLE_POINTS_H

#include "common/join.h"

#include "perception_types.h"

/**
 * Type definition for bundles of lines in the image.
 */
typedef std::array<VehiclePoints, LINESPEC_N>   LineVehiclePoints;

inline auto join(const LineVehiclePoints& ldp) {
  return common::join(
      ldp[LINESPEC_LEFT], ldp[LINESPEC_MIDDLE], ldp[LINESPEC_NO_PASSING], ldp[LINESPEC_RIGHT]);
}

void inline clear(LineVehiclePoints* line_points_data) {
  for (auto& line : *line_points_data) {
    line.clear();
  }
}

inline LineSpec smallerLine(const LineVehiclePoints& lines, LineSpec a, LineSpec b) {
  return lines.at(a).size() < lines.at(b).size() ? a : b;
}

#endif  // LINE_VEHICLE_POINTS_H
