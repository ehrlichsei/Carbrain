#include "middle_planner.h"

MiddlePlanner::MiddlePlanner(ParameterInterface &parameters)
    : PathPlanner(parameters) {}

PathPlanner::Path MiddlePlanner::planPathOnCorridor(const DrivingCorridor &corridor) {
  Path path;
  for (const Gate& gate : corridor) {
    path.push_back(gate.getCenterProjection());
  }
  return path;
}
