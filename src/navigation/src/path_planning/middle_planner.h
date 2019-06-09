#ifndef MIDDLE_PLANNER_H
#define MIDDLE_PLANNER_H

#include "path_planner.h"

class MiddlePlanner : public PathPlanner {
 public:

  MiddlePlanner(ParameterInterface &parameters);

  virtual Path planPathOnCorridor(const DrivingCorridor &corridor) override;

};

#endif  // MIDDLE_PLANNER_H
