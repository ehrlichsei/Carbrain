#ifndef PATH_PLANNER_H
#define PATH_PLANNER_H

#include "common/parameter_interface.h"
#include "common/types.h"
#include "navigation/driving_corridor.h"

/**
 * Abstract class for planning a path to drive on given a corridor which
 * describes the lane constraints.
 */
class PathPlanner {
 public:
  typedef common::Vector2dVector Path;

  PathPlanner(ParameterInterface &parameters);

  virtual ~PathPlanner() = default;

  /**
   * \brief plans path on given corridor
   *
   * \param corridor to plan path on
   * \return path to drive on
   */
  virtual Path planPathOnCorridor(const DrivingCorridor &corridor) = 0;

 protected:
  ParameterInterface &parameters_;
};

#endif  // PATH_PLANNER_H
