#ifndef LEAST_SQUARES_PLANNER_DEBUG_H
#define LEAST_SQUARES_PLANNER_DEBUG_H

#include "../least_squares_planner.h"

#include <navigation_msgs/CeresSummary.h>

/**
 * class for generating debug output for LeastSquaresPlanner
*/
class LeastSquaresPlannerDebug : public LeastSquaresPlanner {
 public:
  LeastSquaresPlannerDebug(ParameterInterface& parameters,
                           std::shared_ptr<navigation_msgs::CeresSummary> ceres_summary);

  virtual void solveOptimizationProblem(ceres::Problem& problem,
                                        ceres::Solver::Summary& summary) override;

  std::shared_ptr<navigation_msgs::CeresSummary> ceres_summary;
};

#endif  // LEAST_SQUARES_PLANNER_DEBUG_H
