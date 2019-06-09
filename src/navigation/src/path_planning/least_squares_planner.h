#ifndef LEAST_SQUARES_PLANNER_H
#define LEAST_SQUARES_PLANNER_H

#include "path_planner.h"

THIRD_PARTY_HEADERS_BEGIN
#include <ceres/ceres.h>
THIRD_PARTY_HEADERS_END

/**
 * uses non linear least squares optimizer for path optimizaiton
 */
class LeastSquaresPlanner : public PathPlanner {
 public:
  LeastSquaresPlanner(ParameterInterface& parameters);

  virtual Path planPathOnCorridor(const DrivingCorridor &corridor) override;

 protected:
  virtual void addParameterBlocks(const DrivingCorridor &corridor,
                                  std::vector<double>& gate_parameters,
                                  ceres::Problem& problem) const;
  virtual void addCostFunctors(const DrivingCorridor& corridor,
                               std::vector<double>& gate_parameters,
                               ceres::Problem& problem) const;
  virtual void solveOptimizationProblem(ceres::Problem& problem,
                                        ceres::Solver::Summary& summary);

 private:
  int getIndexLastTrackedGate(const DrivingCorridor& corridor,
                                       std::vector<double>& gate_parameters) const;
  std::vector<std::tuple<int, double>> tracked_gates_;
};

#endif  // LEAST_SQUARES_PLANNER_H
