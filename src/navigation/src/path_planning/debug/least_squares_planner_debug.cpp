#include "least_squares_planner_debug.h"

LeastSquaresPlannerDebug::LeastSquaresPlannerDebug(ParameterInterface& parameters,
                                                   std::shared_ptr<navigation_msgs::CeresSummary> ceres_summary)
    : LeastSquaresPlanner(parameters), ceres_summary(std::move(ceres_summary)) {}

void LeastSquaresPlannerDebug::solveOptimizationProblem(ceres::Problem& problem,
                                                        ceres::Solver::Summary& summary) {
  LeastSquaresPlanner::solveOptimizationProblem(problem, summary);
  ceres_summary->message = summary.message;
  ceres_summary->initial_cost = summary.initial_cost;
  ceres_summary->final_cost = summary.final_cost;
  ceres_summary->num_successful_steps = summary.num_successful_steps;
  ceres_summary->num_unsuccessful_steps = summary.num_unsuccessful_steps;
  ceres_summary->total_time = summary.total_time_in_seconds;
  ceres_summary->num_parameters = summary.num_parameters;
  ceres_summary->num_parameter_blocks = summary.num_parameter_blocks;
  ceres_summary->num_parameters_reduced = summary.num_parameters_reduced;
  ceres_summary->num_parameter_blocks_reduced = summary.num_parameter_blocks_reduced;
}
