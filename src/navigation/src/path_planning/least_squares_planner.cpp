#include "least_squares_planner.h"
#include <common/macros.h>

THIRD_PARTY_HEADERS_BEGIN
#include <ros/console.h>
#include <boost/algorithm/clamp.hpp>
THIRD_PARTY_HEADERS_END

#include "least_squares_functors/angle_functor.h"
#include "least_squares_functors/triangle_area_functor.h"
#include "least_squares_functors/curvature_functor.h"
#include "least_squares_functors/distance_functor.h"
#include "least_squares_functors/endpoint_angle_functor.h"
#include "least_squares_functors/prefered_point_functor.h"

const ParameterString<double> PARAM_LSQ_CENTER_WEIGHT("lsq_center_weight");
const ParameterString<double> PARAM_LSQ_BORDER_RESIDUAL_AT_BORDER(
    "lsq_border_residual_at_border");
const ParameterString<double> PARAM_LSQ_BORDER_START_DISTANCE(
    "lsq_border_start_distance");
const ParameterString<double> PARAM_LSQ_PREFERED_POINT_WEIGHT(
    "lsq_prefered_point_weight");
const ParameterString<double> PARAM_LSQ_DISTANCE_WEIGHT("lsq_distance_weight");
const ParameterString<double> PARAM_LSQ_ANGLE_WEIGHT("lsq_angle_weight");
const ParameterString<double> PARAM_LSQ_TRIANGLE_AREA_WEIGHT(
    "lsq_triangle_area_weight");
const ParameterString<double> PARAM_LSQ_CURVATURE_WEIGHT(
    "lsq_curvature_weight");
const ParameterString<double> PARAM_LSQ_ENDPOINT_ANGLE_WEIGHT(
    "lsq_endpoint_angle_weight");
const ParameterString<double> PARAM_LSQ_CENTER_VALUE("lsq_center_value");
const ParameterString<double> PARAM_LSQ_MIN_GATE_WIDTH("lsq_min_gate_width");
const ParameterString<int> MAX_GATES_INITIALIZATION("max_gates_initalization");
const ParameterString<int> NUMBER_FREE_GATES_TRACKING_COLLISION(
    "number_free_gates_tracking_collision");
const ParameterString<int> PARAM_CERES_MAX_NUM_ITERATIONS(
    "ceres_max_num_iterations");
const ParameterString<double> PARAM_CERES_MAX_SOLVER_TIME(
    "ceres_max_solver_time");

LeastSquaresPlanner::LeastSquaresPlanner(ParameterInterface& parameters)
    : PathPlanner(parameters) {
  parameters.registerParam(PARAM_LSQ_CENTER_WEIGHT);
  parameters.registerParam(PARAM_LSQ_BORDER_RESIDUAL_AT_BORDER);
  parameters.registerParam(PARAM_LSQ_BORDER_START_DISTANCE);
  parameters.registerParam(PARAM_LSQ_PREFERED_POINT_WEIGHT);
  parameters.registerParam(PARAM_LSQ_DISTANCE_WEIGHT);
  parameters.registerParam(PARAM_LSQ_ANGLE_WEIGHT);
  parameters.registerParam(PARAM_LSQ_TRIANGLE_AREA_WEIGHT);
  parameters.registerParam(PARAM_LSQ_CURVATURE_WEIGHT);
  parameters.registerParam(PARAM_LSQ_ENDPOINT_ANGLE_WEIGHT);
  parameters.registerParam(PARAM_LSQ_CENTER_VALUE);
  parameters.registerParam(PARAM_LSQ_MIN_GATE_WIDTH);
  parameters.registerParam(MAX_GATES_INITIALIZATION);
  parameters.registerParam(NUMBER_FREE_GATES_TRACKING_COLLISION);
  parameters.registerParam(PARAM_CERES_MAX_NUM_ITERATIONS);
  parameters.registerParam(PARAM_CERES_MAX_SOLVER_TIME);
}

void addBoundedParameter(ceres::Problem& problem,
                         double* gate_parameter,
                         const Gate& bound,
                         const double& min_gate_width) {
  problem.AddParameterBlock(gate_parameter, 1);
  double left_bound = bound.getParam<Gate::LEFT>();
  double right_bound = bound.getParam<Gate::RIGHT>();
  double distance = std::abs(right_bound - left_bound);
  if (distance < min_gate_width) {
    left_bound = boost::algorithm::clamp(
        left_bound - (min_gate_width - distance) / 2.0, 0.0, 1.0 - min_gate_width);
    right_bound = boost::algorithm::clamp(
        right_bound + (min_gate_width - distance) / 2.0, min_gate_width, 1.0);
  }
  problem.SetParameterLowerBound(gate_parameter, 0, left_bound);
  problem.SetParameterUpperBound(gate_parameter, 0, right_bound);
}

PathPlanner::Path LeastSquaresPlanner::planPathOnCorridor(const DrivingCorridor& corridor) {

  const unsigned long maxGates = parameters_.getParam(MAX_GATES_INITIALIZATION);

  std::vector<double> gate_parameters(corridor.size(), 0.0);

  for (int i = 0; i < static_cast<int>(corridor.size()); i++) {
    int evaluatedGates = std::min(maxGates, (corridor.size() - i));
    double centerSum = 0.0;
    int startGate = std::max(-i, -evaluatedGates);
    for (int j = startGate; j < evaluatedGates; j++) {
      centerSum += corridor.at(i + j).getParam<Gate::CENTER>();
    }
    double param = centerSum / (evaluatedGates - startGate);
    gate_parameters[i] = boost::algorithm::clamp(
        param, corridor.at(i).getParam<Gate::LEFT>(), corridor.at(i).getParam<Gate::RIGHT>());
  }

  ceres::Problem problem;
  addParameterBlocks(corridor, gate_parameters, problem);
  addCostFunctors(corridor, gate_parameters, problem);
  ceres::Solver::Summary summary;
  solveOptimizationProblem(problem, summary);

  Path path;
  path.reserve(corridor.size());
  tracked_gates_.clear();
  tracked_gates_.reserve(corridor.size());
  for (size_t i = 0; i < corridor.size(); i++) {
    const Gate& gate = corridor.at(i);
    path.emplace_back(gate.toPoint(gate_parameters[i]).head<2>());
    tracked_gates_.emplace_back(gate.getId(), gate_parameters[i]);
  }
  return path;
}

int LeastSquaresPlanner::getIndexLastTrackedGate(const DrivingCorridor& corridor,
                                                 std::vector<double>& gate_parameters) const {
  int index_last_tracked_gate = -1;
  std::vector<std::tuple<int, double>>::const_iterator tracked_iterator =
      tracked_gates_.begin();
  for (size_t i = 0; i < corridor.size(); i++) {
    while (tracked_iterator != tracked_gates_.end() &&
           std::get<0>(*tracked_iterator) < corridor.at(i).getId()) {
      tracked_iterator++;
    }
    if (tracked_iterator == tracked_gates_.end()) {
      return index_last_tracked_gate;
    }
    if (std::get<0>(*tracked_iterator) != corridor.at(i).getId()) {
      return index_last_tracked_gate;
    }
    if (!corridor.at(i).contains(std::get<1>(*tracked_iterator))) {
      const int number_free_gates_tracking_collision =
          parameters_.getParam(NUMBER_FREE_GATES_TRACKING_COLLISION);
      index_last_tracked_gate -= number_free_gates_tracking_collision;
      return index_last_tracked_gate;
    }
    index_last_tracked_gate = i;
    gate_parameters[i] = std::get<1>(*tracked_iterator);
  }

  return index_last_tracked_gate;
}

void LeastSquaresPlanner::addParameterBlocks(const DrivingCorridor& corridor,
                                             std::vector<double>& gate_parameters,
                                             ceres::Problem& problem) const {
  const double min_gate_width = parameters_.getParam(PARAM_LSQ_MIN_GATE_WIDTH);

  const int index_last_tracked_gate = getIndexLastTrackedGate(corridor, gate_parameters);
  for (size_t i = 0; i < corridor.size(); i++) {
    if (index_last_tracked_gate < static_cast<int>(i)) {
      addBoundedParameter(problem, &(gate_parameters[i]), corridor.at(i), min_gate_width);
    } else {
      problem.AddParameterBlock(&(gate_parameters[i]), 1);
      problem.SetParameterBlockConstant(&(gate_parameters[i]));
    }
  }
}

void LeastSquaresPlanner::addCostFunctors(const DrivingCorridor& corridor,
                                          std::vector<double>& gate_parameters,
                                          ceres::Problem& problem) const {
  const double center_weight = parameters_.getParam(PARAM_LSQ_CENTER_WEIGHT);
  const double border_residual_at_border =
      parameters_.getParam(PARAM_LSQ_BORDER_RESIDUAL_AT_BORDER);
  const double border_start_distance = parameters_.getParam(PARAM_LSQ_BORDER_START_DISTANCE);
  const double prefered_point_weight = parameters_.getParam(PARAM_LSQ_PREFERED_POINT_WEIGHT);
  const double distance_weight = parameters_.getParam(PARAM_LSQ_DISTANCE_WEIGHT);
  const double triangle_area_weight = parameters_.getParam(PARAM_LSQ_TRIANGLE_AREA_WEIGHT);
  const double angle_weight = parameters_.getParam(PARAM_LSQ_ANGLE_WEIGHT);
  const double curvature_weight = parameters_.getParam(PARAM_LSQ_CURVATURE_WEIGHT);
  const double endpoint_angle_weight = parameters_.getParam(PARAM_LSQ_ENDPOINT_ANGLE_WEIGHT);
  const double center_value = parameters_.getParam(PARAM_LSQ_CENTER_VALUE);

  if (corridor.empty()) {
    return;
  }

  for (size_t i = 0; i < corridor.size(); i++) {
    const Gate& gate = corridor.at(i);
    ceres::CostFunction* cost_function =
        new ceres::AutoDiffCostFunction<CenterFunctor, 2, 1>(new CenterFunctor(
            gate, center_value, center_weight, border_residual_at_border, border_start_distance));
    problem.AddResidualBlock(cost_function, std::nullptr_t(), &(gate_parameters[i]));
  }
  for (size_t i = 0; i < corridor.size(); i++) {
    const Gate& gate = corridor.at(i);
    ceres::CostFunction* cost_function =
        new ceres::AutoDiffCostFunction<PreferedPointFunctor, 1, 1>(
            new PreferedPointFunctor(gate, prefered_point_weight));
    problem.AddResidualBlock(cost_function, std::nullptr_t(), &(gate_parameters[i]));
  }

  if (distance_weight != 0) {
    for (size_t i = 0; i < corridor.size() - 1; i++) {
      const Gate& gate = corridor.at(i);
      const Gate& next_gate = corridor.at(i + 1);
      ceres::CostFunction* cost_function =
          new ceres::AutoDiffCostFunction<DistanceFunctor, 2, 1, 1>(
              new DistanceFunctor(gate, next_gate, distance_weight));
      problem.AddResidualBlock(
          cost_function, std::nullptr_t(), &(gate_parameters[i]), &(gate_parameters[i + 1]));
    }
  }


  for (size_t i = 1; i < corridor.size() - 1; i++) {
    const Gate& previous_gate = corridor.at(i - 1);
    const Gate& gate = corridor.at(i);
    const Gate& next_gate = corridor.at(i + 1);
    ceres::CostFunction* cost_function =
        new ceres::AutoDiffCostFunction<TriangleAreaFunctor, 1, 1, 1, 1>(new TriangleAreaFunctor(
            previous_gate, gate, next_gate, triangle_area_weight));
    problem.AddResidualBlock(cost_function,
                             std::nullptr_t(),
                             &(gate_parameters[i - 1]),
                             &(gate_parameters[i]),
                             &(gate_parameters[i + 1]));
  }


  if (angle_weight > 0.) {
    for (size_t i = 1; i < corridor.size() - 1; i++) {
      const Gate &previous_gate = corridor.at(i - 1);
      const Gate &gate = corridor.at(i);
      const Gate &next_gate = corridor.at(i + 1);
      ceres::CostFunction *cost_function =
          new ceres::AutoDiffCostFunction<AngleFunctor, 1, 1, 1, 1>(
              new AngleFunctor(previous_gate, gate, next_gate, angle_weight));
      problem.AddResidualBlock(cost_function, std::nullptr_t(),
                               &(gate_parameters[i - 1]), &(gate_parameters[i]),
                               &(gate_parameters[i + 1]));
    }
  }
  if (curvature_weight > 0.) {
    for (size_t i = 1; i < corridor.size() - 1; i++) {
      const Gate &previous_gate = corridor.at(i - 1);
      const Gate &gate = corridor.at(i);
      const Gate &next_gate = corridor.at(i + 1);
      ceres::CostFunction *cost_function =
          new ceres::AutoDiffCostFunction<CurvatureFunctor, 1, 1, 1, 1>(
              new CurvatureFunctor(previous_gate, gate, next_gate, curvature_weight));
      problem.AddResidualBlock(cost_function, std::nullptr_t(),
                               &(gate_parameters[i - 1]), &(gate_parameters[i]),
                               &(gate_parameters[i + 1]));
    }
  }

  if (corridor.size() > 1) {
    const size_t i = corridor.size() - 1;
    const Gate& previous_gate = corridor.at(i - 1);
    const Gate& gate = corridor.at(i);
    ceres::CostFunction* cost_function =
        new ceres::AutoDiffCostFunction<EndpointAngleFunctor, 1, 1, 1>(
            new EndpointAngleFunctor(previous_gate, gate, endpoint_angle_weight));
    problem.AddResidualBlock(
        cost_function, std::nullptr_t(), &(gate_parameters[i - 1]), &(gate_parameters[i]));
  }
}


void LeastSquaresPlanner::solveOptimizationProblem(ceres::Problem &problem,
                                                   ceres::Solver::Summary &summary) {
  const unsigned int max_num_iterations = parameters_.getParam(PARAM_CERES_MAX_NUM_ITERATIONS);
  const double max_solver_time = parameters_.getParam(PARAM_CERES_MAX_SOLVER_TIME);
  ceres::Solver::Options options;
  options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
  options.preconditioner_type = ceres::IDENTITY;
  options.parameter_tolerance = 1e-3;
  options.function_tolerance = 1e-3;
  options.max_num_iterations = max_num_iterations;
  options.max_solver_time_in_seconds = max_solver_time;
  // options.minimizer_progress_to_stdout = true;
  Solve(options, &problem, &summary);
  // std::cout << summary.BriefReport() << "\n";
}
