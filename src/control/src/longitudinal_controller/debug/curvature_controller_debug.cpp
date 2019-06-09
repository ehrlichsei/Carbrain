#include "curvature_controller_debug.h"

THIRD_PARTY_HEADERS_BEGIN

THIRD_PARTY_HEADERS_END

CurvatureControllerDebug::CurvatureControllerDebug(CurvatureController&& curvature_controller,
                                                   LongitudinalControllerNode& longitudinal_controller_node)
    : CurvatureController(std::move(curvature_controller)),
      longitudinal_controller_node(longitudinal_controller_node),
      plot_backward_calculation_speed(
          longitudinal_controller_node.node_handle_,
          "backward_calculation_speed",
          0.0,
          1.0,
          0.0,
          [](const ArcLengthParameterizedSpeed& pt,
             const common::Path<>& /*path*/) { return pt.speed; },
          [](const ArcLengthParameterizedSpeed& pt) { return pt.arc_length; }),
      plot_forward_calculation_speed(
          longitudinal_controller_node.node_handle_,
          "forward_calculation_speed",
          0.0,
          0.5,
          0.5,
          [](const ArcLengthParameterizedSpeed& pt,
             const common::Path<>& /*path*/) { return pt.speed; },
          [](const ArcLengthParameterizedSpeed& pt) { return pt.arc_length; }),
      plot_curvature(
          longitudinal_controller_node.node_handle_,
          "abs_curvature",
          1.0,
          0.0,
          0.0,
          [](const ArcLengthParameterizedCurvature& pt,
             const common::Path<>& /*path*/) { return pt.curvature; },
          [](const ArcLengthParameterizedCurvature& pt) { return pt.arc_length; }) {}



double CurvatureControllerDebug::calculateSpeed(double measured_speed,
                                                const ros::Duration& dur,
                                                int driving_direction) {
  const double speed =
      CurvatureController::calculateSpeed(measured_speed, dur, driving_direction);
  if (!path_) {
    return speed;
  }

  calculate_curvature(path_.get());

  if (longitudinal_controller_node.target_path_header_) {
    plot_backward_calculation_speed.plot(
        *longitudinal_controller_node.target_path_header_, v_backward_calculation, *path_);
    plot_forward_calculation_speed.plot(
        *longitudinal_controller_node.target_path_header_, v_forward_calculation, *path_);
    plot_curvature.plot(*longitudinal_controller_node.target_path_header_, curvature, *path_);
  }

  return speed;
}


void CurvatureControllerDebug::calculate_curvature(const common::Path<>& path) {

  curvature.clear();
  const double s_begin = path.front().arc_length;
  const double s_end = path.back().arc_length;

  for (double s = s_begin; s <= s_end; s += params->stepsize) {
    curvature.push_back({std::fabs(path.curvature(s)), s});
  }
}
