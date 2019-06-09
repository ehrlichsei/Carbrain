#ifndef LONGITUDINAL_CONTROL_PARAMS_H
#define LONGITUDINAL_CONTROL_PARAMS_H


#include "common/parameter_interface.h"

struct LongitudinalControlParams {
  static const ParameterString<double> WHEELBASE;
  static const ParameterString<double> MASS;
  static const ParameterString<double> V_MIN_CURVATURE_CONTROL;
  static const ParameterString<double> V_MAX_CURVATURE_CONTROL;
  static const ParameterString<double> V_MAX_CURVATURE_CONTROL_REVERSE;
  static const ParameterString<double> A_KRIT;
  static const ParameterString<double> LOOKAHEAD_CC_T;
  static const ParameterString<double> LOOKAHEAD_STOP_T;
  static const ParameterString<double> MAX_ACC;
  static const ParameterString<double> MAX_DEC;
  static const ParameterString<double> CONSTANT_TIME_GAP;
  static const ParameterString<double> DISTANCE_TIME_CONSTANT;
  static const ParameterString<double> CONSTANT_DISTANCE_GAP;
  static const ParameterString<double> DYNAMIC_OBSTACLE_VELOCITY;
  static const ParameterString<bool> V_OBSTACLE_PARAM;
  static const ParameterString<int> SMOOTHENING_RANGE;
  static const ParameterString<int> SMOOTHENING_STYLE;
  static const ParameterString<double> V_REL_RECURSIVE_PARAM;
  static const ParameterString<double> STOPPING_RADIUS;
  static const ParameterString<double> STOPPING_COMPLETION_THRESHOLD_SPEED;
  static const ParameterString<double> STOPPING_DECELERATION;
  static const ParameterString<double> V_MIN_ABS;
  static const ParameterString<double> STEPSIZE;
  static const ParameterString<double> V_EXTRAPOLATION;
  static const ParameterString<double> ANGLE_KRIT;
  static const ParameterString<double> ANGLE_KRIT_K;
  static const ParameterString<double> STEPSIZE_FORWARD_CALCULATION;

  double wheelbase;
  double mass;
  double v_min_curvature_control;
  double v_max_curvature_control;
  double v_max_curvature_control_reverse;
  double a_krit;
  double lookahead_cc_t;
  double lookahead_stop_t;
  double max_acc;
  double max_dec;
  double constant_time_gap;
  double distance_time_constant;
  double constant_distance_gap;
  double dynamic_obstacle_velocity;
  bool v_obstacle_param;
  int smoothening_range;
  int smoothening_style;
  double v_rel_recursive_param;
  double stopping_radius;
  double stopping_completion_threshold_speed;
  double stopping_deceleration;
  double v_min_abs;
  double stepsize;
  double v_extrapolation;
  double angle_krit;
  double angle_krit_k;
  double stepsize_forward_calculation;

  LongitudinalControlParams(ParameterInterface* parameters);

  static void registerParams(ParameterInterface* parameters);
};

#endif  // LONGITUDINAL_CONTROL_PARAMS_H
