#include "longitudinal_control_params.h"

const ParameterString<double> LongitudinalControlParams::WHEELBASE(
    "/car_specs/wheelbase");
const ParameterString<double> LongitudinalControlParams::MASS(
    "/car_specs/mass");

const ParameterString<double> LongitudinalControlParams::V_MIN_CURVATURE_CONTROL(
    "v_min_cc");
const ParameterString<double> LongitudinalControlParams::V_MAX_CURVATURE_CONTROL(
    "v_max_cc");
const ParameterString<double> LongitudinalControlParams::V_MAX_CURVATURE_CONTROL_REVERSE(
    "v_max_cc_reverse");
const ParameterString<double> LongitudinalControlParams::A_KRIT("a_krit");
const ParameterString<double> LongitudinalControlParams::LOOKAHEAD_CC_T(
    "lookahead_cc_t");

const ParameterString<double> LongitudinalControlParams::MAX_ACC("max_acc");
const ParameterString<double> LongitudinalControlParams::MAX_DEC("max_dec");

const ParameterString<double> LongitudinalControlParams::CONSTANT_TIME_GAP(
    "acc/constant_time_gap");
const ParameterString<double> LongitudinalControlParams::DISTANCE_TIME_CONSTANT(
    "acc/distance_time_constant");
const ParameterString<double> LongitudinalControlParams::CONSTANT_DISTANCE_GAP(
    "acc/constant_distance_gap");
const ParameterString<double> LongitudinalControlParams::DYNAMIC_OBSTACLE_VELOCITY(
    "acc/dynamic_obstacle_velocity");
const ParameterString<bool> LongitudinalControlParams::V_OBSTACLE_PARAM(
    "acc/v_obstacle_param");
const ParameterString<int> LongitudinalControlParams::SMOOTHENING_RANGE(
    "acc/smoothening_range");
const ParameterString<int> LongitudinalControlParams::SMOOTHENING_STYLE(
    "acc/smoothening_style");
const ParameterString<double> LongitudinalControlParams::V_REL_RECURSIVE_PARAM(
    "acc/v_rel_recursive_param");
const ParameterString<double> LongitudinalControlParams::STOPPING_RADIUS(
    "stopping_controller/stopping_radius");
const ParameterString<double> LongitudinalControlParams::STOPPING_COMPLETION_THRESHOLD_SPEED(
    "stopping_controller/stopping_completion_threshold_speed");
const ParameterString<double> LongitudinalControlParams::STOPPING_DECELERATION(
    "stopping_controller/stopping_deceleration");
const ParameterString<double> LongitudinalControlParams::LOOKAHEAD_STOP_T(
    "stopping_controller/lookahead_stop_t");
const ParameterString<double> LongitudinalControlParams::V_MIN_ABS("v_min_abs");
const ParameterString<double> LongitudinalControlParams::STEPSIZE("stepsize");
const ParameterString<double> LongitudinalControlParams::V_EXTRAPOLATION(
    "v_extrapolation");
const ParameterString<double> LongitudinalControlParams::ANGLE_KRIT(
    "angle_krit");
const ParameterString<double> LongitudinalControlParams::ANGLE_KRIT_K(
    "angle_krit_k");
const ParameterString<double> LongitudinalControlParams::STEPSIZE_FORWARD_CALCULATION(
    "stepsize_forward_calculation");


LongitudinalControlParams::LongitudinalControlParams(ParameterInterface *parameters) {

  static bool register_params = true;
  if (register_params) {
    registerParams(parameters);
    register_params = false;
  }

  wheelbase = parameters->getParam(WHEELBASE);
  mass = parameters->getParam(MASS);

  v_min_curvature_control = parameters->getParam(V_MIN_CURVATURE_CONTROL);
  v_max_curvature_control = parameters->getParam(V_MAX_CURVATURE_CONTROL);
  v_max_curvature_control_reverse = parameters->getParam(V_MAX_CURVATURE_CONTROL_REVERSE);
  a_krit = parameters->getParam(A_KRIT);
  lookahead_cc_t = parameters->getParam(LOOKAHEAD_CC_T);
  max_acc = parameters->getParam(MAX_ACC);
  max_dec = parameters->getParam(MAX_DEC);
  v_min_abs = parameters->getParam(V_MIN_ABS);
  stepsize = parameters->getParam(STEPSIZE);
  v_extrapolation = parameters->getParam(V_EXTRAPOLATION);
  angle_krit = parameters->getParam(ANGLE_KRIT);
  angle_krit_k = parameters->getParam(ANGLE_KRIT_K);
  stepsize_forward_calculation = parameters->getParam(STEPSIZE_FORWARD_CALCULATION);

  //! ACC parameters
  constant_time_gap = parameters->getParam(CONSTANT_TIME_GAP);
  distance_time_constant = parameters->getParam(DISTANCE_TIME_CONSTANT);
  constant_distance_gap = parameters->getParam(CONSTANT_DISTANCE_GAP);
  dynamic_obstacle_velocity = parameters->getParam(DYNAMIC_OBSTACLE_VELOCITY);
  v_obstacle_param = parameters->getParam(V_OBSTACLE_PARAM);
  smoothening_range = parameters->getParam(SMOOTHENING_RANGE);
  smoothening_style = parameters->getParam(SMOOTHENING_STYLE);
  v_rel_recursive_param = parameters->getParam(V_REL_RECURSIVE_PARAM);

  //! Stopping controller parameters
  stopping_radius = parameters->getParam(STOPPING_RADIUS);
  stopping_completion_threshold_speed =
      parameters->getParam(STOPPING_COMPLETION_THRESHOLD_SPEED);
  stopping_deceleration = parameters->getParam(STOPPING_DECELERATION);
  lookahead_stop_t = parameters->getParam(LOOKAHEAD_STOP_T);
}


void LongitudinalControlParams::registerParams(ParameterInterface *parameters) {
  parameters->registerParam(WHEELBASE);
  parameters->registerParam(MASS);

  parameters->registerParam(V_MIN_CURVATURE_CONTROL);
  parameters->registerParam(V_MAX_CURVATURE_CONTROL);
  parameters->registerParam(V_MAX_CURVATURE_CONTROL_REVERSE);
  parameters->registerParam(A_KRIT);
  parameters->registerParam(LOOKAHEAD_CC_T);
  parameters->registerParam(MAX_ACC);
  parameters->registerParam(MAX_DEC);
  parameters->registerParam(V_MIN_ABS);
  parameters->registerParam(STEPSIZE);
  parameters->registerParam(V_EXTRAPOLATION);
  parameters->registerParam(ANGLE_KRIT);
  parameters->registerParam(ANGLE_KRIT_K);
  parameters->registerParam(STEPSIZE_FORWARD_CALCULATION);

  //! ACC parameters
  parameters->registerParam(CONSTANT_TIME_GAP);
  parameters->registerParam(DISTANCE_TIME_CONSTANT);
  parameters->registerParam(CONSTANT_DISTANCE_GAP);
  parameters->registerParam(DYNAMIC_OBSTACLE_VELOCITY);
  parameters->registerParam(V_OBSTACLE_PARAM);
  parameters->registerParam(SMOOTHENING_RANGE);
  parameters->registerParam(SMOOTHENING_STYLE);
  parameters->registerParam(V_REL_RECURSIVE_PARAM);

  //! Stopping controller parameters
  parameters->registerParam(STOPPING_RADIUS);
  parameters->registerParam(STOPPING_COMPLETION_THRESHOLD_SPEED);
  parameters->registerParam(STOPPING_DECELERATION);
  parameters->registerParam(LOOKAHEAD_STOP_T);
}
