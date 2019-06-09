#include "lateral_control.h"
#include <common/macros.h>

THIRD_PARTY_HEADERS_BEGIN
#include <ros/ros.h>
#include <cmath>
#include <boost/algorithm/clamp.hpp>
THIRD_PARTY_HEADERS_END

#include "common/math.h"
#include "common/angle_conversions.h"

const ParameterString<double> LateralControl::Params::WHEELBASE(
    "/car_specs/wheelbase");
const ParameterString<double> LateralControl::Params::MAX_STEERING_ANGLE_LEFT(
    "/car_specs/max_steering_angle_left");
const ParameterString<double> LateralControl::Params::MAX_STEERING_ANGLE_RIGHT(
    "/car_specs/max_steering_angle_right");

const ParameterString<double> LateralControl::Params::DIST_K("dist_k");
const ParameterString<double> LateralControl::Params::LOOKAHEAD_DELTA(
    "lookahead_delta");
const ParameterString<double> LateralControl::Params::EPSILON_DIVISION_BY_ZERO(
    "epsilon_division_by_zero");

const ParameterString<double> LateralControl::Params::K_AG("k_ag");
const ParameterString<double> LateralControl::Params::K_YAW("k_yaw");
const ParameterString<double> LateralControl::Params::MIN_LATERAL_CONTROL_SPEED(
    "min_lateral_control_speed");


LateralControl::Params::Params(const ParameterInterface *parameters) {
  wheelbase = parameters->getParam(WHEELBASE);
  max_steering_angle_left = parameters->getParam(MAX_STEERING_ANGLE_LEFT);
  max_steering_angle_right = parameters->getParam(MAX_STEERING_ANGLE_RIGHT);

  dist_k = parameters->getParam(DIST_K);
  lookahead_delta = parameters->getParam(LOOKAHEAD_DELTA);
  epsilon_division_by_zero = parameters->getParam(EPSILON_DIVISION_BY_ZERO);
  k_ag = parameters->getParam(K_AG);
  k_yaw = parameters->getParam(K_YAW);

  min_control_speed = parameters->getParam(MIN_LATERAL_CONTROL_SPEED);
}


void LateralControl::Params::registerParams(ParameterInterface *parameters) {
  parameters->registerParam(WHEELBASE);
  parameters->registerParam(MAX_STEERING_ANGLE_LEFT);
  parameters->registerParam(MAX_STEERING_ANGLE_RIGHT);

  parameters->registerParam(DIST_K);
  parameters->registerParam(K_AG);
  parameters->registerParam(K_YAW);
  parameters->registerParam(EPSILON_DIVISION_BY_ZERO);
  parameters->registerParam(LOOKAHEAD_DELTA);

  parameters->registerParam(MIN_LATERAL_CONTROL_SPEED);
}


LateralControl::LateralControl(ParameterInterface *parameters)
    : params([parameters]() {
        Params::registerParams(parameters);
        return Params(parameters);
      }()),
      parameters_ptr_(parameters) {}

void LateralControl::generateSteeringAngle(const common::Path<> &path,
                                           const Eigen::Affine2d &vehicle_pose,
                                           const double measured_speed_x,
                                           const double /*measured_speed_y*/,
                                           const double measured_yaw_rate,
                                           bool driving_reverse,
                                           double &steering_angle_front,
                                           double &steering_angle_back,
                                           double &lookahead_distance) {

  if (!path.isArcLengthInRange(0.0)) {
    // this should never happen, because lotfusspunkt should be on path
    ROS_ERROR_THROTTLE(
        4, "extrapolating path in lateral control at vehicle foot point");
  }

  double dist = (path(0.0) - vehicle_pose.translation()).norm();

  //! Are we on the left or right side of the trajectory?
  if ((vehicle_pose.translation().y() > path.getY(0.0)) != driving_reverse) {
    // rechts von der Kurve muss der Distanz Term negativ sein,
    // links positiv
    dist = -dist;
  }


  //! Calculate error in orientation between front_axle and path (tangential)
  lookahead_distance = measured_speed_x * params.lookahead_delta;

  if (!path.isArcLengthInRange(lookahead_distance)) {
    ROS_WARN_THROTTLE(
        4, "extrapolating path in lateral control at lookahead point");
  }

  // the angle of the path at the lookahead point
  const double psi_t = std::atan(path.firstDerivative(lookahead_distance));
  const double psi = common::toYaw(vehicle_pose.rotation());
  const double yaw_to_path = psi_t - psi;

  const double yaw_rate_path = measured_speed_x * path.curvature(lookahead_distance);
  const double yaw_steady_state = params.k_ag * measured_speed_x * yaw_rate_path;

  const double weighted_dist_error =
      params.dist_k * dist / (std::abs(measured_speed_x) + params.epsilon_division_by_zero);

  const double yaw_rate_damping = params.k_yaw * (yaw_rate_path - measured_yaw_rate);


  /**************************
   * HIER STECKT DER REGLER *
   **************************/
  /*!
   * New steering angle results from
   * - ideal steering angle necessary to drive through curve
   * - compensation for orientation error
   * - compensation for distance error
   * - damping of yaw rate for stability at high speeds
   */
  double delta = 0.0;
  if (std::fabs(measured_speed_x) > params.min_control_speed) {
    delta = yaw_to_path - yaw_steady_state + std::atan(weighted_dist_error) + yaw_rate_damping;
  } else {
    delta = yaw_to_path;
  }

  const double steering_angle = boost::algorithm::clamp(
      delta, -params.max_steering_angle_right, params.max_steering_angle_left);

  if (driving_reverse) {
    steering_angle_front = 0.0;
    steering_angle_back = steering_angle;
  } else {
    steering_angle_front = steering_angle;
    steering_angle_back = 0.0;
  }
}

void LateralControl::updateParams() { params = Params(parameters_ptr_); }
