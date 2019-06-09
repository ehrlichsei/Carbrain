#include "speed_controller.h"
#include <common/math.h>
#include <boost/algorithm/clamp.hpp>
#include <cmath>
#include "common/realtime_timings.h"
#include "common/ros_chrono_conversions.h"

SpeedController::SpeedController(ParameterInterface *parameters)
    : last_velocity_command_timestamp(ros::Time::now()), parameters_ptr_(parameters) {
  registerParameters(parameters);
}

SpeedControllerRealtime::SpeedControllerRealtime(ParameterInterface *parameters)
    : RealtimeController(RealtimeTimings::LOOP_RATE,
                         RealtimeTimings::LOWLEVEL_CONTROLLER_OFFSET,
                         parameters),
      SpeedController(parameters),
      speed_measure_(CHANNEL_ID_SPEED_MEASURE),
      acceleration_measure_(CHANNEL_ID_ACCELERATION_MEASURE),
      manual_mode_(CHANNEL_ID_MANUAL_MODE),
      engine_power_(CHANNEL_ID_ENGINE_POWER),
      brake_lights_(CHANNEL_ID_BRAKE_LIGHTS) {
  SpeedController::setParams();
}

void SpeedController::registerParameters(ParameterInterface *parameters) {
  parameters->registerParam(PARAM_P_SPEED);
  parameters->registerParam(PARAM_I_SPEED);
  parameters->registerParam(PARAM_D_SPEED);
  parameters->registerParam(PARAM_I_MIN);
  parameters->registerParam(PARAM_I_MAX);
  parameters->registerParam(PARAM_FEED_FORWARD_A);
  parameters->registerParam(PARAM_FEED_FORWARD_B);
  parameters->registerParam(PARAM_FEED_FORWARD_C);
  parameters->registerParam(PARAM_MAX_ENGINE_POWER);
  parameters->registerParam(PARAM_USE_ANTIWINDUP);
  parameters->registerParam(PARAM_DEAD_ZONE_SIZE);
  parameters->registerParam(PARAM_DEAD_ZONE_SPEED_SET_POINT_ACTIVATION_THRESHOLD);
  parameters->registerParam(PARAM_VELOCITY_COMMAND_TIMEOUT);
}

SpeedController::SpeedControllerCommands SpeedController::calcSpeedCommands(
    boost::chrono::nanoseconds time_since_last_update, float current_speed, bool manual_mode) {

  if (emergency_stop_) {
    target_speed_ = 0.0;
  }

  const float speed_error = target_speed_ - current_speed;

  float engine_power = 0.0;
  bool enable_brake_lights = false;

  if (manual_mode) {
    pid_.reset();
    emergency_stop_ = false;
  } else {
    const float pid_output =
        pid_.computeCommand(speed_error, common::fromBoost(time_since_last_update));
    if (std::abs(target_speed_) <= dead_zone_speed_set_point_activation_threshold &&
        std::abs(speed_error) < dead_zone_size) {
      engine_power = 0.0;
      enable_brake_lights = true;
      pid_.reset();
    } else if (common::sgn(pid_output) == common::sgn(target_speed_)) {
      // accelerating
      const float feed_forward_output = feed_forward_.evaluate(target_speed_);
      engine_power = feed_forward_output + pid_output;
      enable_brake_lights = false;
    } else {  // decelerating
      engine_power = pid_output;
      enable_brake_lights = true;
    }
  }

  SpeedControllerCommands speed_controller_commands;
  speed_controller_commands.engine_power =
      boost::algorithm::clamp(engine_power, -max_engine_power, max_engine_power);
  speed_controller_commands.enable_brake_lights = enable_brake_lights;
  speed_controller_commands.speed_error = speed_error;

  return speed_controller_commands;
}

void SpeedControllerRealtime::update(boost::chrono::nanoseconds time_since_last_update) {
  // todooo read ipc values übergebe direct calca
  const ros::WallTime start_cycle = ros::WallTime::now();

  if (ros::Time::now() - last_velocity_command_timestamp >
      ros::Duration(velocity_command_timeout)) {
    target_speed_ = 0.0;
    ROS_ERROR_THROTTLE(4,
                       "velocity command in speed controller older than %lf "
                       "seconds: timeout, stopping vehicle",
                       velocity_command_timeout);
  }

  const SpeedControllerCommands speed_controller_commands = calcSpeedCommands(
      time_since_last_update, *speed_measure_.get(), *manual_mode_.get());


  engine_power_.write(speed_controller_commands.engine_power);
  brake_lights_.write(speed_controller_commands.enable_brake_lights);

  engine_power_publisher_queue_.push(speed_controller_commands.engine_power);
  speed_error_publisher_queue_.push(speed_controller_commands.speed_error);
  const ros::WallTime end_cycle = ros::WallTime::now();
  const int cycle_time = (end_cycle - start_cycle).toNSec() / 1000;
  if (cycle_time > RealtimeTimings::LOWLEVEL_CONTROLLER_CYCLE_TIME) {
    ROS_WARN_THROTTLE(1, "cycle_time too big: %dus!", cycle_time);
  }
}

void SpeedController::setParams() {
  pid_.setGains(parameters_ptr_->getParam(PARAM_P_SPEED),
                parameters_ptr_->getParam(PARAM_I_SPEED),
                parameters_ptr_->getParam(PARAM_D_SPEED),
                parameters_ptr_->getParam(PARAM_I_MAX),
                parameters_ptr_->getParam(PARAM_I_MIN),
                parameters_ptr_->getParam(PARAM_USE_ANTIWINDUP));

  common::QuadraticPolynomial::CoefficientList feed_forward_coefficients = {
      {parameters_ptr_->getParam(PARAM_FEED_FORWARD_A),
       parameters_ptr_->getParam(PARAM_FEED_FORWARD_B),
       parameters_ptr_->getParam(PARAM_FEED_FORWARD_C)}};

  feed_forward_ = common::QuadraticPolynomial(feed_forward_coefficients);

  dead_zone_size = parameters_ptr_->getParam(PARAM_DEAD_ZONE_SIZE);
  dead_zone_speed_set_point_activation_threshold =
      parameters_ptr_->getParam(PARAM_DEAD_ZONE_SPEED_SET_POINT_ACTIVATION_THRESHOLD);

  max_engine_power = parameters_ptr_->getParam(PARAM_MAX_ENGINE_POWER);

  velocity_command_timeout = parameters_ptr_->getParam(PARAM_VELOCITY_COMMAND_TIMEOUT);
}

void SpeedControllerRealtime::pollParameters() { setParams(); }

void SpeedControllerRealtime::start() {
  RealtimeController::startControlThread();
}

void SpeedControllerRealtime::stop() {
  RealtimeController::stopControlThread();
}

void SpeedController::setTargetSpeed(float target_speed,
                                     const ros::Time &velocity_command_timestamp) {
  target_speed_ = target_speed;
  last_velocity_command_timestamp = velocity_command_timestamp;
}

void SpeedController::setEmergencyStop(bool stop) { emergency_stop_ = stop; }

void SpeedController::reset() {
  pid_.reset();
  target_speed_ = 0.0;
  last_velocity_command_timestamp = ros::Time::now();
}

void SpeedControllerRealtime::reset() {
  SpeedController::reset();
  engine_power_.write(0.0);
}

bool SpeedController::getNextEnginePower(double *engine_power) {
  return engine_power_publisher_queue_.pop(engine_power);
}

bool SpeedController::getNextSpeedError(double *speed_error) {
  return speed_error_publisher_queue_.pop(speed_error);
}
