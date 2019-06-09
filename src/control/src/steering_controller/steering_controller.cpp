#include "steering_controller.h"
#include <boost/algorithm/clamp.hpp>
#include "common/realtime_timings.h"
#include "common/ros_chrono_conversions.h"

THIRD_PARTY_HEADERS_BEGIN
#include <cmath>
THIRD_PARTY_HEADERS_END

SteeringController::SteeringController(ParameterInterface *parameters)
    : parameters_ptr_(parameters) {
  registerParameters(parameters);
  setParams();
}


SteeringControllerRealtime::SteeringControllerRealtime(ParameterInterface *parameters,
                                                       int channel_id_steering_command,
                                                       int channel_id_steering_angle_measure)
    : RealtimeController(RealtimeTimings::LOOP_RATE,
                         RealtimeTimings::LOWLEVEL_CONTROLLER_OFFSET,
                         parameters),
      SteeringController(parameters),
      steering_angle_measure_(channel_id_steering_angle_measure),
      manual_mode_(CHANNEL_ID_MANUAL_MODE),
      steering_control_(channel_id_steering_command) {
  ROS_INFO(
      "started steering_controller with the steering output IPC "
      "channel_id %#x and the steering angle measure IPC channel_id %#x",
      channel_id_steering_command,
      channel_id_steering_angle_measure);

  assert((channel_id_steering_command == CHANNEL_ID_STEERING_SERVO_OUTPUT ||
          channel_id_steering_command == CHANNEL_ID_STEERING_BACK_SERVO_OUTPUT) &&
         "invalid channel_id for the steering output realtime IPC");
}


void SteeringController::registerParameters(ParameterInterface *parameters) {
  parameters->registerParam(PARAM_P_STEERING);
  parameters->registerParam(PARAM_I_STEERING);
  parameters->registerParam(PARAM_D_STEERING);
  parameters->registerParam(PARAM_I_MIN);
  parameters->registerParam(PARAM_I_MAX);
  parameters->registerParam(PARAM_FEED_FORWARD_A);
  parameters->registerParam(PARAM_FEED_FORWARD_B);
  parameters->registerParam(PARAM_FEED_FORWARD_C);
  parameters->registerParam(PARAM_USE_ANTIWINDUP);
  parameters->registerParam(PARAM_FEED_FORWARD_D);
}

SteeringController::SteeringControlCommands SteeringController::calcSteeringCommands(
    boost::chrono::nanoseconds time_since_last_update, float current_steering_angle, bool manual_mode) {

  SteeringControlCommands steering_control_commands;

  if (manual_mode) {
    pid_.reset();
  } else {
    float steering_output = 0.f;
    if (std::isnan(current_steering_angle)) {
      steering_output = 0.f;
    } else {
      const float angle_error = target_angle_ - current_steering_angle;
      const float pid_output =
          pid_.computeCommand(angle_error, common::fromBoost(time_since_last_update));

      steering_output = pid_output;

      steering_control_commands.angle_error = angle_error;
    }
    steering_output += feed_forward_.evaluate(target_angle_);
    // steering_output += 0.5f;
    steering_output = boost::algorithm::clamp(steering_output, 0.f, 1.f);
    steering_control_commands.steering_output = steering_output;
  }
  return steering_control_commands;
}

void SteeringController::setParams() {
  pid_.setGains(parameters_ptr_->getParam(PARAM_P_STEERING),
                parameters_ptr_->getParam(PARAM_I_STEERING),
                parameters_ptr_->getParam(PARAM_D_STEERING),
                parameters_ptr_->getParam(PARAM_I_MAX),
                parameters_ptr_->getParam(PARAM_I_MIN),
                parameters_ptr_->getParam(PARAM_USE_ANTIWINDUP));
  common::CubicPolynomial::CoefficientList feed_forward_coefficients = {
      {parameters_ptr_->getParam(PARAM_FEED_FORWARD_A),
       parameters_ptr_->getParam(PARAM_FEED_FORWARD_B),
       parameters_ptr_->getParam(PARAM_FEED_FORWARD_C),
       parameters_ptr_->getParam(PARAM_FEED_FORWARD_D)}};
  feed_forward_ = common::CubicPolynomial(feed_forward_coefficients);
}


void SteeringControllerRealtime::update(boost::chrono::nanoseconds time_since_last_update) {
  const ros::WallTime start_cycle = ros::WallTime::now();

  const SteeringControlCommands steering_control_commands = calcSteeringCommands(
      time_since_last_update, *steering_angle_measure_.get(), *manual_mode_.get());


  steering_control_.write(steering_control_commands.steering_output);

  servo_set_value_publisher_queue_.push(steering_control_commands.steering_output);

  if (steering_control_commands.angle_error != boost::none) {
    angle_error_publisher_queue_.push(*(steering_control_commands.angle_error));
  }
  const ros::WallTime end_cycle = ros::WallTime::now();
  const int cycle_time = (end_cycle - start_cycle).toNSec() / 1000;
  if (cycle_time > RealtimeTimings::LOWLEVEL_CONTROLLER_CYCLE_TIME) {
    ROS_WARN_THROTTLE(1, "cycle_time is too big: %dus!", cycle_time);
  }
}

void SteeringControllerRealtime::pollParameters() { setParams(); }

void SteeringController::setTargetAngle(float target_angle) {
  target_angle_ = target_angle;
}

void SteeringController::reset() {
  pid_.reset();
  target_angle_ = 0.0;
}

void SteeringControllerRealtime::reset() {
  SteeringController::reset();
  steering_control_.write(0.5);
}

void SteeringControllerRealtime::start() {
  RealtimeController::startControlThread();
}

void SteeringControllerRealtime::stop() {
  RealtimeController::stopControlThread();
}

bool SteeringController::getNextServoSetValue(double *servo_set_value) {
  return servo_set_value_publisher_queue_.pop(servo_set_value);
}

bool SteeringController::getNextAngleError(double *angle_error) {
  return angle_error_publisher_queue_.pop(angle_error);
}
