#include "state_estimation.h"
#include <common/macros.h>
#include <common/math.h>

THIRD_PARTY_HEADERS_BEGIN
#include <cmath>
#include <ros/ros.h>
#include <cmath>
#include <limits>
THIRD_PARTY_HEADERS_END

#include "kalman_filter/linear_kalman_filter/linear_2ws/linear_2ws_kalman_filter.h"
#include "kalman_filter/linear_kalman_filter/linear_2ws/linear_2ws_kalman_filter_debug.h"
#include "kalman_filter/linear_kalman_filter/linear_4ws_fbs/linear_4ws_front_back_speed_kalman_filter.h"
#include "kalman_filter/linear_kalman_filter/linear_4ws_fbs/linear_4ws_front_back_speed_kalman_filter_debug.h"

#include <common/realtime_channel_ids.h>
#include <common/realtime_timings.h>

const ParameterString<double> StateEstimation::MAX_SPEED("max_speed");
const ParameterString<double> StateEstimation::MAX_YAW_RATE("max_yaw_rate");
const ParameterString<double> StateEstimation::MAX_ACCELERATION(
    "max_acceleration");
const ParameterString<double> StateEstimation::MAX_WHEEL_SPEED_MEASURE(
    "max_wheel_speed_measure");
const ParameterString<double> StateEstimation::MAX_YAW_RATE_MEASURE(
    "max_yaw_rate_measure");
const ParameterString<double> StateEstimation::MAX_ACCELERATION_MEASURE(
    "max_acceleration_measure");
const ParameterString<double> StateEstimation::MAX_STEERING_ANGLE_MEASURE(
    "max_steering_angle_measure");

using common::squared;
#define NUM_RUNTIME_MEASUREMENTS 10000

StateEstimation::StateEstimation(ParameterInterface* parameters,
                                 std::unique_ptr<KalmanFilter> kalman_filter)
    : runtime_analyzer_("state_estimation", 2, NUM_RUNTIME_MEASUREMENTS, parameters),
      kalman_filter_(std::move(kalman_filter)),
      parameters_ptr_(parameters) {

  runtime_analyzer_.registerPeriodName("wait");
  runtime_analyzer_.registerPeriodName("compute");
  runtime_analyzer_.startMeasuringIfEnabled();

  registerStateEstimationParameters(parameters);

  updateParameters();
}

StateEstimationRealtime::StateEstimationRealtime(ParameterInterface* parameters)
    : RealtimeController(
          RealtimeTimings::LOOP_RATE, RealtimeTimings::STATE_ESTIMATION_OFFSET, parameters),
      StateEstimation(
          parameters,
          [parameters]() -> std::unique_ptr<KalmanFilter> {
            ParameterString<std::string> KALMAN_FILTER_NAME(
                "kalman_filter_name");
            parameters->registerParam(KALMAN_FILTER_NAME);
            std::string kalman_filter_name = parameters->getParam(KALMAN_FILTER_NAME);
            if (kalman_filter_name == "linear_2ws") {
              return std::make_unique<Linear2WSKalmanFilter>(
                  RealtimeTimings::LOOP_RATE, *parameters);
            } else if (kalman_filter_name == "linear_4ws_fbs") {
              return std::make_unique<Linear4WSFrontBackSpeedKalmanFilter>(
                  RealtimeTimings::LOOP_RATE, *parameters);
            } else {
              throw std::runtime_error(kalman_filter_name +
                                       " is not a valid kalman_filter_name");
            }
          }()),
      sensor_measurements_(CHANNEL_ID_SENSOR_MEASUREMENTS),
      front_servo_command_ipc(CHANNEL_ID_STEERING_SERVO_OUTPUT),
      back_servo_command_ipc(CHANNEL_ID_STEERING_BACK_SERVO_OUTPUT),
      engine_command_ipc(CHANNEL_ID_ENGINE_POWER),
      estimated_speed_(CHANNEL_ID_SPEED_MEASURE),
      estimated_yaw_rate_(CHANNEL_ID_YAW_RATE_MEASURE),
      estimated_acceleration_(CHANNEL_ID_ACCELERATION_MEASURE),
      estimated_steering_angle_front(CHANNEL_ID_STEERING_ANGLE_MEASURE_FRONT),
      estimated_steering_angle_back(CHANNEL_ID_STEERING_ANGLE_MEASURE_BACK) {
  SensorMeasurements zero;
  sensor_measurements_.write(zero);
  front_servo_command_ipc.write(0.5f);
  back_servo_command_ipc.write(0.5f);
  engine_command_ipc.write(0.f);
}

StateEstimationDebug::StateEstimationDebug(ParameterInterface* parameters,
                                           ros::NodeHandle* node_handle)
    : StateEstimation(
          parameters,
          [&parameters, &node_handle]() -> std::unique_ptr<KalmanFilter> {
            ParameterString<std::string> KALMAN_FILTER_NAME(
                "kalman_filter_name");
            parameters->registerParam(KALMAN_FILTER_NAME);
            std::string kalman_filter_name = parameters->getParam(KALMAN_FILTER_NAME);
            if (kalman_filter_name == "linear_2ws") {
              return std::make_unique<Linear2WSKalmanFilterDebug>(
                  getUpdateRateParameter(), *parameters, node_handle);
            } else if (kalman_filter_name == "linear_4ws_fbs") {
              return std::make_unique<Linear4WSFrontBackSpeedKalmanFilterDebug>(
                  getUpdateRateParameter(), *parameters, node_handle);
            } else {
              throw std::runtime_error(
                  kalman_filter_name +
                  " is not a valid kalman_filter_name or the " + kalman_filter_name +
                  " kalman filter doesn't implement a debug class");
            }
          }()) {}

void StateEstimation::registerStateEstimationParameters(ParameterInterface* parameters) {
  parameters->registerParam(MAX_SPEED);
  parameters->registerParam(MAX_YAW_RATE);
  parameters->registerParam(MAX_ACCELERATION);
  parameters->registerParam(MAX_WHEEL_SPEED_MEASURE);
  parameters->registerParam(MAX_YAW_RATE_MEASURE);
  parameters->registerParam(MAX_ACCELERATION_MEASURE);
  parameters->registerParam(MAX_STEERING_ANGLE_MEASURE);
}

void StateEstimation::setStateEstimationParameters(const ParameterInterface* parameters) {
  max_speed = parameters->getParam(MAX_SPEED);
  max_yaw_rate = parameters->getParam(MAX_YAW_RATE);
  max_acceleration = parameters->getParam(MAX_ACCELERATION);
  max_wheel_speed_measure = parameters->getParam(MAX_WHEEL_SPEED_MEASURE);
  max_acceleration_measure = parameters->getParam(MAX_ACCELERATION_MEASURE);
  max_yaw_rate_measure = parameters->getParam(MAX_YAW_RATE_MEASURE);
  max_steering_angle_measure = parameters->getParam(MAX_STEERING_ANGLE_MEASURE);
}


bool StateEstimation::measurementsPlausible(const SensorMeasurements& measurements) const {
  auto check_wheel_speeds_axle = [this](const SensorMeasurements::Axle& wheel_speeds) {
    return (std::abs(wheel_speeds.left) < max_wheel_speed_measure) &&
           (std::abs(wheel_speeds.right) < max_wheel_speed_measure);
  };
  auto check_angular_velocity =
      [this](const SensorMeasurements::IMU::AngularVelocity& angular_velocity) {
        return (std::abs(angular_velocity.x) < max_yaw_rate_measure) &&
               (std::abs(angular_velocity.y) < max_yaw_rate_measure) &&
               (std::abs(angular_velocity.z) < max_yaw_rate_measure);
      };
  auto check_acceleration =
      [this](const SensorMeasurements::IMU::Acceleration& acceleration) {
        return (std::abs(acceleration.x) < max_acceleration_measure) &&
               (std::abs(acceleration.y) < max_acceleration_measure) &&
               (std::abs(acceleration.z) < max_acceleration_measure);
      };
  auto check_IMU =
      [check_angular_velocity, check_acceleration](const SensorMeasurements::IMU& imu) {
        return check_angular_velocity(imu.angular_velocity) &&
               check_acceleration(imu.acceleration);
      };
  auto check_steering_angles_axle = [this](const SensorMeasurements::Axle& steering_angles) {
    return (std::abs(steering_angles.left) < max_steering_angle_measure) &&
           (std::abs(steering_angles.right) < max_steering_angle_measure);
  };

  return check_IMU(measurements.left_IMU) && check_IMU(measurements.right_IMU) &&
         check_wheel_speeds_axle(measurements.angular_wheel_speeds.front) &&
         check_wheel_speeds_axle(measurements.angular_wheel_speeds.back) /*&&
         check_steering_angles_axle(measurements.steering_angles.front) &&
         check_steering_angles_axle(measurements.steering_angles.back)*/;
}


/**
 * @brief StateEstimation::update updates the kalman filter
 * @param time_since_last_update the time (in seconds) since the last update.
 *
 * This is runs in a real-time thread. Please insert only real-time capable
 * code.
 */
void StateEstimationRealtime::update(boost::chrono::nanoseconds /*time_since_last_update*/) {
  runtime_analyzer_.measureAndPushCheckPointIfActive();
  const ros::WallTime start_cycle = ros::WallTime::now();
  const SensorMeasurements* sensor_measurements = sensor_measurements_.get();
  const float front_servo_command = *front_servo_command_ipc.get();
  const float back_servo_command = *back_servo_command_ipc.get();
  const float engine_command = *engine_command_ipc.get();

  // get estimation
  VehicleState estimated_state = performKalmanIteration(
      *sensor_measurements, front_servo_command, back_servo_command, engine_command);

  // write results into realtime IPC
  estimated_speed_.write(estimated_state.speed_x);
  estimated_yaw_rate_.write(estimated_state.yaw_rate);
  estimated_acceleration_.write(estimated_state.acceleration);

  estimated_steering_angle_front.write(estimated_state.steering_angle_front);
  estimated_steering_angle_back.write(estimated_state.steering_angle_back);

  // make sure our result gets published on the ROS topic
  estimated_state_results_queue_.push(estimated_state);

  runtime_analyzer_.measureAndPushCheckPointIfActive();

  const ros::WallTime end_cycle = ros::WallTime::now();
  const int cycle_time = (end_cycle - start_cycle).toNSec() / 1000;
  if (cycle_time > RealtimeTimings::STATE_ESTIMATION_CYCLE_TIME) {
    ROS_WARN_THROTTLE(1, "cycle_time is too big: %dus!", cycle_time);
  }
}

void StateEstimationRealtime::pollParameters() { updateParameters(); }

int StateEstimationRealtime::getUpdateRate() const {
  return RealtimeController::getUpdateRate();
}

void StateEstimationRealtime::start() {
  StateEstimation::start();
  startControlThread();
}

void StateEstimationRealtime::stop() { stopControlThread(); }

// only relevant for integration tests: iterates over all measurements and
// calls
// performKalmanIteration to get estimations
std::vector<VehicleState> StateEstimation::processSensorData(
    const std::vector<std::tuple<SensorMeasurements, float, float, float>>& test_data) {
  updateParameters();
  kalman_filter_->reset();
  std::vector<VehicleState> intermediate_states;
  intermediate_states.reserve(test_data.size() + 1);
  intermediate_states.emplace_back();
  std::transform(test_data.begin(),
                 test_data.end(),
                 std::back_inserter(intermediate_states),
                 [this](auto data) {
                   return this->performKalmanIteration(std::get<0>(data),
                                                       std::get<1>(data),
                                                       std::get<2>(data),
                                                       std::get<3>(data));
                 });
  return intermediate_states;
}

// the heart of the state estimation: performKalmanIteration calls the
// prediction and innovation step of the kalman filter, checks the estimation
// for plausibility and resets the kalman filter if necessary
VehicleState StateEstimation::performKalmanIteration(const SensorMeasurements& measure,
                                                     float front_servo_command,
                                                     float back_servo_command,
                                                     float engine_command) {
  VehicleState estimated_state;
  static VehicleState last_estimated_state;

  estimated_state =
      kalman_filter_->predict(front_servo_command, back_servo_command, engine_command);


  if (measurementsPlausible(measure)) {
    estimated_state =
        kalman_filter_->innovate(measure, front_servo_command, back_servo_command);
  } else {
    ROS_WARN("Implausible measurements, skipping innovation step.");
    ROS_WARN("measurements:\n%s", to_string(measure).c_str());
  }

  estimated_state.stamp = measure.stamp;

  // plausibility checks
  if (std::fabs(std::sqrt(squared(estimated_state.speed_x) +
                          squared(estimated_state.speed_y))) < max_speed &&
      std::fabs(estimated_state.yaw_rate) < max_yaw_rate &&
      std::fabs(estimated_state.acceleration) < max_acceleration) {
    last_estimated_state = estimated_state;
    return estimated_state;
  } else {
    kalman_filter_->reset();
    ROS_WARN("Invalid state. Reset state.");
    return last_estimated_state;
  }
}

bool StateEstimation::getNextEstimatedState(VehicleState* state) {
  return estimated_state_results_queue_.pop(state);
}

void StateEstimation::updateParameters() {
  kalman_filter_->updateParams();
  setStateEstimationParameters(parameters_ptr_);
}

void StateEstimation::resetKalmanFilter() { kalman_filter_->reset(); }

void StateEstimation::start() {
  updateParameters();
  resetKalmanFilter();
}


int StateEstimationDebug::getUpdateRateParameter() {
  int debug_update_rate;
  ros::NodeHandle nh("~");
  if (!nh.getParam("debug_measurement_rate", debug_update_rate)) {
    throw std::runtime_error(
        "you have to set the debug_measurement_rate parameter on the "
        "parameter "
        "server to "
        "start the state estimation in debug mode");
  }
  return debug_update_rate;
}


int StateEstimationDebug::getUpdateRate() const {
  return getUpdateRateParameter();
}
