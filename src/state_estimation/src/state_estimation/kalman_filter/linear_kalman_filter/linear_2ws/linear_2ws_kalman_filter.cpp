#include "linear_2ws_kalman_filter.h"

THIRD_PARTY_HEADERS_BEGIN

THIRD_PARTY_HEADERS_END

const ParameterString<double> Linear2WSKalmanFilter::Parameters::SYSTEM_NOISE_SPEED(
    "linear2ws/system_noise_speed");
const ParameterString<double> Linear2WSKalmanFilter::Parameters::SYSTEM_NOISE_YAW_RATE(
    "linear2ws/system_noise_yaw_rate");
const ParameterString<double> Linear2WSKalmanFilter::Parameters::SYSTEM_NOISE_ACCELERATION(
    "linear2ws/system_noise_acceleration");
const ParameterString<double> Linear2WSKalmanFilter::Parameters::MEASUREMENT_NOISE_ANGULAR_SPEED_LEFT(
    "linear2ws/measurement_noise_angular_speed_left");
const ParameterString<double> Linear2WSKalmanFilter::Parameters::MEASUREMENT_NOISE_ANGULAR_SPEED_RIGHT(
    "linear2ws/measurement_noise_angular_speed_right");
const ParameterString<double> Linear2WSKalmanFilter::Parameters::MEASUREMENT_NOISE_ACCELERATION(
    "linear2ws/measurement_noise_acceleration");
const ParameterString<double> Linear2WSKalmanFilter::Parameters::MEASUREMENT_NOISE_YAW_RATE(
    "linear2ws/measurement_noise_yaw_rate");
const ParameterString<double> Linear2WSKalmanFilter::Parameters::MIN_ABS_SPEED_FOR_STEERING_ANGLE_CALCULATION(
    "linear2ws/min_absolute_speed_for_steering_angle_calculation");


Linear2WSKalmanFilter::Parameters::Parameters(const ParameterInterface &parameter_interface)
    : system_noise_speed(parameter_interface.getParam(Parameters::SYSTEM_NOISE_SPEED)),
      system_noise_yaw_rate(parameter_interface.getParam(Parameters::SYSTEM_NOISE_YAW_RATE)),
      system_noise_acceleration(
          parameter_interface.getParam(Parameters::SYSTEM_NOISE_ACCELERATION)),
      measurement_noise_angular_speed_left(parameter_interface.getParam(
          Parameters::MEASUREMENT_NOISE_ANGULAR_SPEED_LEFT)),
      measurement_noise_angular_speed_right(parameter_interface.getParam(
          Parameters::MEASUREMENT_NOISE_ANGULAR_SPEED_RIGHT)),
      measurement_noise_acceleration(
          parameter_interface.getParam(Parameters::MEASUREMENT_NOISE_ACCELERATION)),
      measurement_noise_yaw_rate(
          parameter_interface.getParam(Parameters::MEASUREMENT_NOISE_YAW_RATE)),
      min_absolute_speed_for_steering_angle_calculation(parameter_interface.getParam(
          Parameters::MIN_ABS_SPEED_FOR_STEERING_ANGLE_CALCULATION)) {}

Linear2WSKalmanFilter::Linear2WSKalmanFilter(double update_rate, ParameterInterface &params)
    : LinearSquareRootKalmanFilter(update_rate, params),
      params(Parameters::makeAndRegister(params)) {
  ROS_INFO("started linear two wheel steering (2WS) square root kalman filter");
}


Linear2WSKalmanFilter::Parameters Linear2WSKalmanFilter::Parameters::makeAndRegister(
    ParameterInterface &parameter_interface) {
  parameter_interface.registerParam(Parameters::SYSTEM_NOISE_SPEED);
  parameter_interface.registerParam(Parameters::SYSTEM_NOISE_YAW_RATE);
  parameter_interface.registerParam(Parameters::SYSTEM_NOISE_ACCELERATION);
  parameter_interface.registerParam(Parameters::MEASUREMENT_NOISE_ANGULAR_SPEED_LEFT);
  parameter_interface.registerParam(Parameters::MEASUREMENT_NOISE_ANGULAR_SPEED_RIGHT);
  parameter_interface.registerParam(Parameters::MEASUREMENT_NOISE_ACCELERATION);
  parameter_interface.registerParam(Parameters::MEASUREMENT_NOISE_YAW_RATE);
  parameter_interface.registerParam(Parameters::MIN_ABS_SPEED_FOR_STEERING_ANGLE_CALCULATION);

  return Parameters(parameter_interface);
}

void Linear2WSKalmanFilter::updateKalmanFilterParams() {
  params = Parameters(parameter_interface);
}

// system model
void Linear2WSKalmanFilter::updateSystemModel(double /*front_servo_command*/,
                                              double /*back_servo_command*/) {
  // state(0) = state(0) + state(3) * time_since_last_update;
  // state(1) = state(1);
  // state(2) = state(2);

  kalman_filter.system_matrix = KF::SystemMatrix::Zero();

  kalman_filter.system_matrix(0, 0) = 1.;
  kalman_filter.system_matrix(0, 1) = 0.;
  kalman_filter.system_matrix(0, 2) = time_between_updates;
  kalman_filter.system_matrix(1, 0) = 0.;
  kalman_filter.system_matrix(1, 1) = 1.;
  kalman_filter.system_matrix(1, 2) = 0.;
  kalman_filter.system_matrix(2, 0) = 0.;
  kalman_filter.system_matrix(2, 1) = 0.;
  kalman_filter.system_matrix(2, 2) = 1.;


  kalman_filter.control_vector = KF::ControlVector::Zero();



  kalman_filter.system_noise_matrix = KF::SystemNoiseMatrix::Zero();

  kalman_filter.system_noise_matrix(0, 0) = params.system_noise_speed;
  kalman_filter.system_noise_matrix(0, 1) = 0.;
  kalman_filter.system_noise_matrix(0, 2) = 0.;
  kalman_filter.system_noise_matrix(1, 0) = 0.;
  kalman_filter.system_noise_matrix(1, 1) = params.system_noise_yaw_rate;
  kalman_filter.system_noise_matrix(1, 2) = 0.;
  kalman_filter.system_noise_matrix(2, 0) = 0.;
  kalman_filter.system_noise_matrix(2, 1) = 0.;
  kalman_filter.system_noise_matrix(2, 2) = params.system_noise_acceleration;
}

// measurement model
void Linear2WSKalmanFilter::updateMeasurementModel(double /*front_servo_command*/,
                                                   double /*back_servo_command*/) {
  kalman_filter.measurement_matrix = KF::MeasurementMatrix::Zero();

  kalman_filter.measurement_matrix(0, 0) = 1. / car_specs.tire_radius;
  kalman_filter.measurement_matrix(0, 1) =
      -(car_specs.track / car_specs.tire_radius) * 0.5;
  kalman_filter.measurement_matrix(0, 2) = 0.;
  kalman_filter.measurement_matrix(1, 0) = 1. / car_specs.tire_radius;
  kalman_filter.measurement_matrix(1, 1) = (car_specs.track / car_specs.tire_radius) * 0.5;
  kalman_filter.measurement_matrix(1, 2) = 0.;
  kalman_filter.measurement_matrix(2, 0) = 0.;
  kalman_filter.measurement_matrix(2, 1) = 1.;
  kalman_filter.measurement_matrix(2, 2) = 0.;
  kalman_filter.measurement_matrix(3, 0) = 0.;
  kalman_filter.measurement_matrix(3, 1) = 0.;
  kalman_filter.measurement_matrix(3, 2) = 1.;


  kalman_filter.measurement_noise_matrix = KF::MeasurementNoiseMatrix::Zero();

  kalman_filter.measurement_noise_matrix(0, 0) = params.measurement_noise_angular_speed_left;
  kalman_filter.measurement_noise_matrix(1, 1) = params.measurement_noise_angular_speed_right;
  kalman_filter.measurement_noise_matrix(2, 2) = params.measurement_noise_yaw_rate;
  kalman_filter.measurement_noise_matrix(3, 3) = params.measurement_noise_acceleration;
}

VehicleState Linear2WSKalmanFilter::toVehicleState(const KF::StateVector &state) {
  VehicleState vehicle_state;
  vehicle_state.speed_x = state(0);
  vehicle_state.yaw_rate = state(1);
  vehicle_state.acceleration = state(2);
  vehicle_state.steering_angle_front = calculateFrontSteeringAngle();
  vehicle_state.steering_angle_back = 0.0;
  return vehicle_state;
}

Linear2WSKalmanFilter::KF::MeasurementVector Linear2WSKalmanFilter::toMeasurementVector(
    const SensorMeasurements &sensor_measurements) {
  KF::MeasurementVector measurement_vector = KF::MeasurementVector::Zero();

  measurement_vector(0) = sensor_measurements.angular_wheel_speeds.back.left;
  measurement_vector(1) = sensor_measurements.angular_wheel_speeds.back.right;
  measurement_vector(2) = sensor_measurements.left_IMU.angular_velocity.z;
  measurement_vector(3) = sensor_measurements.left_IMU.acceleration.x;

  return measurement_vector;
}

double Linear2WSKalmanFilter::calculateFrontSteeringAngle() {
  auto state = kalman_filter.getState();
  if (std::fabs(state(0)) > params.min_absolute_speed_for_steering_angle_calculation) {
    return std::atan(state(1) * car_specs.wheelbase / state(0));
  } else {
    return std::numeric_limits<double>::quiet_NaN();
  }
}
