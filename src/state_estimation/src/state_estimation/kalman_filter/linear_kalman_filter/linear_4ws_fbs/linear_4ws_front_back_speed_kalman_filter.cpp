#include "linear_4ws_front_back_speed_kalman_filter.h"

THIRD_PARTY_HEADERS_BEGIN

THIRD_PARTY_HEADERS_END


const ParameterString<double> Linear4WSFrontBackSpeedKalmanFilter::Parameters::SYSTEM_NOISE_SPEED_FRONT(
    "linear4ws_fbs/system_noise_speed_front");
const ParameterString<double> Linear4WSFrontBackSpeedKalmanFilter::Parameters::SYSTEM_NOISE_SPEED_BACK(
    "linear4ws_fbs/system_noise_speed_back");
const ParameterString<double> Linear4WSFrontBackSpeedKalmanFilter::Parameters::SYSTEM_NOISE_YAW_RATE(
    "linear4ws_fbs/system_noise_yaw_rate");
const ParameterString<double> Linear4WSFrontBackSpeedKalmanFilter::Parameters::MEASUREMENT_NOISE_ANGULAR_SPEED_FRONT_LEFT(
    "linear4ws_fbs/measurement_noise_angular_speed_front_left");
const ParameterString<double> Linear4WSFrontBackSpeedKalmanFilter::Parameters::MEASUREMENT_NOISE_ANGULAR_SPEED_FRONT_RIGHT(
    "linear4ws_fbs/measurement_noise_angular_speed_front_right");
const ParameterString<double> Linear4WSFrontBackSpeedKalmanFilter::Parameters::MEASUREMENT_NOISE_ANGULAR_SPEED_BACK_LEFT(
    "linear4ws_fbs/measurement_noise_angular_speed_back_left");
const ParameterString<double> Linear4WSFrontBackSpeedKalmanFilter::Parameters::MEASUREMENT_NOISE_ANGULAR_SPEED_BACK_RIGHT(
    "linear4ws_fbs/measurement_noise_angular_speed_back_right");
const ParameterString<double> Linear4WSFrontBackSpeedKalmanFilter::Parameters::MEASUREMENT_NOISE_YAW_RATE(
    "linear4ws_fbs/measurement_noise_yaw_rate");

Linear4WSFrontBackSpeedKalmanFilter::Parameters::Parameters(const ParameterInterface &parameter_interface)
    : system_noise_speed_front(parameter_interface.getParam(Parameters::SYSTEM_NOISE_SPEED_FRONT)),
      system_noise_speed_back(parameter_interface.getParam(Parameters::SYSTEM_NOISE_SPEED_BACK)),
      system_noise_yaw_rate(parameter_interface.getParam(Parameters::SYSTEM_NOISE_YAW_RATE)),

      measurement_noise_angular_speed_front_left(parameter_interface.getParam(
          Parameters::MEASUREMENT_NOISE_ANGULAR_SPEED_FRONT_LEFT)),
      measurement_noise_angular_speed_back_left(parameter_interface.getParam(
          Parameters::MEASUREMENT_NOISE_ANGULAR_SPEED_BACK_LEFT)),

      measurement_noise_angular_speed_front_right(parameter_interface.getParam(
          Parameters::MEASUREMENT_NOISE_ANGULAR_SPEED_FRONT_RIGHT)),
      measurement_noise_angular_speed_back_right(parameter_interface.getParam(
          Parameters::MEASUREMENT_NOISE_ANGULAR_SPEED_BACK_RIGHT)),

      measurement_noise_yaw_rate(
          parameter_interface.getParam(Parameters::MEASUREMENT_NOISE_YAW_RATE)) {}

Linear4WSFrontBackSpeedKalmanFilter::Parameters Linear4WSFrontBackSpeedKalmanFilter::Parameters::makeAndRegister(
    ParameterInterface &parameter_interface) {
  parameter_interface.registerParam(Parameters::SYSTEM_NOISE_SPEED_FRONT);
  parameter_interface.registerParam(Parameters::SYSTEM_NOISE_SPEED_BACK);
  parameter_interface.registerParam(Parameters::SYSTEM_NOISE_YAW_RATE);

  parameter_interface.registerParam(Parameters::MEASUREMENT_NOISE_ANGULAR_SPEED_FRONT_LEFT);
  parameter_interface.registerParam(Parameters::MEASUREMENT_NOISE_ANGULAR_SPEED_FRONT_RIGHT);
  parameter_interface.registerParam(Parameters::MEASUREMENT_NOISE_ANGULAR_SPEED_BACK_LEFT);
  parameter_interface.registerParam(Parameters::MEASUREMENT_NOISE_ANGULAR_SPEED_BACK_RIGHT);
  parameter_interface.registerParam(Parameters::MEASUREMENT_NOISE_YAW_RATE);

  return Parameters(parameter_interface);
}

Linear4WSFrontBackSpeedKalmanFilter::Linear4WSFrontBackSpeedKalmanFilter(double update_rate,
                                                                         ParameterInterface &params)
    : LinearSquareRootKalmanFilter(update_rate, params),
      front_servo_characteristic(
          params,
          ParameterString<double>(
              "front_servo_characteristic/servo_characteristic_a"),
          ParameterString<double>(
              "front_servo_characteristic/servo_characteristic_b"),
          ParameterString<double>(
              "front_servo_characteristic/servo_characteristic_c"),
          ParameterString<double>(
              "front_servo_characteristic/servo_characteristic_d"),
          ParameterString<double>("front_servo_characteristic/time_constant"),
          ParameterString<double>("front_servo_characteristic/dead_time"),
          1. / time_between_updates),
      back_servo_characteristic(
          params,
          ParameterString<double>(
              "back_servo_characteristic/servo_characteristic_a"),
          ParameterString<double>(
              "back_servo_characteristic/servo_characteristic_b"),
          ParameterString<double>(
              "back_servo_characteristic/servo_characteristic_c"),
          ParameterString<double>(
              "back_servo_characteristic/servo_characteristic_d"),
          ParameterString<double>("back_servo_characteristic/time_constant"),
          ParameterString<double>("back_servo_characteristic/dead_time"),
          1. / time_between_updates),
      params(Parameters::makeAndRegister(params)) {
  ROS_INFO(
      "started linear all wheel steering (4WS) square root kalman filter with "
      "seperate front and back speed state");
}

void Linear4WSFrontBackSpeedKalmanFilter::updateKalmanFilterParams() {
  params = Parameters(parameter_interface);

  front_servo_characteristic.updateParams(1. / time_between_updates);
  back_servo_characteristic.updateParams(1. / time_between_updates);
}

void Linear4WSFrontBackSpeedKalmanFilter::updateSystemModel(double /*front_servo_command*/,
                                                            double /*back_servo_command*/) {
  kalman_filter.system_matrix = KF::SystemMatrix::Identity();

  kalman_filter.control_vector = KF::ControlVector::Zero();

  kalman_filter.system_noise_matrix = KF::SystemNoiseMatrix::Zero();

  kalman_filter.system_noise_matrix(0, 0) = params.system_noise_speed_front;
  kalman_filter.system_noise_matrix(0, 1) = 0.;
  kalman_filter.system_noise_matrix(0, 2) = 0.;
  kalman_filter.system_noise_matrix(1, 0) = 0.;
  kalman_filter.system_noise_matrix(1, 1) = params.system_noise_speed_back;
  kalman_filter.system_noise_matrix(1, 2) = 0.;
  kalman_filter.system_noise_matrix(2, 0) = 0.;
  kalman_filter.system_noise_matrix(2, 1) = 0.;
  kalman_filter.system_noise_matrix(2, 2) = params.system_noise_yaw_rate;
}

void Linear4WSFrontBackSpeedKalmanFilter::updateMeasurementModel(double front_servo_command,
                                                                 double back_servo_command) {
  kalman_filter.measurement_noise_matrix = KF::MeasurementNoiseMatrix::Zero();
  kalman_filter.measurement_noise_matrix(0, 0) = params.measurement_noise_angular_speed_front_left;
  kalman_filter.measurement_noise_matrix(1, 1) =
      params.measurement_noise_angular_speed_front_right;
  kalman_filter.measurement_noise_matrix(2, 2) = params.measurement_noise_angular_speed_back_left;
  kalman_filter.measurement_noise_matrix(3, 3) = params.measurement_noise_angular_speed_back_right;
  kalman_filter.measurement_noise_matrix(4, 4) = params.measurement_noise_yaw_rate;
  kalman_filter.measurement_noise_matrix(5, 5) = params.measurement_noise_yaw_rate;

  steering_angle_front = front_servo_characteristic.setValueToAngle(front_servo_command);
  steering_angle_back = back_servo_characteristic.setValueToAngle(back_servo_command);


  using std::sin;
  using std::cos;

  kalman_filter.measurement_matrix(0, 0) = 1.0 / car_specs.tire_radius;
  kalman_filter.measurement_matrix(0, 1) = 0.0;
  kalman_filter.measurement_matrix(0, 2) =
      (-0.5 * car_specs.track * cos(steering_angle_front)) / car_specs.tire_radius;

  kalman_filter.measurement_matrix(1, 0) = 1.0 / car_specs.tire_radius;
  kalman_filter.measurement_matrix(1, 1) = 0.0;
  kalman_filter.measurement_matrix(1, 2) =
      (0.5 * car_specs.track * cos(steering_angle_front)) / car_specs.tire_radius;
  kalman_filter.measurement_matrix(2, 0) = 0.0;
  kalman_filter.measurement_matrix(2, 1) = 1.0 / car_specs.tire_radius;
  kalman_filter.measurement_matrix(2, 2) =
      (-0.5 * car_specs.track * cos(steering_angle_back)) / car_specs.tire_radius;

  kalman_filter.measurement_matrix(3, 0) = 0.0;
  kalman_filter.measurement_matrix(3, 1) = 1.0 / car_specs.tire_radius;
  kalman_filter.measurement_matrix(3, 2) =
      (0.5 * car_specs.track * cos(steering_angle_back)) / car_specs.tire_radius;

  kalman_filter.measurement_matrix(4, 0) = 0.;
  kalman_filter.measurement_matrix(4, 1) = 0.;
  kalman_filter.measurement_matrix(4, 2) = 1.;

  kalman_filter.measurement_matrix(5, 0) = 0.;
  kalman_filter.measurement_matrix(5, 1) = 0.;
  kalman_filter.measurement_matrix(5, 2) = 1.;
}

VehicleState Linear4WSFrontBackSpeedKalmanFilter::toVehicleState(const KF::StateVector &state) {
  const double v_x_cog_from_front = std::cos(steering_angle_front) * state(0);
  const double v_y_cog_from_front = std::sin(steering_angle_front) * state(0) -
                                    car_specs.distance_cog_front * state(2);

  const double v_x_cog_from_back = std::cos(steering_angle_back) * state(1);
  const double v_y_cog_from_back = std::sin(steering_angle_back) * state(1) +
                                   car_specs.distance_cog_rear * state(2);


  VehicleState vehicle_state;
  vehicle_state.speed_x = 0.5 * (v_x_cog_from_front + v_x_cog_from_back);
  vehicle_state.speed_y = 0.5 * (v_y_cog_from_front + v_y_cog_from_back);
  vehicle_state.yaw_rate = state(2);
  vehicle_state.steering_angle_back = steering_angle_back;
  vehicle_state.steering_angle_front = steering_angle_front;
  return vehicle_state;
}

Linear4WSFrontBackSpeedKalmanFilter::KF::MeasurementVector Linear4WSFrontBackSpeedKalmanFilter::toMeasurementVector(
    const SensorMeasurements &measures) {
  KF::MeasurementVector measurement_vector = KF::MeasurementVector::Zero();

  measurement_vector(0) = measures.angular_wheel_speeds.front.left;
  measurement_vector(1) = measures.angular_wheel_speeds.front.right;
  measurement_vector(2) = measures.angular_wheel_speeds.back.left;
  measurement_vector(3) = measures.angular_wheel_speeds.back.right;
  measurement_vector(4) = measures.left_IMU.angular_velocity.z;
  measurement_vector(5) = measures.right_IMU.angular_velocity.z;

  return measurement_vector;
}
