#include "data_parking.h"

DataParking::DataParking(ParameterInterface *parameter_interface,
                         ICarController &car_controller,
                         IDiagnosticsInterface & /*diagnostics_iface*/,
                         const std::shared_ptr<int> &number_of_park_attempts)
    : parameter_interface_(parameter_interface),
      car_controller(car_controller),
      PARAM_REVERSE_OUT_OF_PARKING_SPOT_DISTANCE(
          "reverse_out_of_parking_spot_distance"),
      PARAM_WHEELBASE("wheelbase"),
      PARAM_DISTANCE_TO_REAR_BUMPER("distance_to_rear_bumper"),
      PARAM_REAR_SAFETY_MARGIN_PARKING("rear_safety_margin_parking"),
      PARAM_VELOCITY_SEARCH_PARKING_SPOTS("velocity_search_parking_spots"),
      PARAM_TIMEOUT_SEARCH_PARKING_SPOT("timeout_search_parking_spot"),
      PARAM_WAITING_TIME_IN_PARKING_SPOT("waiting_time_in_parking_spot"),
      PARAM_NUMBER_OF_PARK_ATTEMPTS("number_of_park_attempts"),
      PARAM_PARKING_SPOT_UPDATE_DISTANCE("parking_spot_update_distance"),
      number_of_park_attempts_(number_of_park_attempts) {
  parameter_interface->registerParam(PARAM_REVERSE_OUT_OF_PARKING_SPOT_DISTANCE);
  parameter_interface->registerParam(PARAM_WHEELBASE);
  parameter_interface->registerParam(PARAM_DISTANCE_TO_REAR_BUMPER);
  parameter_interface->registerParam(PARAM_REAR_SAFETY_MARGIN_PARKING);
  parameter_interface->registerParam(PARAM_VELOCITY_SEARCH_PARKING_SPOTS);
  parameter_interface->registerParam(PARAM_TIMEOUT_SEARCH_PARKING_SPOT);
  parameter_interface->registerParam(PARAM_WAITING_TIME_IN_PARKING_SPOT);
  parameter_interface->registerParam(PARAM_PARKING_SPOT_UPDATE_DISTANCE);
  parameter_interface->registerParam(PARAM_NUMBER_OF_PARK_ATTEMPTS);
  ROS_INFO("FreeDriveStateMachine created.");
}


ICarController &DataParking::getCarController() { return car_controller; }

ParametersFreeDriveStateMachine DataParking::getParams() const {
  return {.reverse_out_of_parking_spot_distance =
              parameter_interface_->getParam(PARAM_REVERSE_OUT_OF_PARKING_SPOT_DISTANCE),
          .wheelbase = parameter_interface_->getParam(PARAM_WHEELBASE),
          .distance_to_rear_bumper = parameter_interface_->getParam(PARAM_DISTANCE_TO_REAR_BUMPER),
          .rear_safety_margin_parking =
              parameter_interface_->getParam(PARAM_REAR_SAFETY_MARGIN_PARKING),
          .velocity_search_parking_spots =
              parameter_interface_->getParam(PARAM_VELOCITY_SEARCH_PARKING_SPOTS),
          .timeout_search_parking_spot =
              parameter_interface_->getParam(PARAM_TIMEOUT_SEARCH_PARKING_SPOT),
          .waiting_time_in_parking_spot =
              parameter_interface_->getParam(PARAM_WAITING_TIME_IN_PARKING_SPOT),
          .number_of_park_attempts = parameter_interface_->getParam(PARAM_NUMBER_OF_PARK_ATTEMPTS),
          .parking_spot_update_distance =
              parameter_interface_->getParam(PARAM_PARKING_SPOT_UPDATE_DISTANCE)};
}

void DataParking::initValuesNewParkingAttempt(const EventStartLineDetected &ev) {
  distance_behind_start_line_ = -ev.dist_before_vehicle;
  stopping_id_ = 0;
  max_speed_limitation_id_ = 0;
  set_path_ = true;
}

void DataParking::increaseNumberOfParkAttemptsCounter() {
  (*number_of_park_attempts_)++;
}

bool DataParking::reachedNumberOfParkAttempts() const {
  return *number_of_park_attempts_ >= getParams().number_of_park_attempts;
}
