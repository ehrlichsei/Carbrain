#include "car_specs.h"

bool CarSpecs::parameters_registered = false;
const ParameterString<double> CarSpecs::TRACK("/car_specs/track");
const ParameterString<double> CarSpecs::TIRE_RADIUS("/car_specs/tire_radius");
const ParameterString<double> CarSpecs::WHEELBASE("/car_specs/wheelbase");
const ParameterString<double> CarSpecs::DISTANCE_COG_FRONT(
    "/car_specs/distance_cog_front");
const ParameterString<double> CarSpecs::DISTANCE_COG_BACK(
    "/car_specs/distance_cog_back");
const ParameterString<double> CarSpecs::MAX_STEERING_ANGLE_LEFT(
    "/car_specs/max_steering_angle_left");
const ParameterString<double> CarSpecs::MAX_STEERING_ANGLE_RIGHT(
    "/car_specs/max_steering_angle_right");

CarSpecs::CarSpecs(ParameterInterface &parameter_interface) {
  if (!parameters_registered) {
    registerParams(parameter_interface);
    parameters_registered = true;
  }
  updateParams(parameter_interface);
}

void CarSpecs::updateParams(const ParameterInterface &parameter_interface) {
  track = parameter_interface.getParam(TRACK);
  tire_radius = parameter_interface.getParam(TIRE_RADIUS);
  wheelbase = parameter_interface.getParam(WHEELBASE);
  max_steering_angle_left = parameter_interface.getParam(MAX_STEERING_ANGLE_LEFT);
  max_steering_angle_right = parameter_interface.getParam(MAX_STEERING_ANGLE_RIGHT);
}

void CarSpecs::registerParams(ParameterInterface &parameter_interface) const {
  parameter_interface.registerParam(TRACK);
  parameter_interface.registerParam(TIRE_RADIUS);
  parameter_interface.registerParam(WHEELBASE);
  parameter_interface.registerParam(MAX_STEERING_ANGLE_LEFT);
  parameter_interface.registerParam(MAX_STEERING_ANGLE_RIGHT);
}
