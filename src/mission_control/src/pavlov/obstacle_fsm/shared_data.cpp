#include "shared_data.h"

SharedData::SharedData(ParameterInterface *parameter_interface,
                       ICarController &car_controller,
                       IDiagnosticsInterface &diagnostics_iface)
    : parameter_interface_(parameter_interface),
      car_controller(car_controller),
      diagnostics_iface(diagnostics_iface),
      PARAM_BLOCKING_ENTITY_PREEMPTION_DISTANCE_THRESHOLD(
          "blocking_entity_preemption_distance_threshold_in_m"),
      PARAM_SAFETY_MARGIN_CROSSWALK("safety_margin_crosswalk"),
      PARAM_SAFETY_MARGIN_ROAD_CLOSURE("safety_margin_road_closure"),
      PARAM_SAFETY_MARGIN_JUNCTION("safety_margin_junction"),      
      PARAM_MAX_STOP_AT_UPDATE_DISTANCE("max_stop_at_update_distance"),      
      PARAM_APPROACHING_UNSURE_OBSERVATION_SPEED_IN_M_PER_S(
          "approaching_unsure_observation_speed_in_m_per_s"),
      PARAM_UNIDENTIFIED_OBJECT_VELOCITY_FACTOR(
          "unidentified_object_velocity_factor"),
      PARAM_UNIDENTIFIED_OBJECT_DISTANCE_LOWEST_VELOCITY(
          "unidentified_object_distance_lowest_velocity"),
      PARAM_UNIDENTIFIED_OBJECT_LOWEST_VELOCITY(
          "unidentified_object_lowest_velocity"),
      PARAM_SPEED_LIMIT_RESET_DISTANCE_WITH_10_KMH(
          "speed_limit_reset_distance/with_10_kmh"),
      PARAM_SPEED_LIMIT_RESET_DISTANCE_WITH_20_KMH(
          "speed_limit_reset_distance/with_20_kmh"),
      PARAM_SPEED_LIMIT_RESET_DISTANCE_WITH_30_KMH(
          "speed_limit_reset_distance/with_30_kmh"),
      PARAM_SPEED_LIMIT_RESET_DISTANCE_WITH_40_KMH(
          "speed_limit_reset_distance/with_40_kmh"),
      PARAM_SPEED_LIMIT_RESET_DISTANCE_WITH_50_KMH(
          "speed_limit_reset_distance/with_50_kmh"),
      PARAM_SPEED_LIMIT_RESET_DISTANCE_WITH_60_KMH(
          "speed_limit_reset_distance/with_60_kmh"),
      PARAM_SPEED_LIMIT_RESET_DISTANCE_WITH_70_KMH(
          "speed_limit_reset_distance/with_70_kmh"),
      PARAM_SPEED_LIMIT_RESET_DISTANCE_WITH_80_KMH(
          "speed_limit_reset_distance/with_80_kmh"),
      PARAM_SPEED_LIMIT_RESET_DISTANCE_WITH_90_KMH(
          "speed_limit_reset_distance/with_90_kmh") {
  parameter_interface->registerParam(PARAM_BLOCKING_ENTITY_PREEMPTION_DISTANCE_THRESHOLD);
  parameter_interface->registerParam(PARAM_SAFETY_MARGIN_CROSSWALK);
  parameter_interface->registerParam(PARAM_SAFETY_MARGIN_ROAD_CLOSURE);
  parameter_interface->registerParam(PARAM_SAFETY_MARGIN_JUNCTION);
  parameter_interface->registerParam(PARAM_MAX_STOP_AT_UPDATE_DISTANCE);
  parameter_interface->registerParam(PARAM_APPROACHING_UNSURE_OBSERVATION_SPEED_IN_M_PER_S);
  parameter_interface->registerParam(PARAM_UNIDENTIFIED_OBJECT_VELOCITY_FACTOR);
  parameter_interface->registerParam(PARAM_UNIDENTIFIED_OBJECT_DISTANCE_LOWEST_VELOCITY);
  parameter_interface->registerParam(PARAM_UNIDENTIFIED_OBJECT_LOWEST_VELOCITY);
  parameter_interface->registerParam(PARAM_SPEED_LIMIT_RESET_DISTANCE_WITH_10_KMH);
  parameter_interface->registerParam(PARAM_SPEED_LIMIT_RESET_DISTANCE_WITH_20_KMH);
  parameter_interface->registerParam(PARAM_SPEED_LIMIT_RESET_DISTANCE_WITH_30_KMH);
  parameter_interface->registerParam(PARAM_SPEED_LIMIT_RESET_DISTANCE_WITH_40_KMH);
  parameter_interface->registerParam(PARAM_SPEED_LIMIT_RESET_DISTANCE_WITH_50_KMH);
  parameter_interface->registerParam(PARAM_SPEED_LIMIT_RESET_DISTANCE_WITH_60_KMH);
  parameter_interface->registerParam(PARAM_SPEED_LIMIT_RESET_DISTANCE_WITH_70_KMH);
  parameter_interface->registerParam(PARAM_SPEED_LIMIT_RESET_DISTANCE_WITH_80_KMH);
  parameter_interface->registerParam(PARAM_SPEED_LIMIT_RESET_DISTANCE_WITH_90_KMH);
}

ICarController &SharedData::getCarController() { return car_controller; }

IDiagnosticsInterface &SharedData::getDiagnosticsInterface() {
  return diagnostics_iface;
}

double SharedData::getBlockingEntityPreemptionDistanceThresholdInM() {
  return parameter_interface_->getParam(PARAM_BLOCKING_ENTITY_PREEMPTION_DISTANCE_THRESHOLD);
}

SafetyMargins SharedData::getSafetyMargins() const {
  return {.crosswalk_margin = parameter_interface_->getParam(PARAM_SAFETY_MARGIN_CROSSWALK),
          .road_closure_margin = parameter_interface_->getParam(PARAM_SAFETY_MARGIN_ROAD_CLOSURE),
          .junction_margin = parameter_interface_->getParam(PARAM_SAFETY_MARGIN_JUNCTION)};
}

double SharedData::getMaxStopAtUpdateDistance() {
  return parameter_interface_->getParam(PARAM_MAX_STOP_AT_UPDATE_DISTANCE);
}


double SharedData::getApproachingUnsureObservationSpeedInMPerS() {
  return parameter_interface_->getParam(PARAM_APPROACHING_UNSURE_OBSERVATION_SPEED_IN_M_PER_S);
}

UnidentifiedObjectParams SharedData::getUnidentifiedObjectParams() const {
  return {.lowest_velocity = parameter_interface_->getParam(PARAM_UNIDENTIFIED_OBJECT_LOWEST_VELOCITY),
          .distance_lowest_velocity = parameter_interface_->getParam(
              PARAM_UNIDENTIFIED_OBJECT_DISTANCE_LOWEST_VELOCITY),
          .velocity_factor =
              parameter_interface_->getParam(PARAM_UNIDENTIFIED_OBJECT_VELOCITY_FACTOR)};
}

double SharedData::getSpeedLimitResetDistance(const unsigned int speed_limit_in_km_per_h) {
  unsigned int speed_floored = (speed_limit_in_km_per_h / 10) * 10;
  if (speed_floored <= 10) {
    return parameter_interface_->getParam(PARAM_SPEED_LIMIT_RESET_DISTANCE_WITH_10_KMH);
  } else if (speed_floored == 20) {
    return parameter_interface_->getParam(PARAM_SPEED_LIMIT_RESET_DISTANCE_WITH_20_KMH);
  } else if (speed_floored == 30) {
    return parameter_interface_->getParam(PARAM_SPEED_LIMIT_RESET_DISTANCE_WITH_30_KMH);
  } else if (speed_floored == 40) {
    return parameter_interface_->getParam(PARAM_SPEED_LIMIT_RESET_DISTANCE_WITH_40_KMH);
  } else if (speed_floored == 50) {
    return parameter_interface_->getParam(PARAM_SPEED_LIMIT_RESET_DISTANCE_WITH_50_KMH);
  } else if (speed_floored == 60) {
    return parameter_interface_->getParam(PARAM_SPEED_LIMIT_RESET_DISTANCE_WITH_60_KMH);
  } else if (speed_floored == 70) {
    return parameter_interface_->getParam(PARAM_SPEED_LIMIT_RESET_DISTANCE_WITH_70_KMH);
  } else if (speed_floored == 80) {
    return parameter_interface_->getParam(PARAM_SPEED_LIMIT_RESET_DISTANCE_WITH_80_KMH);
  } else {
    return parameter_interface_->getParam(PARAM_SPEED_LIMIT_RESET_DISTANCE_WITH_90_KMH);
  }
}
