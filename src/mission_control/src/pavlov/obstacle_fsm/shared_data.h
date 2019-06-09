#ifndef SHARED_DATA_H
#define SHARED_DATA_H

#include "../fsm_utils/carcontroller.h"
#include "../fsm_utils/diagnostics_interface.h"
#include "common/parameter_interface.h"
#include "../fsm_utils/safety_margins.h"
#include "../fsm_utils/unidentified_object_params.h"

class SharedData {
 public:
  SharedData(ParameterInterface* parameter_interface,
             ICarController& car_controller,
             IDiagnosticsInterface& diagnostics_iface);


  ICarController& getCarController();

  IDiagnosticsInterface& getDiagnosticsInterface();

  double getBlockingEntityPreemptionDistanceThresholdInM();

  SafetyMargins getSafetyMargins() const;

  double getMaxStopAtUpdateDistance();


  double getApproachingUnsureObservationSpeedInMPerS();

  UnidentifiedObjectParams getUnidentifiedObjectParams() const;

  double getSpeedLimitResetDistance(const unsigned int speed_limit_in_km_per_h);

private:
  const ParameterInterface* const parameter_interface_;
  ICarController& car_controller;
  IDiagnosticsInterface& diagnostics_iface;

  const ParameterString<double> PARAM_BLOCKING_ENTITY_PREEMPTION_DISTANCE_THRESHOLD;
  const ParameterString<double> PARAM_SAFETY_MARGIN_CROSSWALK;
  const ParameterString<double> PARAM_SAFETY_MARGIN_ROAD_CLOSURE;
  const ParameterString<double> PARAM_SAFETY_MARGIN_JUNCTION;  
  const ParameterString<double> PARAM_MAX_STOP_AT_UPDATE_DISTANCE;  
  const ParameterString<double> PARAM_APPROACHING_UNSURE_OBSERVATION_SPEED_IN_M_PER_S;  
  const ParameterString<double> PARAM_UNIDENTIFIED_OBJECT_VELOCITY_FACTOR;
  const ParameterString<double> PARAM_UNIDENTIFIED_OBJECT_DISTANCE_LOWEST_VELOCITY;
  const ParameterString<double> PARAM_UNIDENTIFIED_OBJECT_LOWEST_VELOCITY;
  const ParameterString<double> PARAM_SPEED_LIMIT_RESET_DISTANCE_WITH_10_KMH;
  const ParameterString<double> PARAM_SPEED_LIMIT_RESET_DISTANCE_WITH_20_KMH;
  const ParameterString<double> PARAM_SPEED_LIMIT_RESET_DISTANCE_WITH_30_KMH;
  const ParameterString<double> PARAM_SPEED_LIMIT_RESET_DISTANCE_WITH_40_KMH;
  const ParameterString<double> PARAM_SPEED_LIMIT_RESET_DISTANCE_WITH_50_KMH;
  const ParameterString<double> PARAM_SPEED_LIMIT_RESET_DISTANCE_WITH_60_KMH;
  const ParameterString<double> PARAM_SPEED_LIMIT_RESET_DISTANCE_WITH_70_KMH;
  const ParameterString<double> PARAM_SPEED_LIMIT_RESET_DISTANCE_WITH_80_KMH;
  const ParameterString<double> PARAM_SPEED_LIMIT_RESET_DISTANCE_WITH_90_KMH;    
};

#endif  // SHARED_DATA_H
