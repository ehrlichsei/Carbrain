#ifndef DATA_PARKING_H
#define DATA_PARKING_H

#include "../fsm_utils/carcontroller.h"
#include "../fsm_utils/diagnostics_interface.h"
#include "common/parameter_interface.h"
#include "../fsm_utils/unidentified_object_params.h"
#include "../fsm_utils/state_timeouts.h"
#include "parameters_free_drive_state_machine.h"
#include "../fsm_utils/timeout.h"
#include "../fsm_utils/events.h"
class DataParking {
 public:
  DataParking(ParameterInterface* parameter_interface,
              ICarController& car_controller,
              IDiagnosticsInterface& diagnostics_iface,
              const std::shared_ptr<int>& number_of_park_attempts);


  ICarController& getCarController();

  void increaseNumberOfParkAttemptsCounter();
  bool reachedNumberOfParkAttempts() const;
  ParametersFreeDriveStateMachine getParams() const;

  perception_msgs::PerpendicularParkingSpot parking_spot_;
  Timeout timeout_;
  unsigned long max_speed_limitation_id_ = 0;
  unsigned long stopping_id_ = 0;
  nav_msgs::Path reverse_path_;
  Eigen::Affine3d pose_reverse_path_;
  bool set_path_ = true;
  ros::Time time_changed_pose_;
  double distance_behind_start_line_ = 0;

  void initValuesNewParkingAttempt(const EventStartLineDetected& ev);

 private:
  const ParameterInterface* const parameter_interface_;
  ICarController& car_controller;
  const ParameterString<double> PARAM_REVERSE_OUT_OF_PARKING_SPOT_DISTANCE;
  const ParameterString<double> PARAM_WHEELBASE;
  const ParameterString<double> PARAM_DISTANCE_TO_REAR_BUMPER;
  const ParameterString<double> PARAM_REAR_SAFETY_MARGIN_PARKING;
  const ParameterString<double> PARAM_VELOCITY_SEARCH_PARKING_SPOTS;
  const ParameterString<double> PARAM_TIMEOUT_SEARCH_PARKING_SPOT;
  const ParameterString<double> PARAM_WAITING_TIME_IN_PARKING_SPOT;
  const ParameterString<int> PARAM_NUMBER_OF_PARK_ATTEMPTS;
  const ParameterString<double> PARAM_PARKING_SPOT_UPDATE_DISTANCE;
  std::shared_ptr<int> number_of_park_attempts_;
};

#endif  // DATA_PARKING_H
