#ifndef PARAMS_FREE_DRIVE_STATE_MACHINES
#define PARAMS_FREE_DRIVE_STATE_MACHINES

struct ParametersFreeDriveStateMachine {
  double reverse_out_of_parking_spot_distance;
  double wheelbase;
  double distance_to_rear_bumper;
  double rear_safety_margin_parking;
  double velocity_search_parking_spots;
  double timeout_search_parking_spot;
  double waiting_time_in_parking_spot;
  int number_of_park_attempts;
  double parking_spot_update_distance;
};

#endif
