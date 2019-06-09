#ifndef FIND_PARKING_LOT_SPEED_CONTROLLER
#define FIND_PARKING_LOT_SPEED_CONTROLLER
#include <common/macros.h>

#include "paramreader.h"

THIRD_PARTY_HEADERS_BEGIN
#include <ros/ros.h>
THIRD_PARTY_HEADERS_END

#define PARAM_HLC_SPEED "/control/longitudinal_controller/v_max_cc"

/*!
* Implements a simple state machine controlling the speed of the car,
* depending on slot size.
*/
class SpeedController {
 public:
  SpeedController() {}

  void setSpeedForSlotSize(double free_space_behind, double /*free_space_ahead*/, bool stop_at_this_slot) {
    const double min_slot_size = params.getMinSlotSize();
    const double speed_behind =
        params.getVMax() *
        (0.7f * fmax(0, min_slot_size - free_space_behind) / min_slot_size + 0.3f);

    double speed_setpoint = speed_behind;
    if (free_space_behind > params.getMaxSlotSize()) {
      speed_setpoint = params.getVMax();
    } else if (stop_at_this_slot) {
      speed_setpoint = 0.1;  // std::max(free_space_ahead, 0.1);
    }

    adjustSpeed(speed_setpoint);
  }

 private:
  ParamReader params;

  void adjustSpeed(double desired_speed) {
    ros::param::set(PARAM_HLC_SPEED, desired_speed);
  }
};

#endif  // FIND_PARKING_LOT_SPEED_CONTROLLER
