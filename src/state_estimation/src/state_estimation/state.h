#ifndef STATE_H
#define STATE_H
#include <common/macros.h>

THIRD_PARTY_HEADERS_BEGIN
#include <ros/time.h>
THIRD_PARTY_HEADERS_END

/*!
 * \brief the state of the vehicle
 */
struct VehicleState {
  double speed_x = 0.0;
  double speed_y = 0.0;
  double yaw_rate = 0.0;
  double acceleration = 0.0;
  double steering_angle_back = 0.0;
  double steering_angle_front = 0.0;
  ros::Time stamp;
};

#endif  // STATE_H
