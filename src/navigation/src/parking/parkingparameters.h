#ifndef PARKINGPARAMETERS_H
#define PARKINGPARAMETERS_H
#include <common/macros.h>

THIRD_PARTY_HEADERS_BEGIN
#include <ros/node_handle.h>
THIRD_PARTY_HEADERS_END

class ParkingParameters {
 public:

  void readParameters(const ros::NodeHandle &node) {
    node.param<double>("max_speed", max_speed, 0.4);
    node.param<int>("wait_between_turns", wait_between_turns, 600);
    node.param<double>("max_steering_angle", max_steering_angle, 2 * M_PI / 4);
    node.param<double>("max_turn_radius_r", max_turn_radius_r, 0.77);
    node.param<double>("max_turn_radius_l", max_turn_radius_l, 0.77);
    node.param<double>("end_angle_offset", end_angle_offset, 0);
  }

  double max_steering_angle = 0.4;
  int wait_between_turns = 600;
  double max_speed = M_PI / 2.0;
  double max_turn_radius_l = 0.77;
  double max_turn_radius_r = 0.77;
  double end_angle_offset = 0.0;

};

#endif // PARKINGPARAMETERS_H
