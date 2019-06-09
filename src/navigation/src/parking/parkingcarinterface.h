#ifndef PARKINGCARINTERFACE_H
#define PARKINGCARINTERFACE_H
#include <common/macros.h>

#include "parkingparameters.h"

THIRD_PARTY_HEADERS_BEGIN
#include <ros/publisher.h>
#include "lateral_controller_msgs/DrivingSteeringAngle.h"
#include "longitudinal_controller_msgs/DrivingSpeed.h"
THIRD_PARTY_HEADERS_END

typedef boost::function<bool()> ConditionPredicate;

class ParkingCarInterface {

 public:
  ParkingCarInterface(ParkingParameters,
                      ros::Publisher &publisher_speed,
                      ros::Publisher &publisher_steering_angle);

  void setBackwardsSpeed(double speed);

  void setForwardSpeed(double speed);

  void setSteeringAngle(double angle);

  void steerMaxRight();

  void steerMaxLeft();

  void driveBackwardsUntil(const ConditionPredicate& p);

  void driveForwardUntil(const ConditionPredicate& p);

 private:
  longitudinal_controller_msgs::DrivingSpeed speedMessage;
  lateral_controller_msgs::DrivingSteeringAngle steeringAngleMessage;

  ParkingParameters parameters;
  ros::Publisher &publisher_speed;
  ros::Publisher &publisher_steering_angle;

  void rate_observing_rosloop(const ConditionPredicate& p);
};

#endif  // PARKINGCARINTERFACE_H
