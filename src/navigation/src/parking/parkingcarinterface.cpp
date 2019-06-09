#include "parkingcarinterface.h"
#include <common/macros.h>

THIRD_PARTY_HEADERS_BEGIN
#include <ros/ros.h>
THIRD_PARTY_HEADERS_END

ParkingCarInterface::ParkingCarInterface(ParkingParameters parameters,
                                         ros::Publisher &publisher_speed,
                                         ros::Publisher &publisher_steering_angle)
    : parameters(parameters),
      publisher_speed(publisher_speed),
      publisher_steering_angle(publisher_steering_angle) {}

/**
 * @brief Publishes a new speed command. This will make the car drive forwards
 * with the specified speed.
 * @param speed the speed to set in m/s
 */
void ParkingCarInterface::setForwardSpeed(const double speed) {
  speedMessage.header.stamp = ros::Time::now();
  speedMessage.speed = speed;
  publisher_speed.publish(speedMessage);
}

/**
 * @brief Publishes a new speed command. This will make the car drive forwards
 * with the specified speed.
 * @param speed the speed to set in m/s
 */
void ParkingCarInterface::setBackwardsSpeed(const double speed) {
  speedMessage.header.stamp = ros::Time::now();
  speedMessage.speed = -speed;
  publisher_speed.publish(speedMessage);
}

/**
 * Sets the steering angle of the car directly
 * @param angle the angle, in radians. 0 means the wheels look forward.
 */
void ParkingCarInterface::setSteeringAngle(const double angle) {
  steeringAngleMessage.header.stamp = ros::Time::now();
  steeringAngleMessage.steering_angle = angle;
  publisher_steering_angle.publish(steeringAngleMessage);
}

/**
 * @brief steer to the maximum extend to the right
 */
void ParkingCarInterface::steerMaxRight() {
  setSteeringAngle(-parameters.max_steering_angle);
}

/**
 * @brief steer to the maximum extend to the left
 */
void ParkingCarInterface::steerMaxLeft() {
  setSteeringAngle(parameters.max_steering_angle);
}

/**
 * @brief convenience method that starts driving and blocks until a condition is
 * fulfilled
 * @param p the condition that must be satisfied.
 */
void ParkingCarInterface::driveForwardUntil(const ConditionPredicate& p) {
  setForwardSpeed(parameters.max_speed);
  rate_observing_rosloop(p);
  setForwardSpeed(0);
}

/**
 * @brief convenience method that starts driving and blocks until a condition is
 * fulfilled
 * @param p the condition that must be satisfied.
 */
void ParkingCarInterface::driveBackwardsUntil(const ConditionPredicate& p) {
  setBackwardsSpeed(parameters.max_speed);
  rate_observing_rosloop(p);
  setBackwardsSpeed(0);
}

/**
 * @brief conditional ros::spin() that outputs warnings if the loop rate is
 * below 60hz.
 * @param p the condition that must be satisfied to return
 */
void ParkingCarInterface::rate_observing_rosloop(const ConditionPredicate& p) {
  ros::Time last_loop_time = ros::Time::now();
  while (!p()) {
    ros::spinOnce();
    ros::Duration delta_t = ros::Time::now() - last_loop_time;
    if (delta_t > ros::Duration(1 / 60.0)) {
      ROS_WARN_THROTTLE(1, "We are looping too slowly!");
    }
  }
}
