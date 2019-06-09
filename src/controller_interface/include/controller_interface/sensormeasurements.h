#ifndef SENSORMEASUREMENTS_H
#define SENSORMEASUREMENTS_H
#include <common/macros.h>

THIRD_PARTY_HEADERS_BEGIN
#include <controller_msgs/StateMeasure.h>
#include <ros/time.h>
THIRD_PARTY_HEADERS_END

struct SensorMeasurements {
  ros::Time stamp;
  struct Axle {
    double left = 0.0;
    double right = 0.0;
  };

  struct IMU {
    struct AngularVelocity {
      double x = 0.0;
      double y = 0.0;
      double z = 0.0;
    } angular_velocity;
    struct Acceleration {
      double x = 0.0;
      double y = 0.0;
      double z = 0.0;
    } acceleration;
  } left_IMU, right_IMU;

  struct AngularWheelSpeeds {
    Axle front, back;
  } angular_wheel_speeds;

  struct SteeringAngles {
    Axle front, back;
  } steering_angles;

  bool manual_mode = false;
};


inline std::string to_string(SensorMeasurements::IMU::AngularVelocity val) {
  return "angular velocity: (" + std::to_string(val.x) + ", " +
         std::to_string(val.y) + ", " + std::to_string(val.z) + ")";
}
inline std::string to_string(SensorMeasurements::IMU::Acceleration val) {
  return "acceleration: (" + std::to_string(val.x) + ", " +
         std::to_string(val.y) + ", " + std::to_string(val.z) + ")";
}
inline std::string to_string(SensorMeasurements::IMU val) {
  return "IMU:\n  " + to_string(val.angular_velocity) + "\n  " + to_string(val.acceleration);
}
inline std::string to_string(SensorMeasurements::AngularWheelSpeeds val) {
  return "wheel speeds:\n  front left: " + std::to_string(val.front.left) +
         "\n  front right: " + std::to_string(val.front.right) +
         "\n  back left: " + std::to_string(val.back.left) +
         "\n  back right: " + std::to_string(val.back.right);
}
inline std::string to_string(SensorMeasurements::SteeringAngles val) {
  return "steering angles:\n  front left: " + std::to_string(val.front.left) +
         "\n  front right: " + std::to_string(val.front.right) +
         "\n  back left: " + std::to_string(val.back.left) +
         "\n  back right: " + std::to_string(val.back.right);
}
inline std::string to_string(SensorMeasurements val) {
  return "\nleft IMU:\n  " + to_string(val.left_IMU) + "\nright IMU:\n  " +
         to_string(val.right_IMU) + "\n\n" + to_string(val.angular_wheel_speeds) +
         "\n\n" + to_string(val.steering_angles) + "\n";
}


#endif  // SENSORMEASUREMENTS_H
