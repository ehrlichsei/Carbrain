#ifndef VEHICLE_H
#define VEHICLE_H

#include "navigation/line_segment.h"
#include "Eigen/Geometry"

class Vehicle {
 public:
  Vehicle() = default;
  Vehicle(double width, double distance_to_rear_bumper, double distance_to_front_bumper);


  LineSegment getLeftSide() const;

  LineSegment getRightSide() const;

  LineSegment getRearBumper() const;

  LineSegment getFrontBumper() const;

  Vehicle transformed(const Eigen::Affine3d& transformation) const;

  Vehicle& transform(const Eigen::Affine3d& transformation);

 private:

  Eigen::Vector3d rear_left_;
  Eigen::Vector3d rear_right_;
  Eigen::Vector3d front_left_;
  Eigen::Vector3d front_right_;

};

#endif  // VEHICLE_H
