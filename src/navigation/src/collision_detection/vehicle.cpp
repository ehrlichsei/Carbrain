#include "vehicle.h"

Vehicle::Vehicle(double width, double distance_to_rear_bumper, double distance_to_front_bumper)
    : rear_left_(-distance_to_rear_bumper, 0.5 * width, 0),
      rear_right_(-distance_to_rear_bumper, -0.5 * width, 0),
      front_left_(distance_to_front_bumper, 0.5 * width, 0),
      front_right_(distance_to_front_bumper, -0.5 * width, 0) {}

LineSegment Vehicle::getLeftSide() const {
  return LineSegment(rear_left_.topRows<2>(), front_left_.topRows<2>());
}

LineSegment Vehicle::getRightSide() const {
  return LineSegment(rear_right_.topRows<2>(), front_right_.topRows<2>());
}

LineSegment Vehicle::getRearBumper() const {
  return LineSegment(rear_left_.topRows<2>(), rear_right_.topRows<2>());
}

LineSegment Vehicle::getFrontBumper() const {
  return LineSegment(front_left_.topRows<2>(), front_right_.topRows<2>());
}

Vehicle Vehicle::transformed(const Eigen::Affine3d &transformation) const {
  Vehicle copy = *this;
  copy.transform(transformation);
  return copy;
}

Vehicle& Vehicle::transform(const Eigen::Affine3d &transformation) {
  rear_left_ = transformation * rear_left_;
  rear_right_ = transformation * rear_right_;
  front_left_ = transformation * front_left_;
  front_right_ = transformation * front_right_;
  return *this;
}
