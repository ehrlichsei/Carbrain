#include "fitness_calculator.h"
#include <common/math.h>

THIRD_PARTY_HEADERS_BEGIN
#include <cmath>
THIRD_PARTY_HEADERS_END

using common::squared;

FitnessCalculator::FitnessCalculator() { reset(); }

void FitnessCalculator::addMotionMeasurement(const ros::Time& stamp,
                                             double /*acceleration*/,
                                             double speed_x,
                                             double speed_y,
                                             double /*yaw_rate*/) {

  const double speed = std::sqrt(squared(speed_x) + squared(speed_y));
  max_velocity_ = std::max(max_velocity_, speed);
  min_velocity_ = std::min(min_velocity_, speed);

  const ros::Duration update_duration = stamp - last_stamp_;
  avg_velocity_ =
      (measurement_duration_.toSec() * avg_velocity_ + update_duration.toSec() * speed) /
      (measurement_duration_ + update_duration).toSec();

  driven_distance_ += update_duration.toSec() * speed;

  last_stamp_ = stamp;
  measurement_counter_++;
  measurement_duration_ += update_duration;
}

void FitnessCalculator::addLateralControlError(double error) {
  // squared error weights larger errors more heavily
  squared_lateral_control_error_ += squared(error);
  // absolute error is more robust against outliers (e.g. jumping path)
  abs_lateral_control_error_ += std::fabs(error);
}

void FitnessCalculator::reset() {
  min_velocity_ = std::numeric_limits<double>::max();
  max_velocity_ = std::numeric_limits<double>::min();
  avg_velocity_ = 0.0;
  driven_distance_ = 0.0;
  last_stamp_ = ros::Time::now();
  measurement_duration_ = ros::Duration(0);
  measurement_counter_ = 0;
  squared_lateral_control_error_ = 0;
  abs_lateral_control_error_ = 0;
}

double FitnessCalculator::getAverageVelocity() { return avg_velocity_; }

double FitnessCalculator::getMinVelocity() { return min_velocity_; }

double FitnessCalculator::getMaxVelocity() { return max_velocity_; }

ros::Duration FitnessCalculator::getRoundDuration() {
  return measurement_duration_;
}

double FitnessCalculator::getDrivenDistance() { return driven_distance_; }

double FitnessCalculator::getSquaredLateralControlError() {
  return squared_lateral_control_error_;
}

double FitnessCalculator::getAbsLateralControlError() {
  return abs_lateral_control_error_;
}
