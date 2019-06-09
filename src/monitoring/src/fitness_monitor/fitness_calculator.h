#ifndef FITNESS_CALCULATOR_H
#define FITNESS_CALCULATOR_H
#include <common/macros.h>

THIRD_PARTY_HEADERS_BEGIN
#include <limits>
#include <ros/ros.h>
THIRD_PARTY_HEADERS_END

class FitnessCalculator {
 public:
  FitnessCalculator();

  /**
   * adds new motion measurement to calculator
   * @brief add new motion measurement
   * @param stamp time stamp of measurement, motion measurements has to be in
   * chronological order
   * @param acceleration measured acceleration
   * @param speed_x measured speed in x-direction
   * @param speed_y measured speed in y-direction
   * @param yaw_rate measured yaw rate
   */
  void addMotionMeasurement(const ros::Time& stamp, double acceleration, double speed_x, double speed_y, double yaw_rate);

  void addLateralControlError(double error);

  /**
   * @brief resets all measurements
   */
  void reset();

  /**
   * @brief getAverageVelocity
   * @return average velocity
   */
  double getAverageVelocity();

  /**
   * @brief getMinVelocity
   * @return minimum velocity
   */
  double getMinVelocity();

  /**
   * @brief getMaxVelocity
   * @return maximum velocity
   */
  double getMaxVelocity();

  /**
   * @brief getRoundTime
   * @return duration of round
   */
  ros::Duration getRoundDuration();

  /**
   * @brief getDrivenDistance
   * @return accumulated distance during round
   */
  double getDrivenDistance();

  double getSquaredLateralControlError();

  double getAbsLateralControlError();

 private:
  double min_velocity_ = std::numeric_limits<double>::max();
  double max_velocity_ = std::numeric_limits<double>::min();
  double avg_velocity_ = 0.0;
  double driven_distance_ = 0.0;
  double squared_lateral_control_error_ = 0.0;
  double abs_lateral_control_error_ = 0.0;
  int measurement_counter_ = 0;
  ros::Duration measurement_duration_;
  ros::Time last_stamp_;
};

#endif  // FITNESS_CALCULATOR_H
