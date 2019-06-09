#ifndef KALMANFILTER_H
#define KALMANFILTER_H

#include <controller_interface/sensormeasurements.h>
#include "../car_specs.h"
#include "common/kalman_filter.h"
#include "common/parameter_interface.h"
#include "../servo_characteristic.h"
#include "../state.h"

THIRD_PARTY_HEADERS_BEGIN
#include <eigen_conversions/eigen_msg.h>
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>
#include <eigen3/Eigen/Dense>
#include <memory>
THIRD_PARTY_HEADERS_END


/*!
 * \brief Estimates state based on incoming sensor data. Abstract base class for
 * concrete kalman filter implementations.
 */
class KalmanFilter {
 public:
  KalmanFilter(double update_rate, ParameterInterface& params);

  virtual ~KalmanFilter() = default;

  /*!
  * \brief kalman filter prediction step
  * \return prediction
  */
  virtual VehicleState predict(double front_servo_command,
                               double back_servo_command,
                               double engine_command) = 0;

  /*!
  * \brief kalman filter innovation step
  * \param sensor_measurements sensor measurements
  * \param front_servo_command the command for the front servo.
  * \param back_servo_command the command for the back servo.
  * \return state estimation
  */
  virtual VehicleState innovate(const SensorMeasurements& sensor_measurements,
                                double front_servo_command,
                                double back_servo_command) = 0;

  /*!
  * \brief resets the state and the state covariance - WARNING: this method is
  * not real-time-capable
  */
  void reset();

  /*!
   * \brief updateParams updates the parameters of the kalman filter. Not
   * real-time-capable, only call from parameter polling thread or at startup.
   */
  void updateParams();

 protected:
  /*!
   * \brief car_specs the car_specs parameters
   */
  CarSpecs car_specs;

  /*!
   * \brief kalman_params the parameters of the kalman filter
   */
  ParameterInterface& parameter_interface;

  /*!
  * \brief time between updates (rate at which measurements are received and
  * estimations are published (if the estimations are plausible))
  */
  double time_between_updates;

 private:
  /*!
   * \brief resetStateAndStateCovariance implementation dependant part of
   * reset()
   */
  virtual void resetStateAndStateCovariance() = 0;

  /*!
   * \brief updateKalmanFilterParams implementation dependant part of
   * updateParams()
   */
  virtual void updateKalmanFilterParams() = 0;
};


#endif  // KALMANFILTER_H
