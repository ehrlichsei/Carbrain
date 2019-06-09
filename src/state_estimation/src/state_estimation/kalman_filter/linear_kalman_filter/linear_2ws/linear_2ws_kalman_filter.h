#ifndef LINEAR2_WSKALMAN_FILTER_H
#define LINEAR2_WSKALMAN_FILTER_H

#include <common/macros.h>

THIRD_PARTY_HEADERS_BEGIN

THIRD_PARTY_HEADERS_END

#include "../linear_square_root_kalman_filter.h"


class Linear2WSKalmanFilter : public virtual LinearSquareRootKalmanFilter<3u, 4u> {
 public:
  using LinearSquareRootKalmanFilter<3u, 4u>::KF;

  Linear2WSKalmanFilter(double update_rate, ParameterInterface& params);

  /*!
   * \brief updateKalmanFilterParams polls the parameters of the kalman filter.
   * Not
   * real-time-capable, only call from parameter polling thread or at startup.
   */
  virtual void updateKalmanFilterParams() override;

 protected:
  /*!
   * \brief updateSystemModel sets/updates the system matrix, the control vector
   * and the system noise matrix
   */
  virtual void updateSystemModel(double front_servo_command, double back_servo_command) override;

  /*!
   * \brief updateMeasurementModel sets/updates the measurement matrix and the
   * measurement noise matrix
   */
  virtual void updateMeasurementModel(double front_servo_command,
                                      double back_servo_command) override;

  /*!
  * \brief state vector (Eigen vector) to vehicle state (struct)
  * \param state vector (Eigen vector)
  * \return vehicle_state vehicle state (struct)
  */
  virtual VehicleState toVehicleState(const typename KF::StateVector& state) override;


  /*!
   * \brief toMeasurementVector SensorMeasurements (struct) to MeasurementVector
   * (Eigen vector)
   * \param sensor_measurements (struct)
   * \return MeasurementVector (Eigen vector)
   */
  virtual typename KF::MeasurementVector toMeasurementVector(const SensorMeasurements& sensor_measurements) override;


  double calculateFrontSteeringAngle();

 private:
  /*!
   * \brief the parameters of the kalman filter
   */
  struct Parameters {
    static const ParameterString<double> SYSTEM_NOISE_SPEED;
    static const ParameterString<double> SYSTEM_NOISE_YAW_RATE;
    static const ParameterString<double> SYSTEM_NOISE_ACCELERATION;
    static const ParameterString<double> MEASUREMENT_NOISE_ANGULAR_SPEED_LEFT;
    static const ParameterString<double> MEASUREMENT_NOISE_ANGULAR_SPEED_RIGHT;
    static const ParameterString<double> MEASUREMENT_NOISE_ACCELERATION;
    static const ParameterString<double> MEASUREMENT_NOISE_YAW_RATE;
    static const ParameterString<double> MIN_ABS_SPEED_FOR_STEERING_ANGLE_CALCULATION;

    Parameters(const ParameterInterface& parameter_interface);

    static Parameters makeAndRegister(ParameterInterface& parameter_interface);

    /*!
    * \brief system noise matrix diagonal element for speed
    */
    double system_noise_speed;
    /*!
    * \brief system noise matrix diagonal element for yaw rate
    */
    double system_noise_yaw_rate;
    /*!
    * \brief system noise matrix diagonal element for acceleration
    */
    double system_noise_acceleration;


    /*!
    * \brief measurement noise matrix diagonal element for left  wheel encoder
    */
    double measurement_noise_angular_speed_left;
    /*!
    * \brief measurement noise matrix diagonal element for right  wheel encoder
    */
    double measurement_noise_angular_speed_right;
    /*!
    * \brief measurement noise matrix diagonal element for acceleration
    * measurement
    */
    double measurement_noise_acceleration;
    /*!
    * \brief measurement noise matrix diagonal element for yaw rate measurement
    */
    double measurement_noise_yaw_rate;

    /*!
     * \brief min_absolute_speed_for_steering_angle_calculation for speeds below
     * below min_abs_velocity, the
     * state estimation publishes an angle of NaN since for low speeds the
     * current
     * steering angle cannot be
     * calculated reliably
     */
    double min_absolute_speed_for_steering_angle_calculation;
  } params;
};



#endif  // LINEAR2_WSKALMAN_FILTER_H
