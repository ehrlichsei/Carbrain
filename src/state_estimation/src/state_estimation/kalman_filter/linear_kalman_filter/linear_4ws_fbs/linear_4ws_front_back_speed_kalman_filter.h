#ifndef LINEAR_4WS_FRONT_BACK_SPEED_KALMAN_FILTER_H
#define LINEAR_4WS_FRONT_BACK_SPEED_KALMAN_FILTER_H

#include <common/macros.h>

THIRD_PARTY_HEADERS_BEGIN

THIRD_PARTY_HEADERS_END

#include "../linear_square_root_kalman_filter.h"


class Linear4WSFrontBackSpeedKalmanFilter
    : public virtual LinearSquareRootKalmanFilter<3u, 6u> {
 public:
  using LinearSquareRootKalmanFilter<3u, 6u>::KF;

  Linear4WSFrontBackSpeedKalmanFilter(double update_rate, ParameterInterface& params);

  /*!
   * \brief updateKalmanFilterParams polls the parameters of the kalman filter.
   * Not
   * real-time-capable, only call from parameter polling thread or at startup.
   */
  virtual void updateKalmanFilterParams() override;

 protected:
  /*!
   * \brief front_servo_characteristic the characteristic curve of the front
   * servo that allows the calculation of a servo set value from a steering
   * angle or a steering angle from a servo set value
   */
  ServoCharacteristic front_servo_characteristic;

  /*!
   * \brief front_servo_characteristic the characteristic curve of the back
   * servo that allows the calculation of a servo set value from a steering
   * angle or a steering angle from a servo set value
   */
  ServoCharacteristic back_servo_characteristic;

  /*!
   * \brief steering_angle_front front axle steering angle calculated from front
   * servo set point (not usable for low-level-steering feedback control)
   */
  double steering_angle_front = 0.0;

  /*!
   * \brief steering_angle_back rear axle steering angle calculated from back
   * servo set point (not usable for low-level-steering feedback control)
   */
  double steering_angle_back = 0.0;

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
  virtual VehicleState toVehicleState(const KF::StateVector& state) override;

  /*!
   * \brief toMeasurementVector SensorMeasurements (struct) to MeasurementVector
   * (Eigen vector)
   * \param measures (struct)
   * \return MeasurementVector (Eigen vector)
   */
  virtual KF::MeasurementVector toMeasurementVector(const SensorMeasurements& measures) override;

 private:
  /*!
   * \brief the parameters of the kalman filter
   */
  struct Parameters {
    static const ParameterString<double> SYSTEM_NOISE_SPEED_FRONT;
    static const ParameterString<double> SYSTEM_NOISE_SPEED_BACK;
    static const ParameterString<double> SYSTEM_NOISE_YAW_RATE;
    static const ParameterString<double> MEASUREMENT_NOISE_ANGULAR_SPEED_FRONT_LEFT;
    static const ParameterString<double> MEASUREMENT_NOISE_ANGULAR_SPEED_BACK_LEFT;
    static const ParameterString<double> MEASUREMENT_NOISE_ANGULAR_SPEED_FRONT_RIGHT;
    static const ParameterString<double> MEASUREMENT_NOISE_ANGULAR_SPEED_BACK_RIGHT;
    static const ParameterString<double> MEASUREMENT_NOISE_YAW_RATE;

    Parameters(const ParameterInterface& parameter_interface);

    static Parameters makeAndRegister(ParameterInterface& parameter_interface);

    /*!
    * \brief system noise matrix diagonal element for speed in x-direction
    * (longitudinal)
    */
    double system_noise_speed_front;
    /*!
    * \brief system noise matrix diagonal element for speed in y-direction
    * (lateral)
    */
    double system_noise_speed_back;
    /*!
    * \brief system noise matrix diagonal element for yaw rate (angular speed
    * around z-axis)
    */
    double system_noise_yaw_rate;


    /*!
    * \brief measurement noise matrix diagonal element for front left wheel
    * encoder
    */
    double measurement_noise_angular_speed_front_left;
    /*!
    * \brief measurement noise matrix diagonal element for back left wheel
    * encoder
    */
    double measurement_noise_angular_speed_back_left;
    /*!
    * \brief measurement noise matrix diagonal element for front right wheel
    * encoder
    */
    double measurement_noise_angular_speed_front_right;
    /*!
    * \brief measurement noise matrix diagonal element for back right wheel
    * encoder
    */
    double measurement_noise_angular_speed_back_right;
    /*!
    * \brief measurement noise matrix diagonal element for yaw rate measurement
    * (angular speed around z-axis)
    */
    double measurement_noise_yaw_rate;


  } params;
};


#endif  // LINEAR_4WS_FRONT_BACK_SPEED_KALMAN_FILTER_H
