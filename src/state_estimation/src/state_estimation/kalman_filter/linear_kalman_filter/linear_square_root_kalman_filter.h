#ifndef LINEAR_SQUARE_ROOT_KALMAN_FILTER_H
#define LINEAR_SQUARE_ROOT_KALMAN_FILTER_H

#include <common/macros.h>

THIRD_PARTY_HEADERS_BEGIN

THIRD_PARTY_HEADERS_END

#include "../kalman_filter.h"

/*!
 * \brief The LinearSquareRootKalmanFilter class template base class for linear
 * (time variant or time invariant) square root kalman filter implementations
 * state_dim: dimension of state vector
 * measurement_dim: dimension of measurement vector
 */
template <unsigned int state_dim, unsigned int measurement_dim>
class LinearSquareRootKalmanFilter : public KalmanFilter {

 public:
  LinearSquareRootKalmanFilter(double update_rate, ParameterInterface& params)
      : KalmanFilter(update_rate, params) {}

  using KF = typename common::KalmanFilter<state_dim, measurement_dim>;

  /*!
  * \brief kalman filter prediction step
  * \return prediction
  */
  virtual VehicleState predict(double front_servo_command,
                               double back_servo_command,
                               double /*engine_command*/) override {
    updateSystemModel(front_servo_command, back_servo_command);
    const typename KF::StateVector prediction = kalman_filter.predict();
    return toVehicleState(prediction);
  }

  /*!
  * \brief kalman filter innovation step
  * \param sensor_measurements sensor measurements
  * \return state estimation
  */
  virtual VehicleState innovate(const SensorMeasurements& sensor_measurements,
                                double front_servo_command,
                                double back_servo_command) override {
    updateMeasurementModel(front_servo_command, back_servo_command);

    const typename KF::MeasurementVector measurement_vector =
        toMeasurementVector(sensor_measurements);

    const typename KF::StateVector innovation = kalman_filter.innovate(measurement_vector);
    return toVehicleState(innovation);
  }

 protected:
  /*!
   * \brief kalman_filter the actual kalman filter implementation
   */
  common::KalmanFilter<state_dim, measurement_dim> kalman_filter;

  /*!
   * \brief updateSystemModel sets/updates the system matrix, the control vector
   * and the system noise matrix of the kalman_filter
   */
  virtual void updateSystemModel(double front_servo_command, double back_servo_command) = 0;

  /*!
   * \brief updateMeasurementModel sets/updates the measurement matrix and the
   * measurement noise matrix of the kalman_filter
   */
  virtual void updateMeasurementModel(double front_servo_command,
                                      double back_servo_command) = 0;

  /*!
  * \brief state vector (Eigen vector) to vehicle state (struct)
  * \param state vector (Eigen vector)
  * \return vehicle_state vehicle state (struct)
  */
  virtual VehicleState toVehicleState(const typename KF::StateVector& state) = 0;

  /*!
   * \brief toMeasurementVector SensorMeasurements (struct) to MeasurementVector
   * (Eigen vector)
   * \param measures (struct)
   * \return MeasurementVector (Eigen vector)
   */
  virtual typename KF::MeasurementVector toMeasurementVector(const SensorMeasurements& measures) = 0;

 private:
  virtual void resetStateAndStateCovariance() final override {
    kalman_filter.resetStateAndStateCovariance();
  }
};


#endif  // LINEAR_SQUARE_ROOT_KALMAN_FILTER_H
