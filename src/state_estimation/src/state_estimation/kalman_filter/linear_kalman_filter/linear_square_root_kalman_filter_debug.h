#ifndef LINEAR_SQUARE_ROOT_KALMAN_FILTER_DEBUG_H
#define LINEAR_SQUARE_ROOT_KALMAN_FILTER_DEBUG_H

#include <common/macros.h>

THIRD_PARTY_HEADERS_BEGIN

THIRD_PARTY_HEADERS_END

#include "linear_square_root_kalman_filter.h"


/*!
 * \brief calculateConditionNumber template function to calculate condition
 * number of a matrix (for debugging purposes, not realtime capable)
 * \param matrix the matrix.
 * \return condition number of matrix
 */
template <typename T>
double calculateConditionNumber(const T& matrix) {
  Eigen::JacobiSVD<Eigen::MatrixXd> svd(matrix);
  double cond = svd.singularValues()(0) /
                svd.singularValues()(svd.singularValues().size() - 1);
  return cond;
}


/*!
 * \brief The LinearSquareRootKalmanFilterDebug class publishes the kalman gain,
 * the state
 * covariance matrix root and their condition numbers.
 * This class is not realtime-capable
 */
template <unsigned int state_dim, unsigned int measurement_dim>
class LinearSquareRootKalmanFilterDebug
    : public virtual LinearSquareRootKalmanFilter<state_dim, measurement_dim> {

 public:
  using typename LinearSquareRootKalmanFilter<state_dim, measurement_dim>::KF;

  LinearSquareRootKalmanFilterDebug(double update_rate,
                                    ParameterInterface& params,
                                    ros::NodeHandle* node_handle)
      : LinearSquareRootKalmanFilter<state_dim, measurement_dim>(update_rate, params) {
    kalman_gain_publisher = node_handle->advertise<std_msgs::Float64MultiArray>(
        "debug/kalman_gain", 4, true);
    state_covariance_publisher = node_handle->advertise<std_msgs::Float64MultiArray>(
        "debug/state_covariance_root", 4, true);
    state_covariance_condition_number_publisher = node_handle->advertise<std_msgs::Float64>(
        "debug/state_covariance_root_condition_number", 4, true);
    kalman_gain_condition_number_publisher = node_handle->advertise<std_msgs::Float64>(
        "debug/kalman_gain_condition_number", 4, true);
    innovation_signal_covariance_condition_number_publisher =
        node_handle->advertise<std_msgs::Float64>(
            "debug/innovation_signal_covariance_condition_number", 4, true);
  }

  /*!
  * \brief kalman filter prediction step
  * \return prediction
  */
  virtual VehicleState predict(double front_servo_command,
                               double back_servo_command,
                               double engine_command) override {
    return LinearSquareRootKalmanFilter<state_dim, measurement_dim>::predict(
        front_servo_command, back_servo_command, engine_command);
  }

  /*!
  * \brief kalman filter innovation step
  * \param sensor_measurements sensor measurements
  * \return state estimation
  */
  virtual VehicleState innovate(const SensorMeasurements& sensor_measurements,
                                double front_servo_command,
                                double back_servo_command) override {
    VehicleState state = LinearSquareRootKalmanFilter<state_dim, measurement_dim>::innovate(
        sensor_measurements, front_servo_command, back_servo_command);

    publishKalmanGainConditionNumber(
        calculateConditionNumber(this->kalman_filter.getKalmanGain()));
    publishStateCovarianceRootConditionNumber(calculateConditionNumber(
        this->kalman_filter.getStateCovarianceMatrixRoot()));
    publishInnovationSignalCovarianceConditionNumber(calculateConditionNumber(
        this->kalman_filter.getInnovationSignalCovariance()));
    publishKalmanGain(this->kalman_filter.getKalmanGain());
    publishStateCovarianceRoot(this->kalman_filter.getStateCovarianceMatrixRoot());
    return state;
  }

 private:
  void publishKalmanGain(
      const typename LinearSquareRootKalmanFilterDebug<state_dim, measurement_dim>::KF::KalmanGainMatrix& kalman_gain) {
    std_msgs::Float64MultiArray msg;
    tf::template matrixEigenToMsg<typename common::KalmanFilter<state_dim, measurement_dim>::KalmanGainMatrix>(
        kalman_gain, msg);
    kalman_gain_publisher.publish(msg);
  }
  void publishStateCovarianceRoot(
      const typename LinearSquareRootKalmanFilterDebug<state_dim, measurement_dim>::KF::StateCovarianceMatrix& state_covariance_root) {
    std_msgs::Float64MultiArray msg;
    tf::template matrixEigenToMsg<typename common::KalmanFilter<state_dim, measurement_dim>::StateCovarianceMatrix>(
        state_covariance_root, msg);
    state_covariance_publisher.publish(msg);
  }
  void publishKalmanGainConditionNumber(double condition_number) {
    std_msgs::Float64 msg;
    msg.data = condition_number;
    state_covariance_condition_number_publisher.publish(msg);
  }
  void publishStateCovarianceRootConditionNumber(double condition_number) {
    std_msgs::Float64 msg;
    msg.data = condition_number;
    kalman_gain_condition_number_publisher.publish(msg);
  }
  void publishInnovationSignalCovarianceConditionNumber(double condition_number) {
    std_msgs::Float64 msg;
    msg.data = condition_number;
    innovation_signal_covariance_condition_number_publisher.publish(msg);
  }

  /*!
   * \brief kalman_gain_publisher ros publisher for the kalman gain matrix
   */
  ros::Publisher kalman_gain_publisher;
  /*!
   * \brief state_covariance_publisher ros publisher for the state covariance
   * matrix root
   */
  ros::Publisher state_covariance_publisher;
  /*!
   * \brief kalman_gain_condition_number_publisher ros publisher for the kalman
   * gain matrix condition number
   */
  ros::Publisher kalman_gain_condition_number_publisher;
  /*!
   * \brief state_covariance_condition_number_publisher ros publisher for the
   * state covariance matrix root condition number
   */
  ros::Publisher state_covariance_condition_number_publisher;

  /*!
   * \brief innovation_signal_covariance_condition_number_publisher ros
   * publisher for the signal covariance condition number, which is a good
   * measure of the numerical stability of the square root kalman filter
   */
  ros::Publisher innovation_signal_covariance_condition_number_publisher;
};

#endif  // LINEAR_SQUARE_ROOT_KALMAN_FILTER_DEBUG_H
