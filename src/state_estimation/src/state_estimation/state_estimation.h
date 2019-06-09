#ifndef STATE_ESTIMATION_H
#define STATE_ESTIMATION_H

#include "common/parameter_interface.h"
#include <controller_interface/sensormeasurements.h>
#include "state.h"
#include <common/realtimecontroller.h>
#include <common/realtimeipc.h>
#include <common/concurrent_queue.h>
#include <common/runtime_analyzer.h>
#include "kalman_filter/kalman_filter.h"


THIRD_PARTY_HEADERS_BEGIN
#include <memory>
THIRD_PARTY_HEADERS_END

/*!
 * \brief Filters the incoming sensor data with a kalman filter to estimate the
 * current state of the car. Performs plausibility checks.
 */
class StateEstimation {
 public:
  StateEstimation(ParameterInterface* parameters, std::unique_ptr<KalmanFilter> kalman_filter);

  virtual ~StateEstimation() = default;

  /*!
   * \brief processSensorData processes all sensor samples at once and returns
   * all intermediate states - this function is only used in integration test
   * mode
   * \param test_data the list of measurements
   * \return all intermediate states, one for each measurement
   */
  std::vector<VehicleState> processSensorData(
      const std::vector<std::tuple<SensorMeasurements, float, float, float>>& test_data);

  /*!
   * \brief getNextEstimatedState returns the next available state that has been
   * estimated
   * if no new state is available yet, it blocks until a new state estimation is
   * available or if the thread gets interrupted.
   */
  bool getNextEstimatedState(VehicleState*);

  /*!
  * \brief performKalmanIteration calls the prediction and innovation step of
  * the kalman filter, checks the estimation for plausibility and resets the
  * kalman filter if necessary
  * \param measure the sensor measurements
  */
  VehicleState performKalmanIteration(const SensorMeasurements& measure,
                                      float front_servo_command,
                                      float back_servo_command,
                                      float engine_command);

  /*!
   * \brief updateParameters updates the state estimation and kalman filter
   * parameters
   */
  void updateParameters();

  /*!
   * \brief getUpdateRate get the update rate of the kalman filter
   * \return the update rate of the kalman filter
   */
  virtual int getUpdateRate() const = 0;

  void resetKalmanFilter();

  virtual void start();

  virtual void stop() {}

  /*!
   * \brief estimated_state_results_queue_ this queue buffers the estimated
   * vehicle states before they get published on the state estimation ros topic
   */
  common::ConcurrentQueue<VehicleState, 10> estimated_state_results_queue_;

 protected:
  RuntimeAnalyzer runtime_analyzer_;

 private:
  static const ParameterString<double> MAX_SPEED;
  static const ParameterString<double> MAX_YAW_RATE;
  static const ParameterString<double> MAX_ACCELERATION;
  static const ParameterString<double> MAX_WHEEL_SPEED_MEASURE;
  static const ParameterString<double> MAX_YAW_RATE_MEASURE;
  static const ParameterString<double> MAX_ACCELERATION_MEASURE;
  static const ParameterString<double> MAX_STEERING_ANGLE_MEASURE;

  /*!
  * \brief The kalman filter. All filtering operations are performed in the
  * functions of this object.
  */
  std::unique_ptr<KalmanFilter> kalman_filter_;

  /*!
  * \brief parameters_ptr_ is needed for parameter access
  */
  const ParameterInterface* parameters_ptr_;

  /*!
   * \brief registerStateEstimationParameters registers the parameters of the
   * state estimation (but not those of the kalman filter) on the parameter
   * server
   * \param parameters the parameterinterface
   */
  void registerStateEstimationParameters(ParameterInterface* parameters);

  /*!
   * \brief setStateEstimationParameters requests the state estimation
   * parameters (but not those of the kalman filter) from the parameter server
   * \param parameters the parameterinterface
   */
  void setStateEstimationParameters(const ParameterInterface* parameters);

  /*!
   * \brief measurementsPlausible checks whether the sensor measurements are
   * plausible
   * \param measurements the measurements to check.
   * \return true if the measurements are plausible, false if the measurements
   * are implausible
   */
  bool measurementsPlausible(const SensorMeasurements& measurements) const;

  /*!
  * \brief max_speed is the maximal (absolute) speed estimation the state
  * estimation considers plausible and will publish. Higher (absolute) speeds
  * result in a reset of the state.
  */
  double max_speed = std::numeric_limits<double>::max();
  /*!
  * \brief max_yaw_rate is the maximal (absolute) yaw rate the state estimation
  * considers plausible and will publish. Higher (absolute) yaw rates result in
  * a reset of the state.
  */
  double max_yaw_rate = std::numeric_limits<double>::max();
  /*!
  * \brief max_acceleration is the maximal (absolute) acceleration estimation
  * the state estimation considers plausible and will publish. Higher (absolute)
  * accelerations result in a reset of the state.
  */
  double max_acceleration = std::numeric_limits<double>::max();

  /*!
  * \brief max_wheel_speed_measure is the maximal (absolute) wheel speed measure
  * the state
  * estimation considers plausible. Higher (absolute) wheel speeds
  * result in the skipping of the innovation step.
  */
  double max_wheel_speed_measure = std::numeric_limits<double>::max();
  /*!
  * \brief max_acceleration_measure is the maximal (absolute) acceleration
  * measure the state
  * estimation considers plausible. Higher (absolute) accelerations
  * result in the skipping of the innovation step.
  */
  double max_acceleration_measure = std::numeric_limits<double>::max();
  /*!
  * \brief max_yaw_rate_measure is the maximal (absolute) yaw rate measure the
  * state
  * estimation considers plausible. Higher (absolute) yaw rates
  * result in the skipping of the innovation step.
  */
  double max_yaw_rate_measure = std::numeric_limits<double>::max();
  /*!
  * \brief max_steering_angle_measure is the maximal (absolute) steering angle
  * measure the state
  * estimation considers plausible. Higher (absolute) steering angles
  * result in the skipping of the innovation step.
  */
  double max_steering_angle_measure = std::numeric_limits<double>::max();
};

/*!
 * \brief Filters the incoming sensor data with a kalman filter to estimate the
 * current state of the car and writes the results into shared memory.
 * Performs plausibility checks. If debug-mode is activated StateEstimationDebug
 * is run instad of StateEstimationRealtime.
 */
class StateEstimationRealtime : public common::RealtimeController, public StateEstimation {

 public:
  StateEstimationRealtime(ParameterInterface* parameters);

  /*!
   * \brief update is the real-time control loop method.
   * execution time must be below the configured time interval.
   */
  virtual void update(boost::chrono::nanoseconds) override;

  /*!
   * \brief gets called in regular intervals to perforresetKalmanFilterm
   * parameter updates. Runs
   * iown thread to ensure real time capability of main thread. All parameter
   * updates (except of initialisation) here.
   */
  virtual void pollParameters() override;

  /*!
   * \brief getUpdateRate get the update rate of the kalman filter
   * \return the update rate of the kalman filter
   */
  virtual int getUpdateRate() const override;

  virtual void start() override;

  virtual void stop() override;

 private:
  /*!
  * \brief sensor_measurements_ is the realtime Inter-Process-Communication
  * interface to the controller-interface for the sensor measurements
  */
  common::RealtimeIPC<SensorMeasurements> sensor_measurements_;

  // Communication from the real-time-controllers
  /*!
   * \brief front_servo_command_ipc is the output of the front steering low
   * level
   * controller
   */
  common::RealtimeIPC<float> front_servo_command_ipc;
  /*!
   * \brief back_servo_command_ipc is the output of the back steering low level
   * controller
   */
  common::RealtimeIPC<float> back_servo_command_ipc;
  /*!
   * \brief engine_command_ipc is the output of the longitudinal
   * controller and may take PWM or absolute values in ampere
   */
  common::RealtimeIPC<float> engine_command_ipc;

  // Communication to the real-time controllers
  /*!
  * \brief estimated_speed_ is the realtime Inter-Process-Communication
  * interface to the controllers for the speed estimation
  */
  common::RealtimeIPC<float> estimated_speed_;
  /*!
  * \brief estimated_yaw_rate_ is the realtime Inter-Process-Communication
  * interface to the controllers for the yaw rate estimation
  */
  common::RealtimeIPC<float> estimated_yaw_rate_;
  /*!
  * \brief estimated_acceleration_ is the realtime Inter-Process-Communication
  * interface to the controllers for the acceleration estimation
  */
  common::RealtimeIPC<float> estimated_acceleration_;
  /*!
  * \brief estimated_steering_angle_front is the realtime
  * Inter-Process-Communication interface to the controllers for the steering
  * angle estimation for the steering angle at the front axle
  */
  common::RealtimeIPC<float> estimated_steering_angle_front;
  /*!
  * \brief estimated_steering_angle_back is the realtime
  * Inter-Process-Communication interface to the controllers for the steering
  * angle estimation for the steering angle at the rear axle
  */
  common::RealtimeIPC<float> estimated_steering_angle_back;
};

/*!
 * \brief Filters the incoming sensor data with a kalman filter to estimate the
 * current state of the car.
 * Performs plausibility checks. This class is only used in debug mode.
 */
class StateEstimationDebug : public StateEstimation {
 public:
  StateEstimationDebug(ParameterInterface* parameters, ros::NodeHandle* node_handle);

  static int getUpdateRateParameter();

  /*!
   * \brief getUpdateRate get the update rate of the kalman filter
   * \return the update rate of the kalman filter
   */
  // TODO make double if this method returns double in RealtimeController
  virtual int getUpdateRate() const override;
};


#endif  // STATE_ESTIMATION_H
