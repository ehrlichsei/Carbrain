#ifndef STEERING_CONTROLLER_H
#define STEERING_CONTROLLER_H
#include <common/macros.h>

THIRD_PARTY_HEADERS_BEGIN
#include <control_toolbox/pid.h>
THIRD_PARTY_HEADERS_END

#include "common/parameter_interface.h"

#include <common/realtimecontroller.h>
#include <common/realtimeipc.h>
#include "common/concurrent_queue.h"
#include <common/polynomial.h>
#include <common/debug.h>
#include "state_estimation_msgs/State.h"

#include <common/realtime_channel_ids.h>

const ParameterString<double> PARAM_P_STEERING =
    ParameterString<double>("p_steering");
const ParameterString<double> PARAM_I_STEERING =
    ParameterString<double>("i_steering");
const ParameterString<double> PARAM_D_STEERING =
    ParameterString<double>("d_steering");
const ParameterString<double> PARAM_I_MAX = ParameterString<double>("i_max");
const ParameterString<double> PARAM_I_MIN = ParameterString<double>("i_min");
const ParameterString<double> PARAM_FEED_FORWARD_A =
    ParameterString<double>("feed_forward/servo_characteristic_a");
const ParameterString<double> PARAM_FEED_FORWARD_B =
    ParameterString<double>("feed_forward/servo_characteristic_b");
const ParameterString<double> PARAM_FEED_FORWARD_C =
    ParameterString<double>("feed_forward/servo_characteristic_c");
const ParameterString<double> PARAM_FEED_FORWARD_D =
    ParameterString<double>("feed_forward/servo_characteristic_d");
const ParameterString<bool> PARAM_USE_ANTIWINDUP =
    ParameterString<bool>("use_antiwindup");


class SteeringController {
 public:
  SteeringController() = default;

  SteeringController(ParameterInterface *parameters);

  virtual ~SteeringController() = default;


  virtual void start() {}
  virtual void stop() {}

  /*!
   * \brief setTargetAngle setter method used to set target_angle_ from the node
   * \param target_angle the new target angle.
   */
  void setTargetAngle(float target_angle);

  /*!
   * \brief reset pid and set steering value to a default value
   */
  virtual void reset();

  /*!
   * \brief getNextServoSetValue returns the next available controller output
   * that has been calculated
   * if no new servo set value is available yet, it blocks until a new servo set
   * value is available or if the thread gets interrupted.
   */
  bool getNextServoSetValue(double *);

  /*!
   * \brief getNextAngleError returns the next available controller error that
   * has been calculated
   * if no new angle_error value is available yet, it blocks until a new
   * angle_error is available or if the thread gets interrupted.
   */
  bool getNextAngleError(double *);

  /*!
   * \brief struct
   * storage for the calculated speed commands which will than be pushed into
   * the realtime IPCs or, if in debug mode, published over ROS
   */
  struct SteeringControlCommands {
    boost::optional<float> angle_error;
    float steering_output = 0.5;
  };


  SteeringControlCommands calcSteeringCommands(boost::chrono::nanoseconds time_since_last_update,
                                               float current_steering_angle,
                                               bool manual_mode);


  /*!
   * \brief setParams to set the params for the PID controller as well as
   * other params to calculate the SteeringCommands
   */
  void setParams();

  /*!
   * \brief servo_set_value_publisher_queue_ the queue used to buffer the servo
   * set values before they get published to the debug servo set value topic.
   */
  common::ConcurrentQueue<double, 10> servo_set_value_publisher_queue_;
  /*!
   * \brief angle_error_publisher_queue_ the queue used to buffer the angle
   * errors before they get published to the debug angle error topic.
   */
  common::ConcurrentQueue<double, 10> angle_error_publisher_queue_;

 private:
  /*!
   * \brief target_angle_ the set point of the controller
   */
  float target_angle_ = 0.0;

  /*!
   * \brief feed_forward_ the polynomial that represents the feedforward control
   */
  common::CubicPolynomial feed_forward_{0.5};

 protected:
  /*!
  * \brief parameters_ptr_ is needed for parameter access
  */
  const ParameterInterface *parameters_ptr_;

  /*!
   * \brief pid_ is the actual implementation of a pid controller
   */
  control_toolbox::Pid pid_;

  static void registerParameters(ParameterInterface *parameters);
};


/*!
 * \brief The SteeringController is a pid controller for the steering angle of
 * the car.
 */
class SteeringControllerRealtime : public common::RealtimeController, public SteeringController {
 public:
  SteeringControllerRealtime(ParameterInterface *parameters,
                             int channel_id_steering_command,
                             int channel_id_steering_angle_measure);

  // RealtimeController interface
  /*!
   * \brief update is the real-time control loop method.
   * execution time must be below the configured time interval.
   */
  virtual void update(boost::chrono::nanoseconds) override;

  /*!
   * \brief reset will be called from the realtime controller.
   */
  virtual void reset() override;

  /*!
   * \brief gets called in regular intervals to perform parameter updates. Runs
   * in own thread to ensure real time capability of main thread. All parameter
   * updates (except of initialisation) here.
   */
  virtual void pollParameters() override;

  virtual void start() override;
  virtual void stop() override;

  /*!
   * \brief setTargetAngle setter method used to set target_angle_ from the node
   * \param target_angle the new target angle.
   */
  void setTargetAngle(float target_angle);


  /*!
   * \brief getNextServoSetValue returns the next available controller output
   * that has been calculated if no new servo set value is available yet, it
   * blocks until a new servo set value is available or if the thread gets
   * interrupted.
   */
  bool getNextServoSetValue(double *);

  /*!
   * \brief getNextAngleError returns the next available controller error that
   * has been calculated
   * if no new angle_error value is available yet, it blocks until a new
   * angle_error is
   * available or if the thread gets interrupted.
   */
  bool getNextAngleError(double *);

 private:
  // input IPCs
  /*!
   * \brief manual_mode_ is a real-time communication channel from the
   * controller_interface node
   */
  common::RealtimeIPC<float> steering_angle_measure_;

  /*!
   * \brief manual_mode_ is a real-time communication channel from the
   * controller_interface node
   */
  common::RealtimeIPC<bool> manual_mode_;


  // output IPCs
  /*!
   * \brief steering_control_ is a real-time communication channel to the
   * controller_interface node
   */
  common::RealtimeIPC<float> steering_control_;
};
#endif  // STEERING_CONTROLLER_H
