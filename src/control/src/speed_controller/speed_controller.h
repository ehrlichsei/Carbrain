#ifndef SPEED_CONTROLLER_H
#define SPEED_CONTROLLER_H
#include <common/macros.h>

THIRD_PARTY_HEADERS_BEGIN
#include <control_toolbox/pid.h>
#include "state_estimation_msgs/State.h"
THIRD_PARTY_HEADERS_END

#include <common/polynomial.h>
#include <common/realtime_channel_ids.h>
#include "common/concurrent_queue.h"
#include "common/parameter_interface.h"
#include "common/realtimecontroller.h"
#include "common/realtimeipc.h"

const ParameterString<double> PARAM_P_SPEED =
    ParameterString<double>("p_speed");
const ParameterString<double> PARAM_I_SPEED =
    ParameterString<double>("i_speed");
const ParameterString<double> PARAM_D_SPEED =
    ParameterString<double>("d_speed");
const ParameterString<double> PARAM_I_MAX = ParameterString<double>("i_max");
const ParameterString<double> PARAM_I_MIN = ParameterString<double>("i_min");
const ParameterString<double> PARAM_FEED_FORWARD_A =
    ParameterString<double>("feed_forward_a");
const ParameterString<double> PARAM_FEED_FORWARD_B =
    ParameterString<double>("feed_forward_b");
const ParameterString<double> PARAM_FEED_FORWARD_C =
    ParameterString<double>("feed_forward_c");
const ParameterString<double> PARAM_MAX_ENGINE_POWER =
    ParameterString<double>("max_engine_power");
const ParameterString<bool> PARAM_USE_ANTIWINDUP =
    ParameterString<bool>("use_antiwindup");
const ParameterString<double> PARAM_DEAD_ZONE_SIZE =
    ParameterString<double>("dead_zone_size");
const ParameterString<double> PARAM_DEAD_ZONE_SPEED_SET_POINT_ACTIVATION_THRESHOLD =
    ParameterString<double>("dead_zone_activation_speed_set_point_threshold");
const ParameterString<double> PARAM_VELOCITY_COMMAND_TIMEOUT =
    ParameterString<double>("velocity_command_timeout");


class SpeedController {

 public:
  SpeedController() = default;
  SpeedController(ParameterInterface *parameters);

  virtual ~SpeedController() = default;

  virtual void start() {}
  virtual void stop() {}

  /*!
  * \brief engine_power_publisher_queue_ the queue used to buffer the engine
  * power values before they get published to the debug engine power topic.
  */
  common::ConcurrentQueue<double, 10> engine_power_publisher_queue_;

  /*!
  * \brief speed_error_publisher_queue_ the queue used to buffer the speed
  * error values before they get published to the debug speed error topic.
  */
  common::ConcurrentQueue<double, 10> speed_error_publisher_queue_;

  /*!
   * \brief setTargetSpeed setter method used to set target_speed_ from the node
   * \param target_speed the new target speed.
   */
  void setTargetSpeed(float target_speed, const ros::Time &velocity_command_timestamp);

  /*!
   * \brief struct
   * storage for the calculated speed commands which will than be pushed into
   * the realtime IPCs, or, if in debug mode, published over ROS
   */
  struct SpeedControllerCommands {
    float engine_power = 0.0;
    bool enable_brake_lights = false;
    float speed_error = 0.0;
  };

  /*!
   * \brief calcSpeedCommands calculated the engine_power and
   * speed_error and writes these in the class
   * intern storage struct speed_control_commands. It also decides whether to
   * set enable_brake_lights.
   * This method is called by the realtime controller via update method or
   * over handleState service in debug mode.
   */
  SpeedControllerCommands calcSpeedCommands(boost::chrono::nanoseconds, float, bool);

  /*!
   * \brief setEmergencyStop setter method used to set emergency_stop_ from the
   * node.
   * \param stop the new value.
   */
  void setEmergencyStop(bool stop);

  /*!
   * \brief reset pid and set speed to zero
   */
  void virtual reset();

  /*!
   * \brief getNextEnginePower returns the next available controller engine
   * power output that has been calculated
   * if no new engine_power value is available yet, it blocks until a new
   * engine_power is
   * available or if the thread gets interrupted.
   */
  bool getNextEnginePower(double *);

  /*!
   * \brief getNextSpeedError returns the next available controller error that
   * has been calculated
   * if no new speed_error value is available yet, it blocks until a new
   * speed_error is available or if the thread gets interrupted.
   */
  bool getNextSpeedError(double *);

  /*!
   * \brief setParams to set the params for the PID controller as well as
   * other params to calculate the speedCommands
   */
  void setParams();


 protected:
  /*!
   * \brief last_velocity_command_timestamp timestamp of last velocity command
   * to check for timeout of velocity command
   */
  ros::Time last_velocity_command_timestamp;

  /*!
   * \brief velocity_command_timeout timeout of velocity command
   */
  double velocity_command_timeout = 0.04;

  /*!
   * \brief dead_zone_size size of the dead zone (maximum absolute speed error
   * without controller reaction)
   */
  double dead_zone_size = 0.f;

  /*!
   * \brief dead_zone_speed_set_point_activation_threshold maximum absolute
   * speed set point value for which the dead zone is active
   */
  double dead_zone_speed_set_point_activation_threshold = 0.1;

  static void registerParameters(ParameterInterface *parameters);

  /*!
  * \brief parameters_ptr_ is needed for parameter access
  */
  const ParameterInterface *parameters_ptr_;

  /*!
  * \brief pid_ is the actual implementation of a pid controller
  */
  control_toolbox::Pid pid_;

  /*!
  * \brief feed_forward_ the polynomial that represents the feedforward control
  */
  common::QuadraticPolynomial feed_forward_{0};

  /*!
  * \brief target_speed_ the set point of the controller
  */
  float target_speed_ = 0.0;

  /*!
  * \brief emergency_stop_ activate emergency stop if true, continue driving if
  * false
  */
  bool emergency_stop_ = false;

  /*!
  * \brief limit the output engine power to [-max_engine_power,
  * max_engine_power]
  */
  float max_engine_power = 1.f;
};


class SpeedControllerRealtime : public common::RealtimeController, public SpeedController {
 public:
  SpeedControllerRealtime(ParameterInterface *parameters);

  /*!
   * \brief update will be called from the realtime controller. It will call the
   * calcSpeedCommands from the parent class and push the values into the
   * realtime IPC.
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

 private:
  // input IPCs
  /*!
   * \brief speed_measure_ is a real-time communication channel from the state
   * estimation node
   */
  common::RealtimeIPC<float> speed_measure_;

  /*!
   * \brief acceleration_measure_ is a real-time communication channel from the
   * state estimation node
   */
  common::RealtimeIPC<float> acceleration_measure_;

  /*! \brief manual_mode_ is a real-time communication channel from the
   * controller_interface node
   */
  common::RealtimeIPC<bool> manual_mode_;


  // output IPCs
  /*!
   * \brief engine_power is a real-time communication channel to the
   * controller_interface node
   */
  common::RealtimeIPC<float> engine_power_;

  /*!
   * \brief break_lights_ is a real-time communication channel to the
   * controller_interface node
   */
  common::RealtimeIPC<bool> brake_lights_;
};

#endif  // SPEED_CONTROLLER_H
