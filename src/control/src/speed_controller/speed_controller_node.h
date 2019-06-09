#ifndef SPEED_CONTROLLER_NODE_H
#define SPEED_CONTROLLER_NODE_H
#include <common/macros.h>

THIRD_PARTY_HEADERS_BEGIN
#include <std_msgs/Bool.h>
#include <std_msgs/Empty.h>
#include <std_srvs/Empty.h>
THIRD_PARTY_HEADERS_END

#include "common/node_base.h"
#include "longitudinal_controller_msgs/DrivingSpeed.h"
#include "speed_controller.h"

/*!
 * \brief Node that controls the speed of the car. The controller has real-time
 * constraints and directly interfaces the Engine actuator.
 */
class SpeedControllerNode : public NodeBase {
 public:
  /*!
   * \brief SpeedControllerNode the constructor.
   * \param node_handle the NodeHandle to be used.
   */
  SpeedControllerNode(ros::NodeHandle &node_handle);

  virtual ~SpeedControllerNode() override;

  /*!
   * \brief returns the name of the node. It is needed in main and onInit
   * (nodelet) method.
   * \return the name of the node
   */
  static const std::string getName();

  /*!
   * \brief onVelocityCommandReceived callback for the drive command coming from
   * the high level controller containing the target speed
   * \param msg the message containing the target speed value
   */
  void onVelocityCommandReceived(const longitudinal_controller_msgs::DrivingSpeedConstPtr &msg);

  /*!
   * \brief onEmergencyStopReceived callback for the emergency stop topic used
   * to stop the vehicle as fast as possible circumventing the path planning and
   * high_level_controller
   * \param msg true for activating the emergency stop, false for deactivating
   * the emergency stop
   */
  void onEmergencyStopReceived(std_msgs::Bool msg);

  /**
   * @brief debugPublisherLoop runs in a dedicated thread and
   * publishes debug values to a ros topic
   */
  void debugPublisherLoop();

  /*!
   * \brief resetCallback callback of the reset service to reset the speed
   * controller (used after completing stopping by the high level controller)
   * \return wheter the function succeeded.
   */
  bool resetCallback(std_srvs::EmptyRequest &, std_srvs::EmptyResponse &);

  void autoResetCallback(const std_msgs::Empty &auto_reset);

 protected:
  // NodeBase interface
  /*!
   * \brief startModule is called, if the node shall be turned active. In here
   * the subscribers and publishers are started.
   */
  virtual void startModule() override = 0;
  /*!
   * \brief stopModule is called, if the node shall be turned inactive. In this
   * function subscribers and publishers are shutdown.
   */
  virtual void stopModule() override = 0;

  /*!
   * \brief speed_controller contains the ROS-independent implementation of this
   * node.
   */
  std::unique_ptr<SpeedController> speed_controller_;

  /*!
   * \brief velocity_command_subscriber_ the ros subscriber for the drive
   * command with the target speed
   */
  ros::Subscriber velocity_command_subscriber_;
  /*!
   * \brief emergency_stop_command_subscriber_ the ros subscriber for the
   * emergency stop topic
   */
  ros::Subscriber emergency_stop_command_subscriber_;

  ros::Subscriber auto_reset_subscriber_;

  /*!
   * \brief reset_service_server_ the ros service server for the reset service
   */
  ros::ServiceServer reset_service_server_;

  //! debug publishers

  /*!
   * \brief engine_power_publisher_ the ros publisher for the engine power. This
   * is a debug publisher intended for e.g. visualization, the
   * controller_interface (and thus the motor) get their actual values over
   * shared memory in the speed_controller. This publisher runs in it's own
   * thread to not interfere with the actual realtime-capable controller.
   */
  ros::Publisher engine_power_publisher_;

  /*!
   * \brief speed_error_publisher_ the ros publisher for the speed error. This
   * is a debug publisher intended for e.g. visualization. This publisher runs
   * in it's own thread to not interfere with the actual realtime-capable
   * controller.
   */
  ros::Publisher speed_error_publisher_;

  /*!
   * \brief engine_power_publisher_thread_ the thread used for publishing the
   * engine power for debugging purposes
   */
  boost::thread debug_publisher_thread_;
};

class SpeedControllerNodeRealtime : public SpeedControllerNode {
 public:
  SpeedControllerNodeRealtime(ros::NodeHandle &node_handle);

 protected:
  virtual void startModule() override;
  virtual void stopModule() override;
};

#endif  // SPEED_CONTROLLER_NODE_H
