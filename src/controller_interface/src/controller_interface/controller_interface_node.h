#ifndef CONTROLLER_INTERFACE_NODE_H
#define CONTROLLER_INTERFACE_NODE_H
#include <common/macros.h>

THIRD_PARTY_HEADERS_BEGIN
#include <common_msgs/Float32Stamped.h>
#include <std_srvs/SetBool.h>
#include "common_msgs/MissionMode.h"
#include "controller_msgs/LightsCommand.h"
THIRD_PARTY_HEADERS_END

#include "common/node_base.h"
#include "controller_interface.h"

/*!
 * \brief manages the communication with Arduino
 */
class ControllerInterfaceNode : public NodeBase {
 public:
  /*!
   * \brief ControllerInterfaceNode the constructor.
   * \param node_handle the NodeHandle to be used.
   */
  ControllerInterfaceNode(ros::NodeHandle& node_handle);

  /*!
   * \brief ~ControllerInterfaceNode() the constructor which terminates al threads cleanly.
   */
  virtual ~ControllerInterfaceNode() override;
  /*!
   * \brief returns the name of the node. It is needed in main and onInit
   * (nodelet) method. \return the name of the node
   */
  static const std::string getName();

  /**
   * @brief controllerInterfacePublisherLoop runs in a dedicated thread and
   * publishes new measure information to a ros topic as it becomes available.
   */
  void controllerInterfacePublisherLoop();

  /**
   * @brief controllerInterfaceSnapshootLoop runs in a dedicated thread and
   * publishes snapshot requests to a ros topic as it becomes available
   */
  void controllerInterfaceSnapshotLoop();

  /**
   * @brief controllerInterfaceMissionModePublisherLoop runs in a dedicated thread and publishes new mode
   * information to a ros topic as it becomes available.
   */
  void controllerInterfaceMissionModePublisherLoop();

  /**
   * @brief controllerInterfaceDistanceSensorsLoop runs in a dedicated thread and publishes new distance sensor measures
   * information to a ros topic as it becomes available.
   */
  void controllerInterfaceDistanceSensorsLoop();

  void controllerInterfaceLoggingLoop();

  void onEnginePowerReceived(const common_msgs::Float32Stamped::ConstPtr& msg);
  void onSteeringControlReceived(const common_msgs::Float32Stamped::ConstPtr& msg);
  void onBackSteeringControlReceived(const common_msgs::Float32Stamped::ConstPtr& msg);
  void onLightsCommandReceived(const controller_msgs::LightsCommandConstPtr& msg);
  bool onBrakeLightCommandReceived(std_srvs::SetBool::Request& request,
                                   std_srvs::SetBool::Response& response);
  void onEngineBrakeReceived(const common_msgs::Float32Stamped::ConstPtr& msg);
  void onMissionMode(const common_msgs::MissionMode::ConstPtr& msg);
  void onDebugLightCommandReceived(const std_msgs::ColorRGBA::ConstPtr& msg);

 private:
  // NodeBase interface
  /*!
   * \brief startModule is called, if the node shall be turned active. In here
   * the subrscribers an publishers are started.
   */
  void startModule() override;
  /*!
   * \brief stopModule is called, if the node shall be turned inactive. In this
   * function subscribers and publishers are shutdown.
   */
  void stopModule() override;

  bool leftManualMode(const std_msgs::Bool& current_manual_mode_state);

  void sendIdleMode();

  const bool use_back_steering_;

  // needs to be on stack for realtime
  MeasuresHandler measures_handler_;
  CommandHandler command_handler_;

  bool last_manual_mode_ = false;

  /*!
   * \brief controller_interface contains the ROS-indipendent implementation of this node.
   */
  ControllerInterface controller_interface_;
  boost::thread controller_interface_publisher_thread_;
  boost::thread controller_interface_snaphost_publisher_thread_;
  boost::thread controller_interface_mission_mode_publisher_thread_;
  boost::thread controller_interface_logging_thread_;
  boost::thread controller_interface_distance_sensors_thread_;

  ros::Publisher state_measure_publisher_;
  ros::Publisher manual_mode_publisher_;
  ros::Publisher snapshot_request_publisher_;
  ros::Publisher auto_reset_publisher_;
  ros::Publisher mission_mode_publisher_;
  ros::Publisher front_ir_sensor_publisher_;
  ros::Publisher back_ir_sensor_publisher_;
  ros::Publisher front_us_sensor_publisher_;
  ros::Publisher back_us_sensor_publisher_;

  //! use these subscribers for debugging only!
  //! turn off speed_controller and steering_controller
  //! so they do not write on the realtimeIPC shared memory
  ros::Subscriber engine_power_subscriber_;
  ros::Subscriber steering_control_subscriber_;
  ros::Subscriber steering_control_back_subscriber_;

  ros::Subscriber blinker_command_subscriber_;
  ros::Subscriber debug_light_command_subscriber_;
  ros::Subscriber engine_brake_subscriber_;
  ros::Subscriber mission_mode_;

  ros::ServiceServer brake_lights_service_server_;
};

#endif  // CONTROLLER_INTERFACE_NODE_H
