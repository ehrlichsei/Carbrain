#ifndef STEERING_CONTROLLER_NODE_H
#define STEERING_CONTROLLER_NODE_H
#include <common/macros.h>

THIRD_PARTY_HEADERS_BEGIN
#include <std_msgs/Empty.h>
#include "lateral_controller_msgs/DrivingSteeringAngle.h"
THIRD_PARTY_HEADERS_END

#include "common/node_base.h"
#include "steering_controller.h"

/*!
 * \brief The SteeringController is a controller for the steering angle of
 * the car.
 */
class SteeringControllerNode : public NodeBase {
 public:
  /*!
   * \brief SteeringControllerNode the constructor.
   * \param node_handle the NodeHandle to be used.
   */
  SteeringControllerNode(ros::NodeHandle& node_handle);

  virtual ~SteeringControllerNode() override;


  /*!
   * \brief returns the name of the node. It is needed in main and onInit
   * (nodelet) method.
   * \return the name of the node
   */
  static const std::string getName();

  /*!
   * \brief onVelocityCommandReceived callback for the drive command coming from
   * the high level controller containing the desired steering angle
   * \param msg the message containing the desired steering angle value
   */
  void onSteeringCommandReceived(const lateral_controller_msgs::DrivingSteeringAngleConstPtr& msg);

  /**
   * @brief servoSetValuePublisherLoop runs in a dedicated thread and
   * publishes new debug messages to a ros topic.
   */
  void servoSetValuePublisherLoop();

  /**
   * @brief angleErrorPublisherLoop runs in a dedicated thread and
   * publishes new debug messages to a ros topic.
   */
  void angleErrorPublisherLoop();

  void autoResetCallback(const std_msgs::Empty& auto_reset);

 protected:
  /*!
   * \brief controls_rear_axle true if this controller controls the rear axle,
   * false if this controller controls the front axle. This parameter determines
   * the channel_id of the steering output and steering angle measure IPC
   */
  const bool controls_rear_axle;

  // NodeBase interface
  /*!
   * \brief startModule is called, if the node shall be turned active. In here
   * the subscribers an publishers are started.
   */
  virtual void startModule() override = 0;
  /*!
   * \brief stopModule is called, if the node shall be turned inactive. In this
   * function subscribers and publishers are shutdown.
   */
  virtual void stopModule() override = 0;

  /*!
   * \brief steering_controller contains the ROS-indipendent implementation of
   * this node.
   */
  std::unique_ptr<SteeringController> steering_controller_;

  /*!
   * \brief steering_angle_command_subscriber_ the ros subscriber for the drive
   * command with the desired steering angle
   */
  ros::Subscriber steering_angle_command_subscriber_;

  ros::Subscriber auto_reset_subscriber_;

  //! debug publishers

  /*!
   * \brief servo_set_value_publisher_ the ros publisher for the servo set
   * value. This is a debug publisher intended for e.g. visualization, the
   * controller_interface (and thus the servo) get their actual values over
   * shared memory in the steering_controller. This publisher runs in it's own
   * thread to not interfere with the actual realtime-capable controller.
   */
  ros::Publisher servo_set_value_publisher_;

  /*!
   * \brief angle_error_publisher_ the ros publisher for the angle error. This
   * is a debug publisher intended for e.g. visualization. This publisher runs
   * in it's own thread to not interfere with the actual realtime-capable
   * controller.
   */
  ros::Publisher angle_error_publisher_;

  /*!
   * \brief debug_publisher_thread_ the thread used for publishing debug
   * messages
   */
  boost::thread servo_set_value_publisher_thread_;

  /*!
   * \brief angle_error_publisher_thread_ the thread used for publishing debug
   * messages
   */
  boost::thread angle_error_publisher_thread_;
};

class SteeringControllerNodeRealtime : public SteeringControllerNode {
 public:
  SteeringControllerNodeRealtime(ros::NodeHandle& node_handle);

 protected:
  virtual void startModule() override;
  virtual void stopModule() override;
};

#endif  // STEERING_CONTROLLER_NODE_H
