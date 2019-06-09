#ifndef SPEED_CONTROLLER_NODE_DEBUG_H
#define SPEED_CONTROLLER_NODE_DEBUG_H

#include "speed_controller_node.h"
#include <common/macros.h>

THIRD_PARTY_HEADERS_BEGIN
#include <std_msgs/Bool.h>
#include <std_srvs/Empty.h>
#include <state_estimation_msgs/State.h>
THIRD_PARTY_HEADERS_END

#include "longitudinal_controller_msgs/DrivingSpeed.h"
#include "common/node_base.h"
#include "speed_controller.h"

/*!
 * \brief Node that controls the speed of the car.
 * In debug-mode SpeedControllerNodeDebug will be started instead of
 * SpeedControllerNode.
 * SpeedControllerNodeDebug accepts state measurements by ros topic (while
 * SpeedControllerNode does not), but launches
 * SpeedControllerDebug instad of SpeedControllerRealtime, which doesn't allow
 * communication by shared memory.
 * This allows debugging of the speed controll unit with rosbags, but doesn't
 * give any real-time-guarantees.
 */
class SpeedControllerNodeDebug : public SpeedControllerNode {
 public:
  SpeedControllerNodeDebug(ros::NodeHandle &node_handle);

 private:
  // NodeBase interface
  /*!
   * \brief startModule is called, if the node shall be turned active. In here
   * the subscribers an publishers are started.
   */
  virtual void startModule() override;
  /*!
   * \brief stopModule is called, if the node shall be turned inactive. In this
   * function subscribers and publishers are shutdown.
   */
  virtual void stopModule() override;

  /*!
   * \brief handleStateMeasure callback of the state measure topic
   * \param msg the message.
   */
  void handleState(const state_estimation_msgs::State &msg);

  /*!
   * \brief time_of_last_measurement needed for to check for time-jumps in the
   * messages (due to a restarting rosbag or missing messages)
   */
  ros::Time time_of_last_measurement;

  /*!
   * \brief state_estimation_subscriber_ the ros subscriber for the
   * state estimation topic
   */
  ros::Subscriber state_estimation_subscriber_;
};

#endif  // SPEED_CONTROLLER_NODE_DEBUG_H
