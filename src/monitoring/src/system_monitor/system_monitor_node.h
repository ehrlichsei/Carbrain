#ifndef SYSTEM_MONITOR_NODE_H
#define SYSTEM_MONITOR_NODE_H

#include "common/node_base.h"

#include "system_monitor.h"

/*!
 * \brief Monitor checking system health and displaying results/warnings
 */
class SystemMonitorNode : public NodeBase {
 public:
  /*!
   * \brief SystemMonitorNode the constructor.
   * \param node_handle the NodeHandle to be used.
   */
  SystemMonitorNode(ros::NodeHandle& node_handle);
  /*!
   * \brief returns the name of the node. It is needed in main and onInit
   * (nodelet) method.
   * \return the name of the node
   */
  static const std::string getName();

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

  /*!
   * \brief system_monitor contains the ROS-indipendent implementation of this
   * node.
   */

  SystemMonitor system_monitor_;
  common::SelfTests self_tests_;
  common::SelfTests self_tests_fast_;
  ros::Subscriber camera_info_subscriber_;
  ros::Publisher emergency_stop_publisher_;
};

#endif  // SYSTEM_MONITOR_NODE_H
