#ifndef DIAGNOSTIC_MONITOR_NODE_H
#define DIAGNOSTIC_MONITOR_NODE_H

#include "common/node_base.h"

#include "diagnostic_monitor.h"

/*!
 * \brief Emits debug messages via led, sound etc.
 */
class DiagnosticMonitorNode : public NodeBase {
 public:
  /*!
   * \brief DiagnosticMonitorNode the constructor.
   * \param node_handle the NodeHandle to be used.
   */
  DiagnosticMonitorNode(ros::NodeHandle& node_handle);

  void draw(const ros::TimerEvent& event);

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
   * \brief diagnostic_monitor contains the ROS-indipendent implementation of
   * this node.
   */
  DiagnosticMonitor diagnostic_monitor_;
  ros::Publisher led_publisher_;
  ros::Subscriber diagnostics_subscriber_;
  ros::Subscriber emergency_stop_subscriber_;
  ros::Timer led_timer_;
  ros::Timer speach_timer_;
};

#endif  // DIAGNOSTIC_MONITOR_NODE_H
