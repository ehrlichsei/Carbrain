#ifndef NODEBASE_H
#define NODEBASE_H

#include <common/macros.h>

THIRD_PARTY_HEADERS_BEGIN
#include <ros/ros.h>
#include <string>

#include "common_msgs/MissionMode.h"
#include "common_msgs/ActivationService.h"
THIRD_PARTY_HEADERS_END

#include "parameter_handler.h"

namespace common {
/*!
 * \namespace common::node_base
 * \brief node_base contains everything closely related to NodeBase such as
 * ParameterInterface and ParameterHandler.
 *
 */
namespace node_base {
/*!
 * \brief The NodeBase class is meant to be the basis of every node in this
 * project. In this class commonly used basic features are implemented such as
 * parameter handling and dis- and enabling of the node on runtime
 */
class NodeBase {

 protected:
  /*!
   * \brief NodeBase the constructor meant to be used in the context of
   * nodelets, because the NodeHandle has to obtained otherwise in the context
   * of nodelets.
   * \param node_handle the NodeHandle to be used.
   */
  NodeBase(const ros::NodeHandle& node_handle);

  /*!
   * \brief node_handle_ needed for interacting with the ros framework. It is
   * intended to be a private node handle.
   */
  ros::NodeHandle node_handle_;

 public:
  virtual ~NodeBase() = default;
  /*!
   * \brief loop contains the loop (spin, spinOnce or others) of the node. Will
   * not work with nodelets! The default implementation is spin.
   * \return the return value if the node exits.
   */
  virtual int loop();

  /*!
   * \brief isModuleActive returns whether the node is active or not.
   * \return whether the node is active or not.
   */
  bool isModuleActive();

  /*!
   * \brief activateIfDesired checks if activation on start up is desired and
   * start module if desired.
   * This function should be used in the constructor of the node after all
   * initialization.
   */
  void activateIfDesired();

  /*!
   * \brief getName returns the name of the node. This is the fallback
   * implementation if the node itself
   * does not. NodeletBase and the node creation makros rely this function so
   * this fallback is needed.
   * \return returns the name of the node.
   */
  static const std::string getName();

  /*!
   * \brief parameter_handler_ provides parameter access.
   */
  ParameterHandler parameter_handler_;

 private:
  /*!
   * \brief startModule is called, if the node shall be turned active. In this
   * function subscribers and publishers should be started.
   */
  virtual void startModule() = 0;
  /*!
   * \brief stopModule is called, if the node shall be turned inactive. In this
   * function subscribers and publishers should be shut down.
   */
  virtual void stopModule() = 0;

  /*!
   * \brief activateModuleCallback is the callback function which is called if
   * the activation service is called. This function calls startModule or
   * stopModule if its needed.
   * \param req the request of the service
   * \param res the response of the service
   * \return if the service call could be satisfied
   */
  bool activateModuleCallback(common_msgs::ActivationService::Request& req,
                              common_msgs::ActivationService::Response& res);

  /*!
   * \brief missionModeCallback is the callback function which is called if a
   * mission mode message arrives. In this function the change of the mission
   * mode gets executed.
   * \param mission_mode the mission mode message
   */
  void missionModeCallback(const common_msgs::MissionModeConstPtr& mission_mode);

  /*!
   * \brief module_active_ determines if the node is active or not.
   */
  bool module_active_;

  /*!
   * \brief activate_module_service_ to receive activation calls.
   */
  ros::ServiceServer activate_module_service_;
  /*!
   * \brief mission_mode_subscriber_ to receive mission mode messages
   */
  ros::Subscriber mission_mode_subscriber_;

  common_msgs::MissionMode last_mission_mode_;
};

inline ros::NodeHandle getParent(const ros::NodeHandle& child) {
  return ros::NodeHandle(ros::names::parentNamespace(child.getNamespace()));
}

inline ros::TransportHints noDelayTransport() {
  return ros::TransportHints().tcpNoDelay();
}
}  // namespace node_base;
using namespace node_base;
}  // namespace common;
using common::NodeBase;

#endif  // NODEBASE_H
