#ifndef MISSION_CONTROLLER_NODE_H
#define MISSION_CONTROLLER_NODE_H

#include <set>

#include "common/node_base.h"
#include "common/fifo_map.h"


/*!
 * \brief The MissionController (de-)activates all necessary modules depending
 *        on the chosen discipline.
 *
 * The MissionController is the only module that is bypassing the NodeBase
 * interface of start-/stopModule in order to be active right from
 * initialization and keep on managing the other modules.
 *
 * Other modules can also be configured to be persistent in order to being
 * activated on system startup and not being reactivated on mission mode
 * switches.
 */
class MissionControllerNode : public NodeBase {
  typedef common::fifo_map<std::string, ros::ServiceClient> ModuleMap;

 public:
  /*!
   * \brief MissionControllerNode the constructor.
   * \param node_handle the NodeHandle to be used.
   */
  MissionControllerNode(ros::NodeHandle& node_handle);
  /*!
   * \brief returns the name of the node. It is needed in main and onInit
   * (nodelet) method.
   * \return the name of the node
   */
  static const std::string getName();

 private:
  static const ParameterString<std::vector<std::string>> ALL_MODULES;
  static const ParameterString<std::vector<std::string>> PERSISTENT_MODULES;
  static const ParameterString<std::vector<std::string>> ACTIVE_MODULES;
  static const ParameterString<double> SERVICE_CALL_TIMEOUT;

  // NodeBase interface
  void startModule() override;
  void stopModule() override;

  /*!
   * \brief missionModeCallback performs modules deactivation/deactivation.
   * Usually first every known module is deactivated by calling
   * deactivateAllModules"()". Then the desired modules are activated by calling
   * activateSelectedModules"()".
   *
   * Every received message will be republished and latched. This makes it
   * possible to get the current mission mode by subscribing on '/mission_mode'
   * at any point in time.
   *
   * Special cases are:
   *  - if two messages with the same mission mode in it have arrived
   *    consecutively, idle mode is turned on by publishing a message. This
   *    makes it possible to push a mode button twice to switch in idle mode.
   *  - if a parking mode message is recieved, other serivce calls are called,
   *    too.
   * \param mission_mode the new mission mode.
   */
  void missionModeCallback(const common_msgs::MissionModeConstPtr& mission_mode);

  /*!
   * \brief deactivateAllModules deactivates all known modules (given in
   * '~/all_modules') except the persistent ones (given in
   * '~/persistent_modules').
   */
  void deactivateAllModules();
  /*!
   * \brief activateSelectedModules activates the modules to be activated (given
   * in '~/active_modules'). Persistent modules (given in
   * '~/persistent_modules') are skipped.
   */
  void activateSelectedModules();

  /*!
   * \brief tryCall tries to call the activation-service of a module. If it
   * fails, a error message is printed.
   * \param client the module the service shall be called on.
   * \param service_call_timeout the time to wait for the module.
   * \param activate wheather to activate or to deactivate the module.
   */
  static void tryCall(ros::ServiceClient& client, const double service_call_timeout, bool activate);

  /*!
   * \brief tryActivate tries to activate a module. If it fails, an error
   * message is printed.
   * \param client the module to activate.
   * \param service_call_timeout the time to wait for the module.
   */
  static void tryActivate(ros::ServiceClient& client, const double service_call_timeout);

  /*!
   * \brief tryDeactivate tries to deactivate a module. If it fails, an error
   * message is printed.
   * \param client the module to deactive.
   * \param service_call_timeout the timeo to wait for the module.
   */
  static void tryDeactivate(ros::ServiceClient& client, const double service_call_timeout);

  ros::Subscriber mission_mode_subscriber_;
  ros::Publisher mission_mode_publisher_;

  std::vector<std::string> all_modules_;
  std::set<std::string> persistent_modules_;
  ModuleMap module_clients_;
  common_msgs::MissionMode mission_mode_;
};


#endif  // MISSION_CONTROLLER_NODE_H
