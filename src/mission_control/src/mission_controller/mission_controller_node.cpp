#include "mission_controller_node.h"
#include <common/macros.h>

THIRD_PARTY_HEADERS_BEGIN
//#include "mission_control/StartParkingDiscipline.h"
THIRD_PARTY_HEADERS_END

#include "common/node_creation_makros.h"
#include "common/contains.h"
#include "common/container.h"

const ParameterString<std::vector<std::string>> MissionControllerNode::ALL_MODULES(
    "all_modules");
const ParameterString<std::vector<std::string>> MissionControllerNode::PERSISTENT_MODULES(
    "persistent_modules");
const ParameterString<std::vector<std::string>> MissionControllerNode::ACTIVE_MODULES(
    "active_modules");
const ParameterString<double> MissionControllerNode::SERVICE_CALL_TIMEOUT(
    "service_call_timeout");

using namespace common;

void MissionControllerNode::tryCall(ros::ServiceClient& client,
                                    const double service_call_timeout,
                                    bool activate) {
  common_msgs::ActivationService activation_service;
  activation_service.request.moduleActive = activate;

  if (!client.waitForExistence(ros::Duration(service_call_timeout))) {
    ROS_FATAL_STREAM("module " << client.getService() << " is not responding!");
  }

  client.call(activation_service);
}

void MissionControllerNode::tryActivate(ros::ServiceClient& client,
                                        const double service_call_timeout) {
  tryCall(client, service_call_timeout, true);
}

void MissionControllerNode::tryDeactivate(ros::ServiceClient& client,
                                          const double service_call_timeout) {
  tryCall(client, service_call_timeout, false);
}

MissionControllerNode::MissionControllerNode(ros::NodeHandle& node_handle)
    : NodeBase(node_handle) {
  mission_mode_.header.stamp = ros::Time();
  mission_mode_.mission_mode = common_msgs::MissionMode::IDLE;
  parameter_handler_.registerParam(ALL_MODULES);
  parameter_handler_.registerParam(PERSISTENT_MODULES);
  parameter_handler_.registerParam(ACTIVE_MODULES);
  parameter_handler_.registerParam(SERVICE_CALL_TIMEOUT);

  all_modules_ = parameter_handler_.getParam(ALL_MODULES);
  persistent_modules_ = toSet(parameter_handler_.getParam(PERSISTENT_MODULES));

  mission_mode_subscriber_ = node_handle_.subscribe(
      "/mission_mode", 1, &MissionControllerNode::missionModeCallback, this);

  //! mission mode shall be latched
  mission_mode_publisher_ =
      node_handle_.advertise<common_msgs::MissionMode>("/mission_mode", 1, true);

  //! latch idle mode at startup
  mission_mode_publisher_.publish(mission_mode_);

  //! initialize a service for each module
  for (const std::string& module : all_modules_) {
    module_clients_.emplace(module,
                            node_handle_.serviceClient<common_msgs::ActivationService>(
                                module + "/activate_module"));
  }

  //! give the modules some time to startup
  ros::Duration(5).sleep();

  //! activate persistent modules
  const double service_call_timeout = parameter_handler_.getParam(SERVICE_CALL_TIMEOUT);

  ROS_INFO_STREAM("starting persistent modules: " << toString(persistent_modules_));

  for (const std::string& module : persistent_modules_) {
    tryActivate(module_clients_.find(module)->second, service_call_timeout);
  }
}

void MissionControllerNode::startModule() {
  ROS_WARN("you cannot start the mission_controller explicitely!");
}

void MissionControllerNode::stopModule() {
  ROS_WARN("you cannot stop the mission_controller explicitely!");
}

const std::string MissionControllerNode::getName() {
  return std::string("mission_controller");
}

void MissionControllerNode::missionModeCallback(const common_msgs::MissionModeConstPtr& mission_mode) {

  if (mission_mode->header.stamp <= mission_mode_.header.stamp) {
    ROS_DEBUG("missionModeCallback: duplication through latch");
    return;
  }

  ROS_INFO("missionModeCallback");

  // We have nothing to do, if we are in IDLE Mode and want to change into IDLE
  // Mode
  if (mission_mode->mission_mode == common_msgs::MissionMode::IDLE &&
      mission_mode_.mission_mode == common_msgs::MissionMode::IDLE) {
    return;
  }

  if (mission_mode->mission_mode == mission_mode_.mission_mode) {
    // if old and new mission mode are equal, a mode button has been pressed
    // two times, so the user wants to change into idle mode. We have to
    // notify the whole system, so we need to publish a MissionMode message
    // with IDLE-Mode in it. Also we need to load the default parameters of
    // this node, so we can start the nodes, which should be active in
    // IDLE mode.
    mission_mode_.mission_mode = common_msgs::MissionMode::IDLE;
    mission_mode_.header = mission_mode->header;
    parameter_handler_.changeNamespace("default");
  } else {  // The usual case...
    mission_mode_ = *mission_mode;
  }
  // republish to latch this message
  // Will cause missionModeCallback!
  mission_mode_publisher_.publish(mission_mode_);

  deactivateAllModules();
  activateSelectedModules();

  if (mission_mode_.mission_mode == common_msgs::MissionMode::PARKING) {
    //    mission_control::StartParkingDiscipline parking_service;
    //    ros::ServiceClient parking_client =
    //        node_handle_.serviceClient<mission_control::StartParkingDiscipline>(
    //            "/mission_control/parking_mc/start_parking_discipline");
    //    ROS_INFO("Parking mode activated. Waiting for parking service.");
    //    parking_client.waitForExistence();
    //    ROS_INFO("Service found. Calling it. This might take a while.");
    //    parking_client.call(parking_service);
    //    ROS_INFO("Parking finished. Return to idle mode.");
    //    common_msgs::MissionMode idle_msg;
    //    idle_msg.header.stamp = ros::Time::now();
    //    idle_msg.mission_mode = common_msgs::MissionMode::IDLE;
    //    mission_mode_publisher_.publish(idle_msg);
  }
}

void MissionControllerNode::deactivateAllModules() {
  ROS_INFO("deactivateAllModules");

  const double service_call_timeout = parameter_handler_.getParam(SERVICE_CALL_TIMEOUT);

#pragma omp parallel
  for (const auto& module_pair : module_clients_.get<fifo>()) {
#pragma omp single nowait
    if (common::contains(persistent_modules_, module_pair.first)) {
      ROS_DEBUG_STREAM("skipping persistent module " << module_pair.first);
    } else {
      tryDeactivate(module_pair.second, service_call_timeout);
    }
  }
}

void MissionControllerNode::activateSelectedModules() {
  ROS_INFO_STREAM("activateSelectedModules: "
                  << toString(parameter_handler_.getParam(ACTIVE_MODULES)));

  std::vector<std::string> selected_modules = parameter_handler_.getParam(ACTIVE_MODULES);

  ROS_DEBUG_STREAM("found " << selected_modules.size()
                            << " modules to activate");

  const double service_call_timeout = parameter_handler_.getParam(SERVICE_CALL_TIMEOUT);

#pragma omp parallel
  for (const std::string& module : selected_modules) {
#pragma omp single nowait
    if (common::contains(persistent_modules_, module)) {
      ROS_DEBUG_STREAM("skipping persistent module " << module);
    } else {
      const auto m = module_clients_.find(module);
      if (m == module_clients_.end()) {
        ROS_ERROR_STREAM(
            "module " << module << " not found in vector of module clients!");
      } else {
        tryActivate(m->second, service_call_timeout);
      }
    }
  }
}

CREATE_NODE(MissionControllerNode)
