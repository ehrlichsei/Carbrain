#include "common/node_base.h"

namespace common {
namespace node_base {

NodeBase::NodeBase(const ros::NodeHandle &node_handle)
    : node_handle_(node_handle), parameter_handler_(node_handle), module_active_(false) {
  activate_module_service_ = node_handle_.advertiseService(
      "activate_module", &NodeBase::activateModuleCallback, this);
  mission_mode_subscriber_ =
      node_handle_.subscribe("/mission_mode", 1, &NodeBase::missionModeCallback, this);
  node_handle_.param<bool>("start_activated", module_active_, false);
}

int NodeBase::loop() {
  ros::spin();
  return 0;
}

bool NodeBase::isModuleActive() { return module_active_; }

void NodeBase::activateIfDesired() {
  if (isModuleActive()) {
    startModule();
    ROS_INFO("Started activated.");
  }
}

const std::string NodeBase::getName() { return std::string("not_named"); }

bool NodeBase::activateModuleCallback(common_msgs::ActivationService::Request &req,
                                      UNUSED common_msgs::ActivationService::Response &res) {
  if (module_active_ && !req.moduleActive) {
    ROS_INFO("stopping module");
    stopModule();
    module_active_ = false;
  } else if (!module_active_ && req.moduleActive) {
    ROS_INFO("starting module");
    startModule();
    module_active_ = true;
  }
  return true;
}

void NodeBase::missionModeCallback(const common_msgs::MissionModeConstPtr &mission_mode) {
  if (mission_mode->header.stamp <= last_mission_mode_.header.stamp &&
      mission_mode->mission_mode == last_mission_mode_.mission_mode) {
    return;
  }

  last_mission_mode_ = *mission_mode;

  switch (mission_mode->mission_mode) {
    case common_msgs::MissionMode::FREE_RIDE:
      parameter_handler_.changeNamespace("free_ride_mode");
      break;
    case common_msgs::MissionMode::OBSTACLE:
      parameter_handler_.changeNamespace("obstacle_mode");
      break;
    case common_msgs::MissionMode::PARKING:
      parameter_handler_.changeNamespace("parking_mode");
      break;
    case common_msgs::MissionMode::IDLE:
    default:
      parameter_handler_.changeNamespace("default");
      break;
  }
}

} // namespace node_base
} // namespace common
