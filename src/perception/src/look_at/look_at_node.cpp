#include "look_at_node.h"

#include "common/node_creation_makros.h"
#include "servers/obstacles_server.h"
#include "servers/pedestrian_server.h"
#include "servers/server_interface.h"

#include FANCY_DEBUG_INCLUDE("debug/look_at_node_debug.h")

namespace look_at {

LookAtNode::LookAtNode(ros::NodeHandle &node_handle) : NodeBase(node_handle) {
  // Abb initializations of class member here. This function is meant to be
  // called at construction time an shall be called ONLY in the constructors.

  // push_back all the servers to the servers_ vector
  servers_.push_back(std::make_unique<ObstaclesServer>(
      node_handle_, &parameter_handler_, "look_for_obstacles"));
  servers_.push_back(std::make_unique<PedestrianServer>(
      node_handle_, &parameter_handler_, "look_for_pedestrians"));
}

void LookAtNode::startModule() {
  // sets your node in running mode. Activate publishers, subscribers, service
  // servers, etc here.
  for (auto &server : servers_) {
    server->advertise();
  }
}

void LookAtNode::stopModule() {
  // sets your node in idle mode. Deactivate publishers, subscribers, service
  // servers, etc here.
  for (auto &server : servers_) {
    server->shutdown();
  }
}

const std::string LookAtNode::getName() { return std::string("look_at"); }
}  // namespace look_at

CREATE_NODE_WITH_FANCY_DEBUG(look_at::LookAtNode, look_at::LookAtNodeDebug)
