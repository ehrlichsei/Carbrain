#include "look_at_node_debug.h"
#include "servers/obstacles_server_debug.h"
#include "servers/pedestrian_server_debug.h"

THIRD_PARTY_HEADERS_BEGIN
#include <boost/range/algorithm_ext/erase.hpp>
THIRD_PARTY_HEADERS_END

namespace look_at {
LookAtNodeDebug::LookAtNodeDebug(ros::NodeHandle &node_handle)
    : LookAtNode(node_handle) {
  // add debug servers here
  servers_.push_back(std::make_unique<ObstaclesServerDebug>(
      node_handle_, &parameter_handler_, "look_for_obstacles"));
  servers_.push_back(std::make_unique<PedestrianServerDebug>(
      node_handle_, &parameter_handler_, "look_for_pedestrians"));

  // this has to be called in order to avoid server duplication
  removeNonDebugServers();
  ROS_DEBUG("number of servers is %zu", servers_.size());
}

void LookAtNodeDebug::startModule() { LookAtNode::startModule(); }

void LookAtNodeDebug::stopModule() { LookAtNode::stopModule(); }

void LookAtNodeDebug::removeNonDebugServers() {
  boost::remove_erase_if(servers_, [](const auto &s) { return !s->isDebug(); });
}

}  // namespace look_at
