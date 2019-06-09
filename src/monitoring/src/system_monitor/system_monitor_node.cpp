#include "system_monitor_node.h"

#include "common/node_creation_makros.h"

SystemMonitorNode::SystemMonitorNode(ros::NodeHandle &node_handle)
    : NodeBase(node_handle),
      system_monitor_(&parameter_handler_),
      self_tests_fast_(ros::Duration(0.016)) {
}

void SystemMonitorNode::startModule() {
  // sets your node in running mode. Activate publishers, subscribers, service
  // servers, etc here.

  camera_info_subscriber_ = node_handle_.subscribe(
      "camera_info", 1, &SystemMonitor::onCameraInfo, &system_monitor_);
  emergency_stop_publisher_ =
      node_handle_.advertise<std_msgs::Bool>("emergency_stop", 1);

  self_tests_.add("CPU Load", boost::bind(&SystemMonitor::testCPU, &system_monitor_, _1));
  self_tests_.add("RAM Utilisation",
                  boost::bind(&SystemMonitor::testRAM, &system_monitor_, _1));
  self_tests_.add("Data Storage Utilisation",
                  boost::bind(&SystemMonitor::testStorage, &system_monitor_, _1));
  self_tests_.add("Temperature",
                  boost::bind(&SystemMonitor::testTemperature, &system_monitor_, _1));
  self_tests_fast_.add(
      "Camera", boost::bind(&SystemMonitor::testCamera, &system_monitor_, _1, emergency_stop_publisher_));
}

void SystemMonitorNode::stopModule() {
  // sets your node in idle mode. Deactivate publishers, subscribers, service
  // servers, etc here.

  self_tests_.removeByName("CPU Load");
  self_tests_.removeByName("RAM Utilisation");
  self_tests_.removeByName("Data Storage Utilisation");
  self_tests_.removeByName("Temperature");
  camera_info_subscriber_.shutdown();
}

const std::string SystemMonitorNode::getName() {
  return std::string("system_monitor");
}

CREATE_NODE(SystemMonitorNode)
