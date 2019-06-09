#include "diagnostic_monitor_node.h"

#include "boost/optional.hpp"
#include "common/node_creation_makros.h"
#include "std_msgs/ColorRGBA.h"

DiagnosticMonitorNode::DiagnosticMonitorNode(ros::NodeHandle& node_handle)
    : NodeBase(node_handle), diagnostic_monitor_(&parameter_handler_) {
  led_publisher_ = node_handle_.advertise<std_msgs::ColorRGBA>("debug_light", 1, true);
  led_timer_ = node_handle_.createTimer(
      ros::Duration(0.1), &DiagnosticMonitorNode::draw, this, false, false);
  speach_timer_ = node_handle_.createTimer(
      ros::Duration(0.3), &DiagnosticMonitor::speak, &diagnostic_monitor_, false, false);
}

void DiagnosticMonitorNode::startModule() {
  diagnostics_subscriber_ = node_handle_.subscribe(
      "diagnostics", 10, &DiagnosticMonitor::onDiagnosticArray, &diagnostic_monitor_);
  emergency_stop_subscriber_ = node_handle_.subscribe<std_msgs::Bool>(
      "emergency_stop",
      1,
      boost::bind(&DiagnosticMonitor::onEmergencyStopReceived, &diagnostic_monitor_, _1, led_publisher_));
  led_timer_.start();
  speach_timer_.start();
}

void DiagnosticMonitorNode::stopModule() {
  diagnostics_subscriber_.shutdown();
  emergency_stop_subscriber_.shutdown();
  led_timer_.stop();
  speach_timer_.stop();
}

void DiagnosticMonitorNode::draw(const ros::TimerEvent& event) {
  boost::optional<std_msgs::ColorRGBA> color = diagnostic_monitor_.draw(event);
  if (color) {
    led_publisher_.publish(*color);
  }
}

const std::string DiagnosticMonitorNode::getName() {
  return std::string("diagnostic_monitor");
}

CREATE_NODE(DiagnosticMonitorNode)
