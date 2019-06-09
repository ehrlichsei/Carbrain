#include "diagnostics_interface.h"

DiagnosticsInterface::DiagnosticsInterface(ros::NodeHandle node_handle) {
  diagnostics_publisher = node_handle.advertise<diagnostic_msgs::DiagnosticArray>("/diagnostics", 1);
}

void DiagnosticsInterface::speak(const unsigned char lvl, const std::string msg) {
  common::DiagnosticStatusWrapper status;
  status.addVoice();
  status.summary(lvl, msg);
  status.hardware_id = "none";
  status.name = "Pavlov";
  publish(status);
}

void DiagnosticsInterface::publish(diagnostic_msgs::DiagnosticStatus& wrapper_msg) {
  diagnostic_msgs::DiagnosticArray array_msg;
  array_msg.status.push_back(wrapper_msg);
  array_msg.header.stamp = ros::Time::now();
  diagnostics_publisher.publish(array_msg);
}
