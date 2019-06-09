#ifndef DIAGNOSTICS_INTERFACE_H
#define DIAGNOSTICS_INTERFACE_H

#include <common/macros.h>

THIRD_PARTY_HEADERS_BEGIN
#include <ros/ros.h>
#include <diagnostic_msgs/DiagnosticStatus.h>
#include <diagnostic_msgs/DiagnosticArray.h>
THIRD_PARTY_HEADERS_END

#include <common/diagnostic_status_wrapper.h>

class IDiagnosticsInterface {
public:
  virtual ~IDiagnosticsInterface() = default;
  virtual void speak(const unsigned char lvl, const std::string msg) = 0;
};

class DiagnosticsInterface : public IDiagnosticsInterface {
public:
  DiagnosticsInterface(ros::NodeHandle node_handle);
  void speak(const unsigned char lvl, const std::string msg) override;

private:
  void publish(diagnostic_msgs::DiagnosticStatus& wrapper_msg);

  ros::Publisher diagnostics_publisher;
};

#endif
