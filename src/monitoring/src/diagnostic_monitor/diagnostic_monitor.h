#ifndef DIAGNOSTIC_MONITOR_H
#define DIAGNOSTIC_MONITOR_H

#include "common/macros.h"
#include "common/parameter_interface.h"

THIRD_PARTY_HEADERS_BEGIN
#include <queue>
#include <vector>
#include "boost/optional.hpp"
#include "diagnostic_msgs/DiagnosticArray.h"
#include "diagnostic_msgs/DiagnosticStatus.h"
#include "ros/ros.h"
#include "std_msgs/Bool.h"
#include "std_msgs/ColorRGBA.h"
THIRD_PARTY_HEADERS_END

using OptionalDiagnosticStatus = boost::optional<diagnostic_msgs::DiagnosticStatus>;
/*!
 * \brief Emits debug messages via led, sound etc.
 */
class DiagnosticMonitor {
 public:
  /*!
   * \brief DiagnosticMonitor is the consstructor. A ros indipendent
   * functionality containing
   * class needs a pointer to a ParameterInterface (in fact a ParameterHandler)
   * to get access to parameters.
   * \param parameters the ParameterInterface
   */
  DiagnosticMonitor(ParameterInterface *parameters);

  /*!
   * \brief receive new status message
   */
  void onDiagnosticArray(const diagnostic_msgs::DiagnosticArray array);

  void onEmergencyStopReceived(const std_msgs::Bool::ConstPtr &msg,
                               ros::Publisher &led_publisher_);

  boost::optional<std_msgs::ColorRGBA> draw(const ros::TimerEvent &event);

  static bool equal(const OptionalDiagnosticStatus &first,
                    const OptionalDiagnosticStatus &second);

  void speak(const ros::TimerEvent &event);

 private:
  /*!
   * \brief parameters_ptr_ is needed for parameter access
   */
  ParameterInterface *parameters_ptr_;
  OptionalDiagnosticStatus led;
  OptionalDiagnosticStatus voice;
  OptionalDiagnosticStatus old_voice;
  ros::Time old_voice_stamp;
  bool espeak_available_;
};

#endif  // DIAGNOSTIC_MONITOR_H
