#ifndef SYSTEM_MONITOR_H
#define SYSTEM_MONITOR_H
#include <common/macros.h>

THIRD_PARTY_HEADERS_BEGIN
#include <sensor_msgs/CameraInfo.h>
#include <std_msgs/Bool.h>
THIRD_PARTY_HEADERS_END

#include <common/self_tests.h>
#include "common/parameter_interface.h"

const ParameterString<double> PARAM_CAMERA_STARTUP_TIMEOUT =
    ParameterString<double>("camera_startup_timeout");
const ParameterString<double> PARAM_CAMERA_TIMEOUT_WARNING =
    ParameterString<double>("camera_timeout_warning");
const ParameterString<double> PARAM_CAMERA_TIMEOUT_ERROR =
    ParameterString<double>("camera_timeout_error");

/*!
 * \brief Monitor checking system health and displaying results/warnings
 */
class SystemMonitor {
 public:
  /*!
  * \brief SystemMonitor is the consstructor. A ros indipendent functionality
  * containing
  * class needs a pointer to a ParameterInterface (in fact a ParameterHandler)
  * to get access to parameters.
  * \param parameters the ParameterInterface
  */
  SystemMonitor(ParameterInterface* parameters);

  void testCPU(diagnostic_updater::DiagnosticStatusWrapper& status);
  void testRAM(diagnostic_updater::DiagnosticStatusWrapper& status);
  void testStorage(diagnostic_updater::DiagnosticStatusWrapper& status);
  void testTemperature(diagnostic_updater::DiagnosticStatusWrapper& status);
  void testCamera(diagnostic_updater::DiagnosticStatusWrapper& status,
                  ros::Publisher& emergency_stop_publisher_);
  void onCameraInfo(const sensor_msgs::CameraInfo&);

 private:
  /*!
  * \brief parameters_ptr_ is needed for parameter access
  */
  ParameterInterface* parameters_ptr_;

  int last_total_jiffies_;
  int last_work_jiffies_;
  // ros::Time last_test_time_;
  ros::Time last_camera_info_;
  const ros::Time startup_time_;
  bool is_in_emergency_stop_;
};

#endif  // SYSTEM_MONITOR_H
