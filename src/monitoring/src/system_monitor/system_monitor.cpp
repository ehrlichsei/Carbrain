#include <common/macros.h>

THIRD_PARTY_HEADERS_BEGIN
#include <boost/algorithm/string.hpp>
#include <boost/bind.hpp>
#include <vector>
#include <cstdlib>
#include <iostream>
#include <fstream>
#include <sys/statvfs.h>
THIRD_PARTY_HEADERS_END

#include "system_monitor.h"

SystemMonitor::SystemMonitor(ParameterInterface* parameters)
    : parameters_ptr_(parameters), startup_time_(ros::Time::now()) {
  parameters_ptr_->registerParam(PARAM_CAMERA_STARTUP_TIMEOUT);
  parameters_ptr_->registerParam(PARAM_CAMERA_TIMEOUT_WARNING);
  parameters_ptr_->registerParam(PARAM_CAMERA_TIMEOUT_ERROR);
  is_in_emergency_stop_ = false;
  last_camera_info_ = ros::Time::now();
  // Initialise CPU jiffies
  std::ifstream cpu_file;
  cpu_file.open("/proc/stat", std::ifstream::in);

  if (!cpu_file.good()) {
    ros::requestShutdown();
    return;
  }

  char cpu_file_content[256];
  cpu_file.getline(cpu_file_content, 256);

  std::vector<std::string> cpu_content_strs;
  boost::split(cpu_content_strs, cpu_file_content, boost::is_any_of(" "));

  last_total_jiffies_ = std::atoi(cpu_content_strs[2].c_str()) +
                        std::atoi(cpu_content_strs[3].c_str()) +
                        std::atoi(cpu_content_strs[4].c_str()) +
                        std::atoi(cpu_content_strs[5].c_str()) +
                        std::atoi(cpu_content_strs[6].c_str()) +
                        std::atoi(cpu_content_strs[7].c_str()) +
                        std::atoi(cpu_content_strs[8].c_str());
  last_work_jiffies_ = std::atoi(cpu_content_strs[2].c_str()) +
                       std::atoi(cpu_content_strs[3].c_str()) +
                       std::atoi(cpu_content_strs[4].c_str());
}

// CPU Load Test
void SystemMonitor::testCPU(diagnostic_updater::DiagnosticStatusWrapper& status) {
  // ros::Time current_time = ros::Time::now();
  // ros::Duration time_since_last_check = current_time - last_test_time_;
  // time_since_last_check.toNSec();

  // Read memory mapped file "/proc/stat" and save in the string
  // "CPU_file_content"
  std::ifstream cpu_file;
  cpu_file.open("/proc/stat", std::ifstream::in);

  if (!cpu_file.good()) {
    status.summary(diagnostic_msgs::DiagnosticStatus::WARN,
                   "Could not pass /proc/stat");
    return;
  }

  char cpu_file_content[256];
  cpu_file.getline(cpu_file_content, 256);

  // Split "CPU_file_content" and save every part in the vector
  // "CPU_content_strs"
  std::vector<std::string> cpu_content_strs;
  boost::split(cpu_content_strs, cpu_file_content, boost::is_any_of(" "));

  // Calculate "total_jiffies" and "work_jiffies"
  int total_jiffies_1 = std::atoi(cpu_content_strs[2].c_str()) +
                        std::atoi(cpu_content_strs[3].c_str()) +
                        std::atoi(cpu_content_strs[4].c_str()) +
                        std::atoi(cpu_content_strs[5].c_str()) +
                        std::atoi(cpu_content_strs[6].c_str()) +
                        std::atoi(cpu_content_strs[7].c_str()) +
                        std::atoi(cpu_content_strs[8].c_str());
  int work_jiffies_1 = std::atoi(cpu_content_strs[2].c_str()) +
                       std::atoi(cpu_content_strs[3].c_str()) +
                       std::atoi(cpu_content_strs[4].c_str());

  double cpu_load = static_cast<double>(work_jiffies_1 - last_work_jiffies_) /
                    static_cast<double>(total_jiffies_1 - last_total_jiffies_);

  last_total_jiffies_ = total_jiffies_1;
  last_work_jiffies_ = work_jiffies_1;

  status.addf("CPU load", "%.2f %%", cpu_load * 100);
  // CPU case distinction for the "SelfTests" class
  if (cpu_load <= 0.8) {
    status.summary(diagnostic_msgs::DiagnosticStatus::OK, "CPU Load OK");
  } else if (cpu_load > 0.99) {
    status.summary(diagnostic_msgs::DiagnosticStatus::ERROR,
                   "CPU Load critical");
  } else {
    status.summary(diagnostic_msgs::DiagnosticStatus::WARN, "CPU Load high");
  }
}

// RAM Utilisation Test
void SystemMonitor::testRAM(diagnostic_updater::DiagnosticStatusWrapper& status) {
  // Read memory mapped file "/proc/meminfo" and save in the string
  // "Mem_file_content"
  std::ifstream mem_file;
  mem_file.open("/proc/meminfo", std::ifstream::in);

  if (!mem_file.good()) {

    return;
  }

  char mem_file_content[256];
  mem_file.getline(mem_file_content, 256);

  // Split "Mem_file_content" and save every part in the vector
  // "Mem_content_strs"
  std::vector<std::string> mem_content_strs_1;
  boost::algorithm::split(
      mem_content_strs_1, mem_file_content, boost::is_any_of("\t "), boost::token_compress_on);

  if (mem_content_strs_1.size() < 2) {
    status.summary(diagnostic_msgs::DiagnosticStatus::WARN,
                   "Could not pass /proc/meminfo");
    return;
  }

  mem_file.getline(mem_file_content, 256);

  // Split "Mem_file_content" and save every part in the vector
  // "Mem_content_strs"
  std::vector<std::string> mem_content_strs_2;
  boost::algorithm::split(
      mem_content_strs_2, mem_file_content, boost::is_any_of("\t "), boost::token_compress_on);

  if (mem_content_strs_2.size() < 2) {
    status.summary(diagnostic_msgs::DiagnosticStatus::WARN,
                   "Could not pass /proc/meminfo");
    return;
  }

  // Calculate RAM Utilisation
  unsigned int ram_total = std::atoi(mem_content_strs_1[1].c_str());
  unsigned int ram_used = std::atoi(mem_content_strs_1[1].c_str()) -
                          std::atoi(mem_content_strs_2[1].c_str());
  double ram_used_rel = static_cast<double>(ram_used) / static_cast<double>(ram_total);

  status.addf(
      "RAM Utilisation", "%.2f %% %u MB/%u MB", ram_used_rel * 100, ram_used / 1024, ram_total / 1024);
  // RAM case distinction for the "SelfTests" class
  if (ram_used_rel <= 0.8) {
    status.summary(diagnostic_msgs::DiagnosticStatus::OK, "RAM Utilisation OK");
  } else if (ram_used_rel > 0.99) {
    status.summary(diagnostic_msgs::DiagnosticStatus::ERROR,
                   "RAM Utilisation critical");
  } else {
    status.summary(diagnostic_msgs::DiagnosticStatus::WARN,
                   "RAM Utilisation high");
  }
}

// Data Storage Utilisation Test
void SystemMonitor::testStorage(diagnostic_updater::DiagnosticStatusWrapper& status) {
  // Use statvfs to read data storage information
  struct statvfs stats;  // NOLINT
  statvfs("/", &stats);

  // Calculate data storage capacity in bytes
  uint64_t free_storage = stats.f_bsize * stats.f_bfree;
  uint64_t total_storage = stats.f_blocks * stats.f_frsize;
  uint64_t storage_used = total_storage - free_storage;
  double storage_used_rel =
      static_cast<double>(storage_used) / static_cast<double>(total_storage);

  status.addf("Used Capacity",
              "%.2f %% %u MB/%u MB",
              storage_used_rel * 100,
              storage_used / 1048576,
              total_storage / 1048576);
  // Data Storage case distinction for the "SelfTests" class
  if (storage_used_rel <= 0.8) {
    status.summary(diagnostic_msgs::DiagnosticStatus::OK,
                   "Enough Data Storage Space");
  } else if (storage_used_rel > 0.99) {
    status.summary(diagnostic_msgs::DiagnosticStatus::ERROR,
                   "Data Storage full");
  } else {
    status.summary(diagnostic_msgs::DiagnosticStatus::WARN,
                   "Data Storage almost full");
  }
}

// Temperatures
// Stream Temperatures
void SystemMonitor::testTemperature(diagnostic_updater::DiagnosticStatusWrapper& status) {
  int file_ok = 1;
  int i = 2;
  int n_cores = 0;
  float temperature_sum = 0.;
  while (file_ok == 1) {
    std::stringstream temperature_file_name;
    temperature_file_name
        << "/sys/bus/platform/devices/coretemp.0/hwmon/hwmon1/temp" << i
        << "_input";
    std::ifstream temperatures;
    temperatures.open(temperature_file_name.str().c_str(), std::ifstream::in);
    if (temperatures.good()) {
      char temperature_str[256];
      temperatures.get(temperature_str, 256);
      temperature_sum = temperature_sum + atoi(temperature_str);
      i++;
    } else {
      file_ok = 0;
      n_cores = i - 2;
    }
  }
  const float temperature_avg = temperature_sum / (n_cores * 1000.f);


  // Add Temperature

  status.addf("Core Temperature", "%.2f C", temperature_avg);
  if (temperature_avg <= 60) {
    status.summary(diagnostic_msgs::DiagnosticStatus::OK, "Temperature OK");
  } else if (temperature_avg > 90) {
    status.summary(diagnostic_msgs::DiagnosticStatus::ERROR,
                   "Temperature Critical");
  } else {
    status.summary(diagnostic_msgs::DiagnosticStatus::WARN, "Temperature High");
  }
}

void SystemMonitor::testCamera(diagnostic_updater::DiagnosticStatusWrapper& status,
                               ros::Publisher& emergency_stop_publisher_) {
  double camera_startup = parameters_ptr_->getParam(PARAM_CAMERA_STARTUP_TIMEOUT);
  double camera_warning = parameters_ptr_->getParam(PARAM_CAMERA_TIMEOUT_WARNING);
  double camera_error = parameters_ptr_->getParam(PARAM_CAMERA_TIMEOUT_ERROR);
  auto time_now = ros::Time::now();
  if ((time_now - startup_time_).toSec() > camera_startup &&
      (time_now - last_camera_info_).toSec() > camera_warning) {
    if ((ros::Time::now() - last_camera_info_).toSec() > camera_error) {
      std_msgs::Bool message;
      message.data = true;
      emergency_stop_publisher_.publish(message);
      is_in_emergency_stop_ = true;
      ROS_ERROR_THROTTLE(2,
                         "Camera not working, last image is older than %f "
                         "seconds. Initiating emergency stop!",
                         camera_error);
      status.add("voice", "true");
      status.summary(diagnostic_msgs::DiagnosticStatus::ERROR,
                     "Camera not working");
    } else {
      ROS_WARN_THROTTLE(
          2, "Camera not working, last image is older than %f seconds!", camera_warning);
      status.summary(diagnostic_msgs::DiagnosticStatus::WARN,
                     "Camera not working!");
    }
  } else {
    if (is_in_emergency_stop_) {
      std_msgs::Bool message;
      message.data = false;
      emergency_stop_publisher_.publish(message);
      is_in_emergency_stop_ = false;
    }
    status.summary(diagnostic_msgs::DiagnosticStatus::OK, "Camera working");
  }
}

void SystemMonitor::onCameraInfo(const sensor_msgs::CameraInfo&) {
  // use current time instead of message stamp because message is too old when
  // callback is called
  last_camera_info_ = ros::Time::now();
}
