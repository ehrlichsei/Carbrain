#ifndef ROUND_DETECTION_H
#define ROUND_DETECTION_H
#include <common/macros.h>

THIRD_PARTY_HEADERS_BEGIN
#include <ros/ros.h>
#include <Eigen/Geometry>
THIRD_PARTY_HEADERS_END

#include "fitness_monitor.h"

const ParameterString<int> ROUND_DETECTION_TYPE =
    ParameterString<int>("round_detection_type");
const ParameterString<double> ROUND_DETECTION_TIME =
    ParameterString<double>("round_detection_time");

class RoundDetection {
 public:
  enum RoundDetectionType { START_LINE = 0, TIME = 1 };

  RoundDetection(ParameterInterface* parameters, FitnessMonitor* fitness_monitor);

  void updateVehiclePosition(const Eigen::Affine3d& vehicle_position);
  void updateStartlinePosition(const Eigen::Affine3d& start_line);

 private:
  bool hasRoundCompleted(const Eigen::Affine3d& vehicle_position);
  bool hasCrossedStartLine(const Eigen::Affine3d& vehicle_position);
  bool hasRoundTimeExpired();

  ParameterInterface* parameters_ptr_;
  FitnessMonitor* fitness_monitor_ptr_;
  Eigen::Affine3d start_line_;
  ros::Time round_started_;
  bool start_line_found_ = false;
  double min_distance_to_start_line_ = std::numeric_limits<double>::max();
};

#endif  // ROUND_DETECTION_H
