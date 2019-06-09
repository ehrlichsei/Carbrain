#include "round_detection.h"

RoundDetection::RoundDetection(ParameterInterface* parameters, FitnessMonitor* fitness_monitor)
    : parameters_ptr_(parameters),
      fitness_monitor_ptr_(fitness_monitor),
      round_started_(ros::Time::now()) {
  parameters->registerParam(ROUND_DETECTION_TYPE);
  parameters->registerParam(ROUND_DETECTION_TIME);
}

void RoundDetection::updateVehiclePosition(const Eigen::Affine3d& vehicle_position) {
  if (hasRoundCompleted(vehicle_position)) {
    // start line passed
    fitness_monitor_ptr_->publishCurrentFitness();
    fitness_monitor_ptr_->resetCurrentFitness();
  }
}

bool RoundDetection::hasRoundCompleted(const Eigen::Affine3d& vehicle_position) {
  const int type = parameters_ptr_->getParam(ROUND_DETECTION_TYPE);
  switch (type) {
    case START_LINE:
      return hasCrossedStartLine(vehicle_position);
    case TIME:
      return hasRoundTimeExpired();
  }
  return false;
}

bool RoundDetection::hasCrossedStartLine(const Eigen::Affine3d& vehicle_position) {
  const double new_distance =
      (vehicle_position.translation() - vehicle_position.translation()).norm();
  if (start_line_found_ && min_distance_to_start_line_ > new_distance) {
    start_line_found_ = false;
    min_distance_to_start_line_ = std::numeric_limits<double>::max();
    return true;
  } else {
    min_distance_to_start_line_ = new_distance;
  }
  return false;
}

bool RoundDetection::hasRoundTimeExpired() {
  const double round_time = parameters_ptr_->getParam(ROUND_DETECTION_TIME);
  if (ros::Time::now() - round_started_ > ros::Duration(round_time)) {
    round_started_ = ros::Time::now();
    return true;
  } else {
    return false;
  }
}

void RoundDetection::updateStartlinePosition(const Eigen::Affine3d& start_line) {
  start_line_ = start_line;
  start_line_found_ = true;
  min_distance_to_start_line_ = std::numeric_limits<double>::max();
}
