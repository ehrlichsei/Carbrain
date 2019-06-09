#include "kalman_filter.h"

KalmanFilter::KalmanFilter(double update_rate, ParameterInterface &params)
    : car_specs(params), parameter_interface(params) {
  time_between_updates = 1. / update_rate;
  ROS_INFO("kalman filter initialized with an update_rate of %.2fHz", update_rate);
}

void KalmanFilter::reset() {
  resetStateAndStateCovariance();
  ROS_WARN("reset kalman filter");
}

void KalmanFilter::updateParams() {
  car_specs.updateParams(parameter_interface);
  updateKalmanFilterParams();
}
