#include "linear_4ws_front_back_speed_kalman_filter_debug.h"

THIRD_PARTY_HEADERS_BEGIN

THIRD_PARTY_HEADERS_END

Linear4WSFrontBackSpeedKalmanFilterDebug::Linear4WSFrontBackSpeedKalmanFilterDebug(
    double update_rate, ParameterInterface &params, ros::NodeHandle *node_handle)
    : LinearSquareRootKalmanFilter(update_rate, params),
      LinearSquareRootKalmanFilterDebug(update_rate, params, node_handle),
      Linear4WSFrontBackSpeedKalmanFilter(update_rate, params) {}
