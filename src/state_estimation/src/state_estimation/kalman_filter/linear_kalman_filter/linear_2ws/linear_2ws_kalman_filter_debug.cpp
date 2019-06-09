#include "linear_2ws_kalman_filter_debug.h"

THIRD_PARTY_HEADERS_BEGIN

THIRD_PARTY_HEADERS_END

Linear2WSKalmanFilterDebug::Linear2WSKalmanFilterDebug(double update_rate,
                                                       ParameterInterface &params,
                                                       ros::NodeHandle *node_handle)
    : LinearSquareRootKalmanFilter(update_rate, params),
      LinearSquareRootKalmanFilterDebug(update_rate, params, node_handle),
      Linear2WSKalmanFilter(update_rate, params) {}
