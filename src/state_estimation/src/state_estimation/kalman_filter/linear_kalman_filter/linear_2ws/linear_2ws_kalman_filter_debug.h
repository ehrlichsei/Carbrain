#ifndef LINEAR_2WS_KALMAN_FILTER_DEBUG_H
#define LINEAR_2WS_KALMAN_FILTER_DEBUG_H

#include <common/macros.h>

THIRD_PARTY_HEADERS_BEGIN

THIRD_PARTY_HEADERS_END

#include "../linear_square_root_kalman_filter_debug.h"
#include "linear_2ws_kalman_filter.h"

class Linear2WSKalmanFilterDebug : public LinearSquareRootKalmanFilterDebug<3u, 4u>,
                                   public Linear2WSKalmanFilter {
 public:
  Linear2WSKalmanFilterDebug(double update_rate,
                             ParameterInterface& params,
                             ros::NodeHandle* node_handle);
};

#endif  // LINEAR_2WS_KALMAN_FILTER_DEBUG_H
