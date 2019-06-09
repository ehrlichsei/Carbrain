#ifndef LINEAR_4WS_FRONT_BACK_SPEED_KALMAN_FILTER_DEBUG_H
#define LINEAR_4WS_FRONT_BACK_SPEED_KALMAN_FILTER_DEBUG_H

#include <common/macros.h>

THIRD_PARTY_HEADERS_BEGIN

THIRD_PARTY_HEADERS_END

#include "linear_4ws_front_back_speed_kalman_filter.h"
#include "../linear_square_root_kalman_filter_debug.h"


class Linear4WSFrontBackSpeedKalmanFilterDebug
    : public LinearSquareRootKalmanFilterDebug<3u, 6u>,
      public Linear4WSFrontBackSpeedKalmanFilter {
 public:
  Linear4WSFrontBackSpeedKalmanFilterDebug(double update_rate,
                                           ParameterInterface& params,
                                           ros::NodeHandle* node_handle);
};

#endif  // LINEAR_4WS_FRONT_BACK_SPEED_KALMAN_FILTER_DEBUG_H
