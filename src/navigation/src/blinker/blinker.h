#ifndef BLINKER_H
#define BLINKER_H
#include <common/macros.h>

#include <navigation/driving_corridor.h>
#include "blinker_decision.h"
#include "common/parameter_interface.h"

THIRD_PARTY_HEADERS_BEGIN
#include <nav_msgs/Path.h>
#include <navigation_msgs/Obstacles.h>
#include <Eigen/Core>
THIRD_PARTY_HEADERS_END

/*!
 * \brief Takes care of activating the blinker before switching lanes
 */
class Blinker {
 public:
  enum class LanePosition { RIGHT, LEFT, NONE };
  static const ParameterString<double> VEHICLE_WIDTH;
  static const ParameterString<double> VEHICLE_WIDTH_SUBTRACTIION;

  /**
   * \brief decideBlink returns a blinker decision for a situation
   */
  BlinkerDecision decideBlink(DrivingCorridor &safe_corridor,
                              const Eigen::Affine3d &vehicle_pose);

 private:
  LanePosition last_vehicle_lane_position = LanePosition::NONE;
  static const int DECISION_THRESHOLD = 3;
};

#endif  // BLINKER_H
