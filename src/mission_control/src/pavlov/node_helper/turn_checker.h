#ifndef TURN_CHECKER_H
#define TURN_CHECKER_H

#include "common/macros.h"

THIRD_PARTY_HEADERS_BEGIN
#include <functional>
#include <Eigen/Dense>
#include <tf2_eigen/tf2_eigen.h>
#include <ros/console.h>
#include <navigation_msgs/Junctions.h>
#include <perception_msgs/ArrowMarkings.h>
THIRD_PARTY_HEADERS_END

#include "common/parameter_interface.h"

class TurnChecker {
 public:
  TurnChecker(ParameterInterface* parameter_interface);
  bool findTurnJunctionPose(const perception_msgs::ArrowMarking& msg,
                            Eigen::Affine3d& out_turn_junction_pose);
  Eigen::Affine3d getLatestWorldToTurnJunctionTransform();

  void setJunctions(const navigation_msgs::Junctions& junctions);

 private:
  bool findTurnJunctionPose(const Eigen::Affine3d& arrow_to_world_transform,
                            Eigen::Affine3d& out_junction_to_world_transform);

  static const ParameterString<double> PARAM_TURN_ARROW_TO_JUNCTION_DISTANCE_THRESHOLD;
  static const ParameterString<double> PARAM_TURN_ARROW_TO_JUNCTION_ASSUMED_DISTANCE;

  const ParameterInterface* const parameter_interface_;
  navigation_msgs::Junctions junctions_;
  Eigen::Affine3d latest_world_to_turn_junction_transform;
};

#endif
