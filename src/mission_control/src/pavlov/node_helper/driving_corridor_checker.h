#ifndef DRIVINGCORRIDORCHECKER_H
#define DRIVINGCORRIDORCHECKER_H

#include "common/macros.h"

THIRD_PARTY_HEADERS_BEGIN
#include <Eigen/Dense>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_eigen/tf2_eigen.h>
#include <ros/console.h>
#include <navigation_msgs/DrivingCorridor.h>
#include <navigation_msgs/Obstacles.h>
THIRD_PARTY_HEADERS_END

#include "common/types.h"
#include "navigation/driving_corridor.h"
#include "navigation/gate.h"
#include "navigation/obstacle.h"

class DrivingCorridorChecker {
 public:
  void handleFullCorridor(const navigation_msgs::DrivingCorridor::ConstPtr& full_corridor_msg);
  void handleObstacles(const navigation_msgs::Obstacles& obstacles_msg);
  bool hasObstacleOnRightLane(const Eigen::Vector3d& start_point,
                              const double distance_of_interest,
                              double& distance_to_next_obstacle,
                              double& velocity_next_obstacle);
  void reset();

 private:
  std::vector<navigation::Obstacle> getObstaclesOnRightLaneAhead(const Eigen::Vector3d& start_point);

  navigation::Obstacle::ObstacleList obstacles;
  DrivingCorridor corridor_;
};

#endif  // DRIVINGCORRIDORCHECKER_H
