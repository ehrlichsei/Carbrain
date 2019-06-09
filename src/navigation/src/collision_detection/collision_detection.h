#ifndef COLLISION_DETECTION_H
#define COLLISION_DETECTION_H
#include <common/macros.h>

#include "common/parameter_interface.h"
#include "common/types.h"

#include "navigation/obstacle.h"
#include "navigation/gate.h"
#include "navigation/driving_corridor.h"
#include "vehicle.h"
#include "navigation/no_passing_zone.h"

THIRD_PARTY_HEADERS_BEGIN
#include "navigation_msgs/Obstacles.h"
#include "navigation_msgs/RoadClosures.h"
#include "navigation_msgs/Crosswalks.h"
#include "perception_msgs/ArrowMarkings.h"
#include "nav_msgs/Path.h"
#include "navigation_msgs/DrivePastNextRoadClosure.h"
#include "navigation_msgs/SetRespectNoPassingZones.h"
#include <std_srvs/Empty.h>
THIRD_PARTY_HEADERS_END

/*!
 * \brief detects obstacles that collide with lane points
 */
class CollisionDetection {  
 public:
  /*!
  * \brief CollisionDetection is the consstructor. A ros indipendent
  * functionality containing
  * class needs a pointer to a ParameterInterface (in fact a ParameterHandler)
  * to get access to parameters.
  * \param parameters the ParameterInterface
  */
  CollisionDetection(ParameterInterface* parameters);

  void updateParameter();

  navigation::Obstacle::ObstacleList determineObstaclesOnCorridor(
      const navigation::Obstacle::ObstacleList& obstacles, const DrivingCorridor& full_corridor);

  bool isOnRightLane(const navigation::Obstacle& obstacle, const DrivingCorridor& full_corridor);

  navigation::Obstacle::ObstacleList determineObstaclesOnRightLane(
      const navigation::Obstacle::ObstacleList& obstacles, const DrivingCorridor& full_corridor);

  void separateObstaclesByLane(const navigation::Obstacle::ObstacleList& obstacles,
                               const DrivingCorridor& full_corridor,
                               navigation::Obstacle::ObstacleList& obstacles_left_lane,
                               navigation::Obstacle::ObstacleList& obstacles_right_lane);

  navigation::Obstacle getClosestObstacle(const Eigen::Vector3d center,
                                          const navigation::Obstacle::ObstacleList& obstacles);

  void cropAroundObstacles(DrivingCorridor& safe_corridor,
                           navigation::Obstacle::ObstacleList obstacles);

  void cropAtIntersections(DrivingCorridor& safe_corridor, Eigen::Vector3d& right_stopline);

  void cropAroundRoadClosures(DrivingCorridor& safe_corridor,
                              const navigation_msgs::RoadClosuresConstPtr& roadClosures);

  void cropAtCrosswalks(DrivingCorridor& safe_corridor,
                              const navigation_msgs::CrosswalksConstPtr& crosswalks);

  void cropAtArrowMarkings(DrivingCorridor& safe_corridor,
                              const perception_msgs::ArrowMarkingsConstPtr& crosswalks);

  bool setDrivePastNextRoadClosure(navigation_msgs::DrivePastNextRoadClosureRequest& req,
                                   navigation_msgs::DrivePastNextRoadClosureResponse& res);
  bool setRespectNoPassingZones(navigation_msgs::SetRespectNoPassingZonesRequest& req,
                                navigation_msgs::SetRespectNoPassingZonesResponse& res);

  void setNoPassingZones(const std::vector<NoPassingZone>& no_passing_zones);

  static const double LENGTH_OF_LEFT_LANE_BLOCKED;  // length of left lane
  // blocked if an obstalce is on the
  // left side of the road
  // closure


 private:
  /*!
  * \brief parameters_ptr_ is needed for parameter access
  */
  ParameterInterface* parameters_ptr_;

  bool drivePastNextRoadClosure = true;
  bool respect_no_passing_zones = false;


  Eigen::Vector3d setGatePoleOnRoadClosureBorder(
      const Eigen::Vector2d& changingPole,
      const Eigen::Vector2d& otherPole,
      const common::EigenAlignedVector<Eigen::Vector3d>& hullPolygon);
  common::EigenAlignedVector<Eigen::Vector3d> convertHullPolygon(
      const std::vector<geometry_msgs::Point>& hullPolygon);
  void extendRoadClosureSafetyMargin(common::EigenAlignedVector<Eigen::Vector3d>& hullPolygon,
                                     double safetyMargin);
  void blockLeftLaneBeforeRoadClosure(Gate& gate,
                                      double blockDistanceCarInRoadClosureArea,
                                      double distanceToStart);

  void blockLeftLane(Gate& gate, const navigation::Obstacle::ObstacleList& obstacles_right_lane);

  double left_lane_block_distance_in_front_of_obstacle = 0.0;
  double left_lane_block_distance_behind_obstacle = 0.0;
  bool should_avoid_obstacles = false;
  std::vector<NoPassingZone> no_passing_zones;
};

#endif  // COLLISION_DETECTION_H
