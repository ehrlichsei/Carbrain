#ifndef FSM_EVENTS_H
#define FSM_EVENTS_H

#include <common/macros.h>

THIRD_PARTY_HEADERS_BEGIN
#include <chrono>
#include <memory>
#include <Eigen/Dense>
#include <ros/ros.h>
#include <tf2_eigen/tf2_eigen.h>
THIRD_PARTY_HEADERS_END

#include "turn_direction.h"
#include "navigation/obstacle.h"

// General events
struct EventStart {};
struct EventReset {};
struct EventNoQRCodeVisible {};
struct EventQRDetection {};
struct EventTimeUpdate {
  EventTimeUpdate(const ros::Time current_time) : current_time(current_time) {}
  const ros::Time current_time;
};
struct EventUpdateObserver {
  EventUpdateObserver(const Eigen::Affine3d& vehicle_pose)
      : vehicle_pose(vehicle_pose) {}
  const Eigen::Affine3d vehicle_pose;
};

struct EventDroveDistance {
  EventDroveDistance(const double distance) : distance(distance) {}
  const double distance;
};
struct EventCarHasStopped {
  EventCarHasStopped(const unsigned long stopping_id)
      : stopping_id(stopping_id) {}
  unsigned long stopping_id;
};

// Crosswalk events
struct EventCrosswalkWithPedestriansDetected {
  EventCrosswalkWithPedestriansDetected(double distance, long id)
      : distance(distance), id(id) {}
  const double distance;
  const long id;
};
struct EventCrosswalkFreeDetected {
  EventCrosswalkFreeDetected(const double distance, const long id)
      : distance(distance), id(id) {}
  const double distance;
  const long id;
};
struct EventCrosswalkPassed {};
struct EventNoCrosswalk {};

// Junction events
struct EventJunctionBlockedByObstacle {};

struct EventEmptyJunction {};
struct EventNoJunction {};
struct EventStopJunction {
  EventStopJunction(const double distance, const unsigned long id)
      : distance(distance), id(id) {}
  const double distance;
  const unsigned long id;
};
struct EventHoldJunction {
  EventHoldJunction(const double distance, const unsigned long id)
      : distance(distance), id(id) {}
  const double distance;
  const unsigned long id;
};

// Road closure events
struct EventRoadClosureBlockedDetected {
  EventRoadClosureBlockedDetected(double distance, const unsigned long id)
      : distance(distance), id(id) {}
  const double distance;
  const unsigned long id;
};
struct EventRoadClosureFreeDetected {};
struct EventNoRoadClosure {};

// Turns
struct EventTurnDetected {
  EventTurnDetected(long id,
                    Eigen::Affine3d turn_junction_pose_in_world,
                    double driving_distance,
                    TurnDirection direction,
                    ros::Time timestamp)
      : id(id),
        turn_junction_pose_in_world(turn_junction_pose_in_world),
        driving_distance(driving_distance),
        direction(direction),
        timestamp(timestamp) {}
  long id;
  Eigen::Affine3d turn_junction_pose_in_world;
  double driving_distance;
  TurnDirection direction;
  ros::Time timestamp;
};
struct EventTurnPassed {
  EventTurnPassed(const unsigned long id) : id(id) {}
  const unsigned long id;
};
struct EventNoTurn {};

// Speed Limit
struct EventSpeedLimitStartPassed {
  EventSpeedLimitStartPassed(const long id, const unsigned int speed_limit_in_km_per_h)
      : id(id), speed_limit_in_km_per_h(speed_limit_in_km_per_h) {}
  const long id;
  const unsigned int speed_limit_in_km_per_h;
};
struct EventSpeedLimitEndPassed {
  EventSpeedLimitEndPassed(const long id) : id(id) {}
  const long id;
};

// No Passing Zone
struct EventNoPassingZone {
  EventNoPassingZone(const Eigen::Vector3d start,
                     const Eigen::Vector3d end,
                     const Eigen::Affine3d vehicle_pose)
      : start(start), end(end), vehicle_pose(vehicle_pose) {}
  const Eigen::Vector3d start;
  const Eigen::Vector3d end;
  const Eigen::Affine3d vehicle_pose;
};

struct EventObstacleAheadDetectedRight {
  EventObstacleAheadDetectedRight(const double obstacle_speed, const double driving_distance)
      : obstacle_speed(obstacle_speed), driving_distance(driving_distance) {}
  double obstacle_speed;
  double driving_distance;
};

struct EventNoObstacleAheadDetectedRight {};

struct EventNoPassingMessageEmpty {
  EventNoPassingMessageEmpty(const ros::Time stamp) : stamp(stamp) {}
  ros::Time stamp;
};


// Events for parking
struct EventStartLineDetected {
  EventStartLineDetected(const double dist_before_vehicle)
      : dist_before_vehicle(dist_before_vehicle) {}
  double dist_before_vehicle;
};
struct EventFoundParkingSpot {
  EventFoundParkingSpot(const Eigen::Vector3d& car_position,
                        const perception_msgs::PerpendicularParkingSpot& parking_spot)
      : car_position(car_position), parking_spot(parking_spot) {}
  const Eigen::Vector3d car_position;
  const perception_msgs::PerpendicularParkingSpot parking_spot;
};
struct EventPathMsgUpdateReversePath {
  EventPathMsgUpdateReversePath(const nav_msgs::Path::ConstPtr& path_msg_p,
                                Eigen::Affine3d path_pose)
      : path_msg(path_msg_p), path_pose(path_pose) {}
  const nav_msgs::Path::ConstPtr path_msg;
  const Eigen::Affine3d path_pose;
};
struct EventPathMsgStopping {
  EventPathMsgStopping(const nav_msgs::Path::ConstPtr& path_msg_p,
                       Eigen::Affine3d path_pose,
                       Eigen::Affine3d vehicle_pose)
      : path_msg(path_msg_p), path_pose(path_pose), vehicle_pose(vehicle_pose) {}
  const nav_msgs::Path::ConstPtr path_msg;
  const Eigen::Affine3d path_pose;
  const Eigen::Affine3d vehicle_pose;
};
struct EventOnRightLane {};
struct EventNotOnRightLane {};
struct EventParkedInParkingLot {};
struct EventNoUnidentifiedObject {
  EventNoUnidentifiedObject(ros::Time stamp) : stamp(stamp) {}
  const ros::Time stamp;
};
struct EventUnidentifiedObject {
  EventUnidentifiedObject(const double distance) : distance(distance) {}
  const double distance;
};
#endif
