#include "collision_detection.h"
#include <common/macros.h>
#include <navigation/driving_corridor.h>
THIRD_PARTY_HEADERS_BEGIN
#include <eigen_conversions/eigen_msg.h>
#include <geometry_msgs/Point.h>
#include <ros/ros.h>
#include <eigen_conversions/eigen_msg.h>
#include <boost/algorithm/cxx11/any_of.hpp>
#include <boost/algorithm/cxx11/none_of.hpp>
#include <boost/algorithm/cxx11/copy_if.hpp>
#include "navigation_msgs/Obstacle.h"
#include <boost/range/algorithm_ext/erase.hpp>
THIRD_PARTY_HEADERS_END
#include <common/best_score.h>
#include <common/eigen_utils.h>
#include <common/tf2_eigen_addon.h>
#include "navigation/triangle.h"

const ParameterString<bool> PARAM_BLOCK_LEFT_LANE("block_left_lane");
const ParameterString<double> PARAM_LEFT_LANE_BLOCK_DISTANCE_IN_FRONT_OF_OBSTACLE(
    "left_lane_block_distance_in_front_of_obstacle");
const ParameterString<double> PARAM_LEFT_LANE_BLOCK_DISTANCE_BEHIND_OBSTACLE(
    "left_lane_block_distance_behind_obstacle");
const ParameterString<double> PARAM_LEFT_LANE_BLOCK_DISTANCE_IN_FRONT_OF_ROAD_CLOSURE(
    "left_lane_block_distance_in_front_of_road_closure");
const ParameterString<double> PARAM_LEFT_LANE_BLOCK_DISTANCE_BEHIND_ROAD_CLOSURE(
    "left_lane_block_distance_behind_road_closure");
const ParameterString<double> PARAM_BLOCK_DISTANCE_CAR_IN_ROAD_CLOSURE_AREA(
    "block_distance_car_in_road_closure_area");
const ParameterString<bool> PARAM_AVOID_OBSTACLES("avoid_obstacles");
const ParameterString<double> PARAM_OBSTACLE_SAFETY_MARGIN(
    "obstacle_safety_margin");
const ParameterString<double> PARAM_ROAD_CLOSURE_SAFETY_MARGIN(
    "road_closure_safety_margin");
const double CollisionDetection::LENGTH_OF_LEFT_LANE_BLOCKED = 0.1;

using namespace navigation;

CollisionDetection::CollisionDetection(ParameterInterface* parameters)
    : parameters_ptr_(parameters) {
  parameters->registerParam(PARAM_BLOCK_LEFT_LANE);
  parameters->registerParam(PARAM_LEFT_LANE_BLOCK_DISTANCE_IN_FRONT_OF_OBSTACLE);
  parameters->registerParam(PARAM_LEFT_LANE_BLOCK_DISTANCE_BEHIND_OBSTACLE);
  parameters->registerParam(PARAM_LEFT_LANE_BLOCK_DISTANCE_IN_FRONT_OF_ROAD_CLOSURE);
  parameters->registerParam(PARAM_LEFT_LANE_BLOCK_DISTANCE_BEHIND_ROAD_CLOSURE);
  parameters->registerParam(PARAM_BLOCK_DISTANCE_CAR_IN_ROAD_CLOSURE_AREA);
  parameters->registerParam(PARAM_AVOID_OBSTACLES);
  parameters->registerParam(PARAM_OBSTACLE_SAFETY_MARGIN);
  parameters->registerParam(PARAM_ROAD_CLOSURE_SAFETY_MARGIN);
}

auto isOn(const DrivingCorridor& full_corridor) {
  return [&full_corridor](const Obstacle& obstacle) {
    return boost::algorithm::any_of(obstacle.getCorners(),
                                    [&full_corridor](const Eigen::Vector3d& p) {
                                      return full_corridor.isPointContained(p);
                                    });

  };
}

Obstacle::ObstacleList CollisionDetection::determineObstaclesOnCorridor(
    const Obstacle::ObstacleList& obstacles, const DrivingCorridor& full_corridor) {
  Obstacle::ObstacleList obstacles_corridor;
  using namespace boost::algorithm;
  copy_if(obstacles, std::back_inserter(obstacles_corridor), isOn(full_corridor));
  return obstacles_corridor;
}

bool CollisionDetection::isOnRightLane(const Obstacle& obstacle,
                                       const DrivingCorridor& full_corridor) {
  return full_corridor.isPointOnRightLane(obstacle.getCenter());
}

void CollisionDetection::separateObstaclesByLane(const Obstacle::ObstacleList& obstacles,
                                                 const DrivingCorridor& full_corridor,
                                                 Obstacle::ObstacleList& obstacles_left_lane,
                                                 Obstacle::ObstacleList& obstacles_right_lane) {
  for (const Obstacle& obstacle : obstacles) {
    if (isOnRightLane(obstacle, full_corridor)) {
      obstacles_right_lane.push_back(obstacle);
    } else {
      obstacles_left_lane.push_back(obstacle);
    }
  }
}

Obstacle CollisionDetection::getClosestObstacle(const Eigen::Vector3d center,
                                                const Obstacle::ObstacleList& obstacles) {
  return *common::min_score(obstacles,
                            [&](const auto& obstacle) {
                              return obstacle.approximateProjectedDistance(center);
                            });
}

bool isBehindGate(const Gate& gate, const Eigen::Vector3d& v) {
  return (gate.getTransformationToGateFrame() * v)[1] < 0;
}

void CollisionDetection::blockLeftLane(Gate& gate, const Obstacle::ObstacleList& obstacles_right_lane) {
  double dist_to_closest_obstacle_behind_vehicle = std::numeric_limits<double>::max();
  double dist_to_closest_obstacle_in_front_of_vehicle =
      std::numeric_limits<double>::max();
  for (const Obstacle& obstacle : obstacles_right_lane) {
    if (isBehindGate(gate, obstacle.getCenter())) {
      dist_to_closest_obstacle_behind_vehicle =
          std::min(dist_to_closest_obstacle_behind_vehicle,
                   obstacle.approximateProjectedDistance(gate.getCenter()));
    } else {
      dist_to_closest_obstacle_in_front_of_vehicle =
          std::min(dist_to_closest_obstacle_in_front_of_vehicle,
                   obstacle.approximateProjectedDistance(gate.getCenter()));
    }
  }

  if (!should_avoid_obstacles ||
      (dist_to_closest_obstacle_in_front_of_vehicle >= left_lane_block_distance_in_front_of_obstacle &&
       dist_to_closest_obstacle_behind_vehicle >= left_lane_block_distance_behind_obstacle)) {
    gate.setLeftPole(gate.getLaneCenter());
    return;
  }

  if (dist_to_closest_obstacle_in_front_of_vehicle < left_lane_block_distance_in_front_of_obstacle &&
      dist_to_closest_obstacle_behind_vehicle < left_lane_block_distance_behind_obstacle) {
    gate.setPreferedPathWeight(0.0);
    return;
  }

  float weight = std::max(1.0,
                          std::min((dist_to_closest_obstacle_in_front_of_vehicle) /
                                       left_lane_block_distance_in_front_of_obstacle,
                                   (dist_to_closest_obstacle_behind_vehicle) /
                                       left_lane_block_distance_behind_obstacle));
  gate.setPreferedPathWeight(weight * weight);
}

auto findClosestIntersectionPoint(const Eigen::Vector2d& changingPole,
                                  const Eigen::Vector2d& otherPole,
                                  const boost::array<LineSegment, 4>& line_segments) {
  Eigen::Vector2d closest_intersection_point = changingPole;
  const LineSegment gate_line_segment(changingPole, otherPole);
  const LineSegment& min_line_segment =
      *common::min_score(line_segments,
                         [&](const LineSegment& line) -> double {
                           Eigen::Vector2d intersection_point;
                           if (!gate_line_segment.intersects(line, &intersection_point)) {
                             return std::numeric_limits<double>::infinity();
                           }
                           return (otherPole - intersection_point).norm();
                         });
  gate_line_segment.intersects(min_line_segment, &closest_intersection_point);
  return to3D(closest_intersection_point);
}

boost::array<LineSegment, 4> toLineSegments(const Obstacle& obstacle) {
  boost::array<LineSegment, 4> lines;
  auto corners = obstacle.getCornersProjection();
  corners.emplace_back(corners.front());
  std::transform(corners.begin(),
                 corners.end() - 1,
                 corners.begin() + 1,
                 lines.begin(),
                 [](const Eigen::Vector2d& a, const Eigen::Vector2d& b) {
                   return LineSegment(a, b);
                 });
  return lines;
}



void CollisionDetection::cropAroundObstacles(DrivingCorridor& safe_corridor,
                                             Obstacle::ObstacleList obstacles) {
  const bool should_block_left_lane = parameters_ptr_->getParam(PARAM_BLOCK_LEFT_LANE);

  left_lane_block_distance_in_front_of_obstacle =
      parameters_ptr_->getParam(PARAM_LEFT_LANE_BLOCK_DISTANCE_IN_FRONT_OF_OBSTACLE);
  left_lane_block_distance_behind_obstacle =
      parameters_ptr_->getParam(PARAM_LEFT_LANE_BLOCK_DISTANCE_BEHIND_OBSTACLE);
  should_avoid_obstacles = parameters_ptr_->getParam(PARAM_AVOID_OBSTACLES);


  Obstacle::ObstacleList obstacles_left_lane;
  Obstacle::ObstacleList obstacles_right_lane;
  if (should_avoid_obstacles) {
    const double obstacle_safety_margin =
        parameters_ptr_->getParam(PARAM_OBSTACLE_SAFETY_MARGIN);
    Obstacle::extendAllBy(obstacles, obstacle_safety_margin);
    const Obstacle::ObstacleList obstacles_road =
        determineObstaclesOnCorridor(obstacles, safe_corridor);
    separateObstaclesByLane(
        obstacles_road, safe_corridor, obstacles_left_lane, obstacles_right_lane);
    const auto obstacleNearNPZone = [&](const Obstacle& obstacle) {
      return boost::algorithm::any_of(
          no_passing_zones,
          [&](const auto& zone) {
            return obstacle.approximateProjectedDistance(zone.getEnd()) <
                       zone.getSafetyMarginAfterZone() ||
                   zone.pointIsInNoPassingZone(obstacle.getCenter());
          });
    };
    boost::remove_erase_if(obstacles_right_lane, obstacleNearNPZone);
  }

  Gate lastGate = safe_corridor.at(0);
  for (Gate& gate : safe_corridor) {
    if (respect_no_passing_zones) {
      bool continueFor = false;
      for (const auto& zone : no_passing_zones) {
        if (zone.pointIsInNoPassingZone(gate.getCenter())) {
          gate.setLeftPole(gate.getLaneCenter());
          continueFor = true;
          break;
        }
      }
      if (continueFor) {
        continue;
      }
    }
    if (should_block_left_lane) {
      blockLeftLane(gate, obstacles_right_lane);
    }
    // handle collisions
    if (should_avoid_obstacles) {
      if (!obstacles_right_lane.empty()) {
        const Obstacle closest_obstacle =
            getClosestObstacle(gate.getRightPole(), obstacles_right_lane);
        gate.setRightPole(
            findClosestIntersectionPoint(gate.getRightPole().topRows<2>(),
                                         gate.getLeftPole().topRows<2>(),
                                         toLineSegments(closest_obstacle)));
      }
      if (!obstacles_left_lane.empty()) {
        Obstacle closest_obstacle =
            getClosestObstacle(gate.getLeftPole(), obstacles_left_lane);
        gate.setLeftPole(
            findClosestIntersectionPoint(gate.getLeftPole().topRows<2>(),
                                         gate.getRightPole().topRows<2>(),
                                         toLineSegments(closest_obstacle)));
      }
    }
    lastGate = gate;
  }
}

void CollisionDetection::cropAtIntersections(DrivingCorridor& safe_corridor,
                                             Eigen::Vector3d& right_stopline) {
  if (safe_corridor.empty()) {
    return;
  }
  if ((safe_corridor.back().getTransformationToGateFrame() * right_stopline).y() > 0) {
    safe_corridor.back().setLeftPole(safe_corridor.back().getLaneCenter());
    return;
  }
  for (size_t i = 0; i < safe_corridor.size(); i++) {
    Gate& gate = safe_corridor.at(i);
    double distance_to_stopline =
        (gate.getTransformationToGateFrame() * right_stopline).y();
    if (-0.45 <= distance_to_stopline && distance_to_stopline <= 0.1) {
      gate.setLeftPole(gate.getLaneCenter());
    }
  }
}

void CollisionDetection::cropAroundRoadClosures(DrivingCorridor& safe_corridor,
                                                const navigation_msgs::RoadClosuresConstPtr& roadClosures) {
  if (roadClosures->sub_messages.empty() || safe_corridor.empty()) {
    return;
  }
  const double safety_margin = parameters_ptr_->getParam(PARAM_ROAD_CLOSURE_SAFETY_MARGIN);
  const double left_lane_block_distance_in_front_of_road_closure =
      parameters_ptr_->getParam(PARAM_LEFT_LANE_BLOCK_DISTANCE_IN_FRONT_OF_ROAD_CLOSURE);
  const double left_lane_block_distance_behind_road_closure =
      parameters_ptr_->getParam(PARAM_LEFT_LANE_BLOCK_DISTANCE_BEHIND_ROAD_CLOSURE);
  const double blockDistanceCarInRoadClosureArea =
      parameters_ptr_->getParam(PARAM_BLOCK_DISTANCE_CAR_IN_ROAD_CLOSURE_AREA);
  const int nextRoadClosureId =
      common::min_score(
          roadClosures->sub_messages,
          [&](navigation_msgs::RoadClosure roadClosure) -> double {
            double score = (safe_corridor.at(0).getLaneCenter() -
                            convertHullPolygon(roadClosure.hull_polygon)[3]).norm();
            return score > 0 ? score : std::numeric_limits<double>::infinity();
          })->id;

  for (const navigation_msgs::RoadClosure& roadClosure : roadClosures->sub_messages) {
    common::EigenAlignedVector<Eigen::Vector3d> hullPolygon =
        convertHullPolygon(roadClosure.hull_polygon);
    extendRoadClosureSafetyMargin(hullPolygon, safety_margin);
    const bool isNextRoadClosure = roadClosure.id == nextRoadClosureId;
    for (size_t i = 0; i < safe_corridor.size(); ++i) {
      Gate& gate = safe_corridor.at(i);
      double distanceToStart = (gate.getRightPole() - hullPolygon[0]).norm();
      if (isBehindGate(gate, hullPolygon[0])) {
        distanceToStart *= -1;
      }
      if (distanceToStart > left_lane_block_distance_in_front_of_road_closure) {
        // checks if safeCorridor should be restricted to right lane
        if (isNextRoadClosure) {
          blockLeftLaneBeforeRoadClosure(gate, blockDistanceCarInRoadClosureArea, distanceToStart);
        }
        continue;
      }
      double distanceToEnd = (gate.getRightPole() - hullPolygon[3]).norm();
      if (isBehindGate(gate, hullPolygon[3])) {
        distanceToEnd *= -1;
      }
      if (distanceToEnd < -left_lane_block_distance_behind_road_closure) {
        i = safe_corridor.size();  // continue with next road closure
        continue;
      }
      // gate is in critical section
      const bool onRightLane = (hullPolygon[0] - gate.getRightLaneBoundary()).norm() <
                               (hullPolygon[0] - gate.getLeftLaneBoundary()).norm();
      if (onRightLane) {
        gate.setLeftPole(gate.getLeftLaneBoundary());
      }
      // checks if safeCorridor should be restricted to right lane
      if (isNextRoadClosure) {
        blockLeftLaneBeforeRoadClosure(gate, blockDistanceCarInRoadClosureArea, distanceToStart);
      }
      if (distanceToStart <= 0 && distanceToEnd >= 0) {
        // gate is in roadClosure area
        const boost::array<LineSegment, 4> road_closure_line_segments = {
            {LineSegment(hullPolygon[0].topRows<2>(), hullPolygon[1].topRows<2>()),
             LineSegment(hullPolygon[1].topRows<2>(), hullPolygon[2].topRows<2>()),
             LineSegment(hullPolygon[2].topRows<2>(), hullPolygon[3].topRows<2>()),
             LineSegment(hullPolygon[3].topRows<2>(), hullPolygon[0].topRows<2>())}};
        if (onRightLane) {
          gate.setRightPole(findClosestIntersectionPoint(
              gate.getRightPole().topRows<2>(), gate.getLeftPole().topRows<2>(), road_closure_line_segments));
        } else {
          gate.setLeftPole(findClosestIntersectionPoint(
              gate.getLeftPole().topRows<2>(), gate.getRightPole().topRows<2>(), road_closure_line_segments));
        }
      }
    }
  }
}

void CollisionDetection::cropAtCrosswalks(DrivingCorridor& safe_corridor,
                                          const navigation_msgs::CrosswalksConstPtr& crosswalks) {
  for (size_t j = 0; j < crosswalks->sub_messages.size(); j++) {
    for (size_t i = 0; i < safe_corridor.size(); i++) {
      Gate& gate = safe_corridor.at(i);
      Eigen::Vector3d crosswalk_position;
      tf2::fromMsg(crosswalks->sub_messages[j].pose.position, crosswalk_position);
      double distance_to_crosswalk =
          (gate.getTransformationToGateFrame() * crosswalk_position).y();
      if (-0.45 <= distance_to_crosswalk && distance_to_crosswalk <= 0.25) {
        gate.setLeftPole(gate.getLaneCenter());
      }
    }
  }
}

void CollisionDetection::cropAtArrowMarkings(DrivingCorridor& safe_corridor,
                                             const perception_msgs::ArrowMarkingsConstPtr& arrow_markings) {
  for (size_t j = 0; j < arrow_markings->sub_messages.size(); j++) {
    for (size_t i = 0; i < safe_corridor.size(); i++) {
      Gate& gate = safe_corridor.at(i);
      Eigen::Vector3d arrow_marking_position;
      tf2::fromMsg(arrow_markings->sub_messages[j].pose.position, arrow_marking_position);
      double distance_to_arrow_marking =
          (gate.getTransformationToGateFrame() * arrow_marking_position).y();
      if (-0.2 <= distance_to_arrow_marking && distance_to_arrow_marking <= 0.05) {
        gate.setLeftPole(gate.getLaneCenter());
      }
    }
  }
}

void CollisionDetection::blockLeftLaneBeforeRoadClosure(Gate& gate,
                                                        double blockDistanceCarInRoadClosureArea,
                                                        double distanceToStart) {
  if (!drivePastNextRoadClosure &&
      distanceToStart - LENGTH_OF_LEFT_LANE_BLOCKED <= blockDistanceCarInRoadClosureArea &&
      distanceToStart > blockDistanceCarInRoadClosureArea) {
    // just change left pole if the pole is left from the middle
    if ((gate.getLeftPole() - gate.getRightPole()).norm() >
        (gate.getLaneCenter() - gate.getRightPole()).norm()) {
      gate.setLeftPole(gate.getLaneCenter());
    }
  }
}

common::EigenAlignedVector<Eigen::Vector3d> CollisionDetection::convertHullPolygon(
    const std::vector<geometry_msgs::Point>& hullPolygon) {
  common::EigenAlignedVector<Eigen::Vector3d> vectors(hullPolygon.size());
  for (size_t i = 0; i < hullPolygon.size(); ++i) {
    tf2::fromMsg(hullPolygon.at(i), vectors.at(i));
  }
  return vectors;
}

void CollisionDetection::extendRoadClosureSafetyMargin(
    common::EigenAlignedVector<Eigen::Vector3d>& hullPolygon, double safetyMargin) {
  Eigen::Vector3d v = hullPolygon[3] - hullPolygon[0];
  if (v.norm() == 0) {
    return;
  }
  v = safetyMargin * v.normalized();
  hullPolygon[0] -= v;
  hullPolygon[1] -= v;
  hullPolygon[2] += v;
  hullPolygon[3] += v;
  v = Eigen::Vector3d(-v.y(), v.x(), 0);
  hullPolygon[1] += v;
  hullPolygon[2] += v;
}

bool CollisionDetection::setDrivePastNextRoadClosure(
    navigation_msgs::DrivePastNextRoadClosureRequest& req,
    navigation_msgs::DrivePastNextRoadClosureResponse&) {
  drivePastNextRoadClosure = req.drive_past_next_road_closure;
  return true;
}

bool CollisionDetection::setRespectNoPassingZones(navigation_msgs::SetRespectNoPassingZonesRequest& req,
                                                  navigation_msgs::SetRespectNoPassingZonesResponse&
                                                  /*res*/) {
  respect_no_passing_zones = req.respect_no_passing_zones;
  return true;
}

void CollisionDetection::setNoPassingZones(const std::vector<NoPassingZone>& no_passing_zones) {
  this->no_passing_zones = no_passing_zones;
}
