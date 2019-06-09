#include "driving_corridor_checker.h"
THIRD_PARTY_HEADERS_BEGIN
#include <boost/range/adaptor/filtered.hpp>
#include <boost/range/algorithm/min_element.hpp>
#include <boost/bind.hpp>
THIRD_PARTY_HEADERS_END
#include "common/best_score.h"
#include "common/make_vector.h"

using namespace navigation;

void DrivingCorridorChecker::handleFullCorridor(
    const navigation_msgs::DrivingCorridor::ConstPtr& full_corridor_msg) {
  corridor_ = DrivingCorridor::fromMessage(full_corridor_msg);
}

void DrivingCorridorChecker::handleObstacles(const navigation_msgs::Obstacles& obstacles_msg) {
  obstacles = navigation::Obstacle::fromMessage(obstacles_msg);
}

bool DrivingCorridorChecker::hasObstacleOnRightLane(const Eigen::Vector3d& start_point,
                                                    const double distance_of_interest,
                                                    double& distance_to_next_obstacle,
                                                    double& velocity_next_obstacle) {

  const auto relevant_obstacles = getObstaclesOnRightLaneAhead(start_point);
  if (relevant_obstacles.empty()) {
    return false;
  }
  const auto distObstToStart = [&start_point](const Obstacle& obstacle) {
    return obstacle.approximateProjectedDistance(start_point);
  };

  const auto nearest_obstacle = *common::min_score(relevant_obstacles, distObstToStart);

  if (distObstToStart(nearest_obstacle) > distance_of_interest) {
    return false;
  }
  distance_to_next_obstacle = distObstToStart(nearest_obstacle);
  velocity_next_obstacle = nearest_obstacle.getVelocity();
  return true;
}

void DrivingCorridorChecker::reset() {
  obstacles.clear();
  corridor_.clear();
}

std::vector<Obstacle> DrivingCorridorChecker::getObstaclesOnRightLaneAhead(const Eigen::Vector3d& start_point) {
  if (corridor_.empty()) {
    ROS_WARN_THROTTLE(1.0, "no gates in driving_corridor_checker");
    return {};
  }

  const Gate nearest_gate =
      *common::min_score(
          corridor_,
          [&](const Gate& g) { return (g.getCenter() - start_point).norm(); });

  const auto isAhead = [&nearest_gate](const Obstacle& obstacle) {
    return nearest_gate.getLaneLineSegment().isPointOnLeftSide(
        obstacle.getCenter().topRows<2>());
  };
  const auto isOnRightLane = [this](const Obstacle& obstacle) {
    return this->corridor_.isPointOnRightLane(obstacle.getCenter());
  };

  using boost::adaptors::filtered;
  return common::make_vector(obstacles | filtered(isAhead) | filtered(isOnRightLane));
}
