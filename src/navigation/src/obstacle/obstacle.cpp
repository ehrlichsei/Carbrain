#include "navigation/obstacle.h"
#include <common/macros.h>

THIRD_PARTY_HEADERS_BEGIN
#include <tf2_eigen/tf2_eigen.h>
#include <boost/range/algorithm/transform.hpp>
THIRD_PARTY_HEADERS_END

#include "common/tf2_eigen_addon.h"

namespace navigation {

Obstacle::Obstacle(const Eigen::Affine3d& pose,
                   const Eigen::Vector3d& scale,
                   const double velocity,
                   const double probability_dynamic)
    : pose_(pose),
      scale_(scale),
      velocity_(velocity),
      probability_dynamic_(probability_dynamic) {}

double Obstacle::approximateProjectedDistance(const Eigen::Vector3d& point) const {
  const double radius = 0.5 * scale_.topRows<2>().norm();
  return (pose_.translation() - point).topRows<2>().norm() - radius;
}

Eigen::Vector3d Obstacle::getScale() const { return scale_; }

Eigen::Vector3d Obstacle::getCenter() const { return pose_.translation(); }

Eigen::Vector2d Obstacle::getCenterProjection() const {
  return getCenter().topRows<2>();
}

std::vector<Eigen::Vector3d> Obstacle::getCorners() const {
  std::vector<Eigen::Vector3d> corners = {{1, 1, 0}, {-1, 1, 0}, {-1, -1, 0}, {1, -1, 0}};
  boost::transform(corners,
                   corners.begin(),
                   [&](const Eigen::Vector3d& x) {
                     return pose_ * x.cwiseProduct(0.5 * scale_);
                   });
  return corners;
}

std::vector<Eigen::Vector2d> Obstacle::getCornersProjection() const {
  const auto corners = getCorners();
  std::vector<Eigen::Vector2d> cornersProjected(corners.size());
  boost::transform(corners,
                   cornersProjected.begin(),
                   [&](const Eigen::Vector3d& x) { return x.topRows<2>(); });
  return cornersProjected;
}

void Obstacle::extendBy(double margin) {
  scale_(0) += 2 * margin;
  scale_(1) += 2 * margin;
}

void Obstacle::extendAllBy(ObstacleList& obstacles, double margin) {
  for (Obstacle& obstacle : obstacles) {
    obstacle.extendBy(margin);
  }
}

Obstacle Obstacle::fromMessage(const navigation_msgs::Obstacle& obstacle_msg) {
  Obstacle obstacle;
  tf2::fromMsg(obstacle_msg.pose, obstacle.pose_);
  tf2::fromMsg(obstacle_msg.scale, obstacle.scale_);
  obstacle.velocity_ = obstacle_msg.velocity;
  obstacle.probability_dynamic_ = obstacle_msg.is_dynamic;
  return obstacle;
}

Obstacle::ObstacleList Obstacle::fromMessage(const navigation_msgs::Obstacles& obstacles_msg) {
  ObstacleList obstacles;
  obstacles.reserve(obstacles_msg.sub_messages.size());
  for (const auto& obstacle_msg : obstacles_msg.sub_messages) {
    const Obstacle obstacle = fromMessage(obstacle_msg);
    obstacles.push_back(obstacle);
  }
  return obstacles;
}

Obstacle::ObstacleList Obstacle::fromMessage(const navigation_msgs::Obstacles::ConstPtr& obstacles_msg) {
  if (!obstacles_msg) {
    return ObstacleList();
  }
  return fromMessage(*obstacles_msg);
}

double Obstacle::getVelocity() const { return velocity_; }

double Obstacle::getProbabilityDynamic() const { return probability_dynamic_; }
}  // namespace navigation
