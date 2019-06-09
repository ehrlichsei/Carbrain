#ifndef NAVIGATION_OBSTACLE_H
#define NAVIGATION_OBSTACLE_H
#include <common/macros.h>

THIRD_PARTY_HEADERS_BEGIN
#include <vector>
#include "navigation_msgs/Obstacles.h"
#include <Eigen/Geometry>
THIRD_PARTY_HEADERS_END

#include "common/types.h"

namespace navigation {

class Obstacle {
 public:
  typedef common::EigenAlignedVector<Obstacle> ObstacleList;

  Obstacle() = default;

  Obstacle(const Eigen::Affine3d& pose,
           const Eigen::Vector3d& scale,
           const double velocity,
           const double probability_dynamic);

  double approximateProjectedDistance(const Eigen::Vector3d& point) const;

  Eigen::Vector3d getScale() const;

  Eigen::Vector3d getCenter() const;

  Eigen::Vector2d getCenterProjection() const;

  std::vector<Eigen::Vector3d> getCorners() const;

  std::vector<Eigen::Vector2d> getCornersProjection() const;

  void extendBy(double margin);

  static void extendAllBy(ObstacleList& obstacles, double margin);

  static Obstacle fromMessage(const navigation_msgs::Obstacle& obstacle_msg);

  static ObstacleList fromMessage(const navigation_msgs::Obstacles& obstacles_msg);
  static ObstacleList fromMessage(const navigation_msgs::Obstacles::ConstPtr& obstacles_msg);

  double getVelocity() const;

  double getProbabilityDynamic() const;

private:
  Eigen::Affine3d pose_;
  Eigen::Vector3d scale_;
  double velocity_;
  double probability_dynamic_;
};
}  // namespace navigation
#endif  // NAVIGATION_OBSTACLE_H
