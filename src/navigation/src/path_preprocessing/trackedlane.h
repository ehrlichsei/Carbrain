#ifndef TRACKEDLANE_H
#define TRACKEDLANE_H
#include <common/macros.h>

THIRD_PARTY_HEADERS_BEGIN
#include <vector>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <boost/function.hpp>
THIRD_PARTY_HEADERS_END

#include "boundingbox.hpp"
#include "common/types.h"

typedef boost::function<Eigen::Vector2d(Eigen::Vector2d)> PointTransformer;
typedef common::EigenAlignedVector<Eigen::Vector2d> PointList;

/**
 *
 * @brief The TrackedLane class represents the lane as seen by the car. All
 * contained points are approximately on the middle of the road. Additionaly,
 * previously stored lane points are kept until they reach a defined distance to
 * the bounding box of newly seen points.
 * The point positions are stored in world coordinates.
 */
class TrackedLane {
 public:
  TrackedLane(double tracking_distance = 1.2)
      : tracking_distance(tracking_distance) {}

  const common::EigenAlignedVector<Eigen::Vector3d>& getLanePoints() const;

  /**
   * @brief integrates a list of new points into the tracked lane.
   * The new points are added and all old points that are too far away are
   * discarded.
   * @param new_points a vector containing _all_ new seen points in world
   * coordinates.
   * @return the axis-aligned bounding box of all points of the tracked lane
   */
  BoundingBox integratePoints(const Eigen::Vector2d& vehicle_pos,
                              common::EigenAlignedVector<Eigen::Vector3d> new_points);

  /**
   * @brief setTrackingDistance sets the maximum distance of lane points to
   * remember.
   * @param tracking_distance the tracking distance.
   */
  void setTrackingDistance(double tracking_distance);

  void clearLanePoints();

 private:
  common::EigenAlignedVector<Eigen::Vector3d> lane_points;

  /**
   * @brief tracking_distance defines the distance that an old point can have to
   * the new points before it is deleted
   */
  double tracking_distance;
};

#endif  // TRACKEDLANE_H
