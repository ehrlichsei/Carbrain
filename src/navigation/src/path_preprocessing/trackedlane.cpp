#include "trackedlane.h"
#include "boundingbox.hpp"

const common::EigenAlignedVector<Eigen::Vector3d>& TrackedLane::getLanePoints() const {
  return lane_points;
}

BoundingBox TrackedLane::integratePoints(const Eigen::Vector2d& vehicle_pos,
                                         common::EigenAlignedVector<Eigen::Vector3d> new_points) {

  assert(new_points.size() > 1);
  BoundingBox new_points_bb(new_points);

  // enlarge the bounding box to the area where old points are kept
  BoundingBox old_points_bb(new_points_bb);
  old_points_bb.enlarge(vehicle_pos);
  old_points_bb.enlarge(tracking_distance);

  // TODO the bounding box is axis aligned in world coords. may produce
  // different results depending on orientation!

  new_points.reserve(lane_points.size() + new_points.size());
  for (const auto& lane_point : lane_points) {
    const Eigen::Vector2d old_point = lane_point.head<2>();
    if (old_points_bb.containsPoint(old_point) && !new_points_bb.containsPoint(old_point)) {
      new_points.push_back(lane_point);
    }
  }
  lane_points = std::move(new_points);
  return new_points_bb;
}

void TrackedLane::setTrackingDistance(double tracking_distance) {
  this->tracking_distance = tracking_distance;
}

void TrackedLane::clearLanePoints() { lane_points.clear(); }
