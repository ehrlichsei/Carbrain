#include "gate_utils.h"

#include "common/polynomial_utils.h"
#include "common/eigen_utils.h"

THIRD_PARTY_HEADERS_BEGIN
#include <boost/algorithm/clamp.hpp>
#include <cmath>
#include <Eigen/Eigen>
THIRD_PARTY_HEADERS_END

using namespace Eigen;

GateUtils::Intersection GateUtils::getIntersection(const Gate& gate,
                                                   const LaneUtils::Lane& path) {
  Intersection intersection;
  if (path.empty()) {
    return intersection;
  }
  Eigen::Vector3d const* point_before_gate = &path.back();
  float dist_before_gate = std::numeric_limits<int>::max();
  Eigen::Vector3d const* point_after_gate = &path.front();
  float dist_after_gate = std::numeric_limits<int>::max();

  const auto gate_line = gate.getLineSegment();
  // check if gate is outside path
  if (gate_line.isPointOnRightSide(point_before_gate->topRows<2>()) ||
      gate_line.isPointOnLeftSide(point_after_gate->topRows<2>())) {
    return intersection;
  }
  const auto line = gate_line.getLine();
  // calculate points next to gate
  for (const Eigen::Vector3d& point : path) {
    const float dist = signedDistance(line, point.topRows<2>());
    const float abs_dist = std::abs(dist);
    if (dist < 0) {
      if (abs_dist < dist_before_gate) {
        point_before_gate = &point;
        dist_before_gate = abs_dist;
      }
    } else {
      // distance should not be zero for not matching the same point as before
      if (0 < abs_dist && abs_dist < dist_after_gate) {
        point_after_gate = &point;
        dist_after_gate = abs_dist;
      }
    }
  }

  Eigen::ParametrizedLine<double, 3> path_segment(
      *point_before_gate, *point_after_gate - *point_before_gate);
  Eigen::ParametrizedLine<double, 3> gate_segment(
      gate.getLeftLaneBoundary(), gate.getRightLaneBoundary() - gate.getLeftLaneBoundary());
  float param =
      gate_segment.intersectionParameter(Eigen::Hyperplane<double, 3>(path_segment));
  intersection.point = gate.toPoint(param);
  intersection.direction = *point_after_gate - *point_before_gate;
  intersection.has_intersection = true;
  return intersection;
}

bool GateUtils::hasIntersection(const Gate& gate, const LaneUtils::Lanes& lanes) {
  return GateUtils::getIntersection(gate, lanes.left).has_intersection ||
         GateUtils::getIntersection(gate, lanes.middle).has_intersection ||
         GateUtils::getIntersection(gate, lanes.right).has_intersection;
}

bool GateUtils::adaptGateToLane(Gate* gate,
                                Vector3d* lane_direction,
                                double* current_lane_width_left,
                                double* current_lane_width_right,
                                const double /*param_lane_width*/,
                                const double distance_to_vehicle,
                                const double weight_middle_intersection,
                                const double weight_own_intersection,
                                const double weight_opposite_intersection,
                                const LaneUtils::Lanes& old_lanes,
                                const LaneUtils::Lanes& lanes) {

  GateUtils::Intersections tracked{
      .left = GateUtils::getIntersection(*gate, old_lanes.left),
      .middle = GateUtils::getIntersection(*gate, old_lanes.middle),
      .right = GateUtils::getIntersection(*gate, old_lanes.right)};

  GateUtils::Intersections percepted{
      .left = GateUtils::getIntersection(*gate, lanes.left),
      .middle = GateUtils::getIntersection(*gate, lanes.middle),
      .right = GateUtils::getIntersection(*gate, lanes.right)};

  Vector3d sum_direction = Vector3d(0, 0, 0);
  if (percepted.left.has_intersection || percepted.right.has_intersection ||
      percepted.middle.has_intersection || tracked.right.has_intersection ||
      tracked.left.has_intersection) {
    Vector3d initial_gate_direction =
        gate->getRightLaneBoundary() - gate->getLeftLaneBoundary();
    initial_gate_direction.normalize();

    // predict gate poles based on old gates

    sum_direction += adaptGateToTrackedLane(
        gate,
        current_lane_width_left,
        current_lane_width_right,
        std::min(*current_lane_width_left, *current_lane_width_right),
        std::max(*current_lane_width_left, *current_lane_width_right),
        &tracked);

    // filter gate poles based on percepted lane path
    sum_direction += adaptGatePerceptedLane(
        gate,
        current_lane_width_left,
        current_lane_width_right,
        std::min(*current_lane_width_left, *current_lane_width_right),
        std::max(*current_lane_width_left, *current_lane_width_right),
        weight_middle_intersection,
        weight_own_intersection,
        weight_opposite_intersection,
        distance_to_vehicle,
        initial_gate_direction,
        tracked,
        &percepted);
    if (sum_direction.norm() > 1e-5) {
      *lane_direction = sum_direction.normalized();
    }
    return true;
  } else {
    return false;
  }
}

Eigen::Vector3d GateUtils::adaptGateToTrackedLane(Gate* gate,
                                                  double* current_lane_width_left,
                                                  double* current_lane_width_right,
                                                  const double /*min_lane_width*/,
                                                  const double /*max_lane_width*/,
                                                  GateUtils::Intersections* tracked) {
  Eigen::Vector3d sum_direction = Eigen::Vector3d(0, 0, 0);
  calculateLaneWidth(current_lane_width_left,
                     current_lane_width_right,
                     std::min(*current_lane_width_left, *current_lane_width_right),
                     std::max(*current_lane_width_left, *current_lane_width_right),
                     *tracked);
  if (tracked->right.has_intersection) {
    sum_direction += tracked->right.direction;
    gate->setRightLaneBoundary(tracked->right.point);
  }
  if (tracked->left.has_intersection) {
    sum_direction += tracked->left.direction;
    gate->setLeftLaneBoundary(tracked->left.point);
  }
  if (tracked->middle.has_intersection) {
    sum_direction += tracked->middle.direction;
    gate->setLaneCenter(tracked->middle.point);
  }
  return sum_direction;
}

Eigen::Vector3d GateUtils::adaptGatePerceptedLane(Gate* gate,
                                                  double* current_lane_width_left,
                                                  double* current_lane_width_right,
                                                  const double /*min_lane_width*/,
                                                  const double /*max_lane_width*/,
                                                  const double weight_middle_intersection,
                                                  const double weight_own_intersection,
                                                  const double weight_opposite_intersection,
                                                  const double distance_to_vehicle,
                                                  const Eigen::Vector3d& initial_gate_direction,
                                                  const GateUtils::Intersections& tracked,
                                                  GateUtils::Intersections* percepted) {
  calculateLaneWidth(current_lane_width_left,
                     current_lane_width_right,
                     std::min(*current_lane_width_left, *current_lane_width_right),
                     std::max(*current_lane_width_left, *current_lane_width_right),
                     *percepted);
  Eigen::Vector3d sum_direction = Eigen::Vector3d(0, 0, 0);
  Vector3d sum_left = Vector3d::Zero();
  Vector3d sum_right = Vector3d::Zero();
  double norm_left = 0, norm_right = 0;
  if (percepted->right.has_intersection) {
    sum_left += (percepted->right.point -
                 ((*current_lane_width_right) + (*current_lane_width_left)) * initial_gate_direction) *
                weight_opposite_intersection;
    norm_left += weight_opposite_intersection;
    sum_right += percepted->right.point * weight_own_intersection;
    norm_right += weight_own_intersection;
    sum_direction += percepted->right.direction;
  }
  if (percepted->left.has_intersection) {
    sum_left += percepted->left.point * weight_own_intersection;
    norm_left += weight_own_intersection;
    sum_right += (percepted->left.point +
                  ((*current_lane_width_right) + (*current_lane_width_left)) * initial_gate_direction) *
                 weight_opposite_intersection;
    norm_right += weight_opposite_intersection;
    sum_direction += percepted->left.direction;
  }
  if (percepted->middle.has_intersection) {
    sum_left += (percepted->middle.point - (*current_lane_width_left) * initial_gate_direction) *
                weight_middle_intersection;
    norm_left += weight_middle_intersection;
    sum_right += (percepted->middle.point + (*current_lane_width_right) * initial_gate_direction) *
                 weight_middle_intersection;
    norm_right += weight_middle_intersection;
    adjustLaneBoundary<Gate::MIDDLE_LANE>(
        gate, !tracked.middle.has_intersection, percepted->middle.point, distance_to_vehicle);
    sum_direction += percepted->middle.direction;
  }
  if (percepted->left.has_intersection || percepted->middle.has_intersection ||
      percepted->right.has_intersection) {
    Vector3d new_left = sum_left / norm_left;
    adjustLaneBoundary<Gate::LEFT_LANE>(
        gate, !tracked.left.has_intersection, new_left, distance_to_vehicle);
    Vector3d new_right = sum_right / norm_right;
    adjustLaneBoundary<Gate::RIGHT_LANE>(
        gate, !tracked.right.has_intersection, new_right, distance_to_vehicle);
  }
  return sum_direction;
}

void GateUtils::calculateLaneWidth(double* lane_width_left,
                                   double* lane_width_right,
                                   const double min_lane_width,
                                   const double max_lane_width,
                                   const GateUtils::Intersections& intersections) {
  bool intersection_left = false;
  if (intersections.left.has_intersection && intersections.middle.has_intersection) {
    intersection_left = true;
    *lane_width_left = (intersections.middle.point - intersections.left.point).norm();
  }
  bool intersection_right = false;
  if (intersections.right.has_intersection && intersections.middle.has_intersection) {
    *lane_width_right = (intersections.middle.point - intersections.right.point).norm();
    intersection_right = true;
  }
  if (intersection_left && intersection_right) {
    return;
  } else if (intersection_left && !intersection_right) {
    *lane_width_right = (*lane_width_left + *lane_width_right) / 2.0;
  } else if (!intersection_left && intersection_right) {
    *lane_width_left = (*lane_width_right + *lane_width_left) / 2.0;
  } else {
    if (intersections.right.has_intersection && intersections.left.has_intersection) {
      *lane_width_right = ((intersections.left.point - intersections.right.point).norm() +
                           *lane_width_right + *lane_width_left) /
                          4.0;
      *lane_width_left = *lane_width_right;
    }
  }
  *lane_width_left =
      boost::algorithm::clamp(*lane_width_left, min_lane_width, max_lane_width);
  *lane_width_right =
      boost::algorithm::clamp(*lane_width_right, min_lane_width, max_lane_width);
}

Gate GateUtils::fromPolynomial(const common::DynamicPolynomial& lane_polynomial,
                               const double x,
                               const Eigen::Affine3d& path_to_world_transform,
                               const double lane_width,
                               const unsigned int id) {
  const Eigen::Vector3d polynomial_point = to3D(common::point(lane_polynomial, x));
  const Eigen::Vector3d polynomial_normal = to3D(common::normal(lane_polynomial, x));
  const Eigen::Vector3d left_pole =
      path_to_world_transform * (polynomial_point + lane_width * polynomial_normal);
  const Eigen::Vector3d right_pole =
      path_to_world_transform * (polynomial_point - lane_width * polynomial_normal);
  return Gate(id, 0.0, 1.0, left_pole, right_pole, 0.5);
}

Gate GateUtils::fromGate(const Gate& gate,
                         const Vector3d& direction,
                         const double distance,
                         const double lane_width,
                         const unsigned int id) {
  const Eigen::Vector3d gate_direction = direction.unitOrthogonal();
  const Eigen::Vector3d left =
      gate.getCenter() + direction * distance + gate_direction * lane_width;
  const Eigen::Vector3d right =
      gate.getCenter() + direction * distance - gate_direction * lane_width;
  return Gate(id, 0.0, 1.0, left, right, 0.5);
}

Eigen::Vector3d GateUtils::calculateNewDirection(const Gate::GateList& gates,
                                                 const size_t count) {
  assert(count >= 1);
  Vector3d path_direction = Eigen::Vector3d(0, 0, 0);
  std::for_each(gates.end() - std::min(static_cast<size_t>(count), gates.size()),
                gates.end(),
                [&path_direction](const Gate& gate) {
                  path_direction += gate.getVectorLeftToRight().unitOrthogonal();
                });
  if (path_direction.norm() < 1e-5) {
    return Eigen::Vector3d{0, 0, 0};
  }
  return path_direction.normalized();
}
