#include "navigation/driving_corridor.h"
#include <common/macros.h>

THIRD_PARTY_HEADERS_BEGIN
#include "navigation/triangle.h"
#include "ros/ros.h"

#include <boost/range/algorithm/unique_copy.hpp>
THIRD_PARTY_HEADERS_END

#include "common/eigen_utils.h"


DrivingCorridor::DrivingCorridor(Gate::GateList gates)
    : gates_(std::move(gates)) {}

Gate& DrivingCorridor::at(size_t index) { return gates_.at(index); }

const Gate& DrivingCorridor::at(size_t index) const { return gates_.at(index); }

size_t DrivingCorridor::size() const { return gates_.size(); }

bool DrivingCorridor::empty() const { return gates_.empty(); }

void DrivingCorridor::clear() { gates_.clear(); }

DrivingCorridor::Iterator DrivingCorridor::begin() { return gates_.begin(); }
DrivingCorridor::ConstIterator DrivingCorridor::begin() const {
  return gates_.begin();
}

DrivingCorridor::Iterator DrivingCorridor::end() { return gates_.end(); }
DrivingCorridor::ConstIterator DrivingCorridor::end() const {
  return gates_.end();
}

DrivingCorridor::ReverseIterator DrivingCorridor::rbegin() {
  return gates_.rbegin();
}

DrivingCorridor::ConstReverseIterator DrivingCorridor::rbegin() const {
  return gates_.rbegin();
}

DrivingCorridor::ReverseIterator DrivingCorridor::rend() {
  return gates_.rend();
}

DrivingCorridor::ConstReverseIterator DrivingCorridor::rend() const {
  return gates_.rend();
}

Gate& DrivingCorridor::back() { return gates_.back(); }
const Gate& DrivingCorridor::back() const { return gates_.back(); }

void DrivingCorridor::pop_back() { gates_.pop_back(); }

void DrivingCorridor::reserve(size_t new_cap) { gates_.reserve(new_cap); }

void DrivingCorridor::erase(const DrivingCorridor::ConstIterator& first,
                            const DrivingCorridor::ConstIterator& last) {
  gates_.erase(first, last);
}

void DrivingCorridor::push_back(const Gate& gate) { gates_.push_back(gate); }

bool DrivingCorridor::isPointApproximateContained(const Eigen::Vector3d& point) const {
  if (gates_.size() < 2) {
    return false;
  }
  if (gates_.size() == 2) {
    Gate gate = gates_.front();
    Gate next_gate = gates_.back();

    const Eigen::Affine3d world_to_gate_transformation =
        gate.getTransformationToGateFrame();
    gate.transform(world_to_gate_transformation);
    next_gate.transform(world_to_gate_transformation);
    const Eigen::Vector3d point_gate = world_to_gate_transformation * point;

    const double min_y = std::min(gate.getLeftPole().y(), gate.getRightPole().y());
    const double max_y =
        std::max(next_gate.getLeftPole().y(), next_gate.getRightPole().y());

    const Eigen::Vector2d min(gate.getLeftPole().x(), min_y);
    const Eigen::Vector2d max(gate.getRightPole().x(), max_y);
    return Eigen::AlignedBox2d(min, max).contains(to2D(point_gate));
  }
  for (size_t i = 1; i < gates_.size() - 1; i++) {
    Gate gate = gates_.at(i);
    Gate previous_gate = gates_.at(i - 1);
    Gate next_gate = gates_.at(i + 1);

    const Eigen::Affine3d world_to_gate_transformation =
        gate.getTransformationToGateFrame();
    gate.transform(world_to_gate_transformation);
    previous_gate.transform(world_to_gate_transformation);
    next_gate.transform(world_to_gate_transformation);
    const Eigen::Vector3d point_gate = world_to_gate_transformation * point;

    const double min_y =
        std::min(previous_gate.getLeftPole().y(), previous_gate.getRightPole().y());
    const double max_y =
        std::max(next_gate.getLeftPole().y(), next_gate.getRightPole().y());

    const Eigen::Vector2d min(gate.getLeftPole().x(), min_y);
    const Eigen::Vector2d max(gate.getRightPole().x(), max_y);
    if (Eigen::AlignedBox2d(min, max).contains(to2D(point_gate))) {
      return true;
    }
  }
  return false;
}

bool DrivingCorridor::isPointContained(const Eigen::Vector3d& point) const {
  for (size_t i = 1; i < gates_.size(); i++) {
    Gate gate = gates_.at(i);
    Gate previous_gate = gates_.at(i - 1);

    /*   2--1
     *   | /
     *   3
     */
    const Triangle left_triangle(gate.getLeftPoleProjection(),
                                 previous_gate.getLeftPoleProjection(),
                                 previous_gate.getRightPoleProjection());
    if (!left_triangle.isPointOutside(point.topRows<2>())) {
      return true;
    }

    /*      1
     *     /|
     *   2--3
     */
    const Triangle right_triangle(gate.getLeftPoleProjection(),
                                  previous_gate.getRightPoleProjection(),
                                  gate.getRightPoleProjection());
    if (!right_triangle.isPointOutside(point.topRows<2>())) {
      return true;
    }
  }
  return false;
}

template <Gate::Pole pole_side>
double minimalDistance(const Gate::GateList& gates, const Eigen::Vector3d& point) {
  double min_distance = std::numeric_limits<double>::max();
  for (const Gate& gate : gates) {
    const double distance = (point - getPole<pole_side>(gate)).norm();
    min_distance = std::min(distance, min_distance);
  }
  return min_distance;
}

bool DrivingCorridor::isPointOnRightLane(const Eigen::Vector3d& point) const {
  const double min_distance_left = minimalDistance<Gate::LEFT>(gates_, point);
  const double min_distance_right = minimalDistance<Gate::RIGHT>(gates_, point);

  return min_distance_right < min_distance_left;
}

class XCoordinateWithTransformationComparator {

 public:
  explicit XCoordinateWithTransformationComparator(const Eigen::Affine3d& transform)
      : transform_(transform) {}

  bool operator()(const Gate& gateA, const Gate& gateB) {
    const Eigen::Vector3d& pointA = transform_ * gateA.getCenter();
    const Eigen::Vector3d& pointB = transform_ * gateB.getCenter();
    return pointA[0] < pointB[0];
  }

 private:
  const Eigen::Affine3d transform_;
};

class GateComparator {

 public:
  explicit GateComparator(const Eigen::Affine3d& transform)
      : v_(transform.rotation().transpose() * Eigen::Vector3d::UnitX()) {}

  bool operator()(const Gate& gateA, const Gate& gateB) {
    const double pointA = v_.dot(gateA.getCenter());
    const double pointB = v_.dot(gateB.getCenter());
    if (pointA < 0 && pointB > 0) {
      // order obvious from path coordinate system
      return true;
    }
    if (pointA > 0 && pointB < 0) {
      // order obvious from path coordinate system
      return false;
    }
    const bool AsmallerB = gateA < gateB;
    if (!AsmallerB && !(gateA > gateB)) {
      // conflict solved by path coordinate system
      return pointA < pointB;
    }
    return AsmallerB;
  }

 private:
  const Eigen::Vector3d v_;
};

void DrivingCorridor::sort(const Eigen::Affine3d& transform) {
  std::sort(gates_.begin(), gates_.end(), GateComparator(transform));
}

DrivingCorridor DrivingCorridor::simplified(const double min_distance) const {
  DrivingCorridor simplified_corridor;

  if (gates_.empty()) {
    return simplified_corridor;
  }

  simplified_corridor.gates_.reserve(gates_.size());

  // remove gates too close to there neighbors
  const auto tooClose = [min_distance](auto& a, auto& b) {
    return a.getLineSegment().getDistanceToPoint(b.getCenterProjection()) < min_distance;
  };

  boost::unique_copy(gates_, std::back_inserter(simplified_corridor.gates_), tooClose);

  // avoid intersections between gates
  int intersections = 1;
  for (unsigned int c = 0; intersections != 0 && c < simplified_corridor.size(); c++) {
    intersections = 0;
    for (unsigned int i = 0; i < simplified_corridor.size() - 1; i++) {
      Gate& gate_i = simplified_corridor.at(i);
      const auto line_i = gate_i.getLaneLineSegment();
      for (unsigned int j = i + 1; j < simplified_corridor.size(); j++) {
        Gate& gate_j = simplified_corridor.at(j);
        if (!line_i.intersects(gate_j.getLineSegment())) {
          break;
        }
        intersections++;
        // swap one end point, ensure gate order
        if (line_i.isPointOnRightSide(gate_j.getRightPoleProjection())) {
          Eigen::Vector3d temp_pole = gate_i.getRightPole();
          Eigen::Vector3d temp_lane_boundary = gate_i.getRightLaneBoundary();
          gate_i.setRightPole(gate_j.getRightPole());
          gate_i.setRightLaneBoundary(gate_j.getRightLaneBoundary());
          gate_j.setRightPole(temp_pole);
          gate_j.setRightLaneBoundary(temp_lane_boundary);
        } else {
          Eigen::Vector3d temp_pole = gate_i.getLeftPole();
          Eigen::Vector3d temp_lane_boundary = gate_i.getLeftLaneBoundary();
          gate_i.setLeftPole(gate_j.getLeftPole());
          gate_i.setLeftLaneBoundary(gate_j.getLeftLaneBoundary());
          gate_j.setLeftPole(temp_pole);
          gate_j.setLeftLaneBoundary(temp_lane_boundary);
        }
      }
    }
  }

  return simplified_corridor;
}

DrivingCorridor DrivingCorridor::fromMessage(const navigation_msgs::DrivingCorridor::ConstPtr& driving_corridor_msg) {
  DrivingCorridor driving_corridor;
  if (!driving_corridor_msg) {
    return driving_corridor;
  }
  driving_corridor.gates_ = Gate::fromMessages(driving_corridor_msg->gates);
  return driving_corridor;
}

navigation_msgs::DrivingCorridor DrivingCorridor::toMessage() const {
  navigation_msgs::DrivingCorridor driving_corridor_msg;
  driving_corridor_msg.gates.reserve(gates_.size());
  for (const Gate& gate : gates_) {
    driving_corridor_msg.gates.emplace_back(gate.toMessage());
  }
  return driving_corridor_msg;
}
