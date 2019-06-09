#include "navigation/gate.h"
#include <common/macros.h>

THIRD_PARTY_HEADERS_BEGIN
#include <tf2/convert.h>
#include <tf2_eigen/tf2_eigen.h>
#include <boost/algorithm/clamp.hpp>
#include <boost/optional.hpp>
THIRD_PARTY_HEADERS_END


Gate::Gate(int id, const Eigen::Vector3d& left_lane_boundary, const Eigen::Vector3d& right_lane_boundary)
    : left_lane_boundary_(left_lane_boundary),
      right_lane_boundary_(right_lane_boundary),
      id_(id) {}

Gate::Gate(int id,
           double left_pole,
           double right_pole,
           const Eigen::Vector3d& left_lane_boundary,
           const Eigen::Vector3d& right_lane_boundary,
           double lane_center_point)
    : left_lane_boundary_(left_lane_boundary),
      right_lane_boundary_(right_lane_boundary),
      id_(id),
      lane_center_point_(lane_center_point),
      left_pole_(left_pole),
      right_pole_(right_pole) {}

int Gate::getId() const { return id_; }

void Gate::setId(const int new_id) { id_ = new_id; }

Eigen::Vector3d Gate::getLeftPole() const { return toPoint(left_pole_); }

Eigen::Vector3d Gate::getRightPole() const { return toPoint(right_pole_); }

Eigen::Vector3d Gate::getPreferedPathPoint() const {
  return toPoint(prefered_path_point_);
}

double Gate::getPreferedPathWeight() const { return prefered_path_weight_; }

Eigen::Vector3d Gate::getCenter() const {
  return toPoint((left_pole_ + right_pole_) / 2.0);
}

Eigen::Vector3d Gate::toPoint(double param) const {
  return left_lane_boundary_ + param * (right_lane_boundary_ - left_lane_boundary_);
}

double Gate::toParam(const Eigen::Vector3d& point) const {
  return boost::algorithm::clamp(
      getLaneLineSegment().getProjectionOfPoint(point.topRows<2>()), 0.0, 1.0);
}

Eigen::Vector3d Gate::getLaneCenter() const {
  return toPoint(lane_center_point_);
}

double Gate::getWidth() const {
  return (right_pole_ - left_pole_) * (right_lane_boundary_ - left_lane_boundary_).norm();
}

bool Gate::isDegenerated() const {
  const double epsilon = 1e-10;
  return getWidth() <= epsilon;
}

bool Gate::contains(const double param) const {
  return left_pole_ <= param && param <= right_pole_;
}

Eigen::Affine3d Gate::getTransformationFromGateFrame() const {
  return Eigen::Translation3d(toPoint(left_pole_)) * getRotation();
}

Eigen::Affine3d Gate::getTransformationToGateFrame() const {
  return getTransformationFromGateFrame().inverse();
}

Eigen::Quaterniond Gate::getRotation() const {

  Eigen::Quaterniond rotation;
  rotation.setFromTwoVectors(Eigen::Vector3d::UnitX(), getVectorLeftToRight());
  return rotation;
}

Eigen::Vector3d Gate::getVectorLeftToRight() const {
  return right_lane_boundary_ - left_lane_boundary_;
}

void Gate::setLeftPole(const Eigen::Vector3d& left_pole) {
  left_pole_ = boost::algorithm::clamp(toParam(left_pole), 0.0, right_pole_);
}

void Gate::setRightPole(const Eigen::Vector3d& right_pole) {
  right_pole_ = boost::algorithm::clamp(toParam(right_pole), left_pole_, 1.0);
}

void Gate::setLeftLaneBoundary(const Eigen::Vector3d& left_lane_boundary) {
  left_lane_boundary_ = left_lane_boundary;
}

void Gate::setRightLaneBoundary(const Eigen::Vector3d& right_lane_boundary) {
  right_lane_boundary_ = right_lane_boundary;
}

void Gate::setLaneCenter(const Eigen::Vector3d& lane_center_point) {
  lane_center_point_ = toParam(lane_center_point);
}

void Gate::setPreferedPathPoint(const Eigen::Vector3d& prefered_path_point) {
  prefered_path_point_ = toParam(prefered_path_point);
}

void Gate::setPreferedPathPoint(double prefered_path_point) {
  prefered_path_point_ = prefered_path_point;
}

void Gate::setPreferedPathWeight(double prefered_path_weight) {
  prefered_path_weight_ = prefered_path_weight;
}

// TODO rename to GateCenter
void Gate::collapseToCenter() {
  const double center = (left_pole_ + right_pole_) / 2.0;
  left_pole_ = center;
  right_pole_ = center;
}

void Gate::shrinkBy(double shrink_distance) {
  if (shrink_distance > getWidth() || isDegenerated()) {
    collapseToCenter();
    return;
  }

  shrink_distance =
      shrink_distance / (right_lane_boundary_ - left_lane_boundary_).norm();
  const double clearance = 0.5 * shrink_distance;
  left_pole_ += clearance;
  right_pole_ -= clearance;
}

void Gate::shrinkFromLeftBy(double shrink_distance) {
  if (shrink_distance > getWidth() || isDegenerated()) {
    left_pole_ = right_pole_;
    return;
  }

  shrink_distance =
      shrink_distance / (right_lane_boundary_ - left_lane_boundary_).norm();
  left_pole_ += shrink_distance;
}

void Gate::shrinkFromRightBy(double shrink_distance) {
  if (shrink_distance > getWidth() || isDegenerated()) {
    right_pole_ = left_pole_;
    return;
  }

  shrink_distance =
      shrink_distance / (right_lane_boundary_ - left_lane_boundary_).norm();
  right_pole_ -= shrink_distance;
}

// TODO needed?
// Edit (felix.hartenbach): Used for generating test tracks
void Gate::transform(const Eigen::Affine3d& transformation) {
  left_lane_boundary_ = transformation * left_lane_boundary_;
  right_lane_boundary_ = transformation * right_lane_boundary_;
}

Gate Gate::fromMessage(const navigation_msgs::Gate& gate_msg) {
  const int id = gate_msg.id;
  const double lane_center_point = gate_msg.lane_center_point;
  const double left_pole = gate_msg.left_pole;
  const double right_pole = gate_msg.right_pole;
  const double prefered_path_point = gate_msg.prefered_path_point;
  const double prefered_path_weight = gate_msg.prefered_path_weight;
  Eigen::Vector3d left_lane_boundary;
  Eigen::Vector3d right_lane_boundary;
  tf2::fromMsg(gate_msg.left_lane_boundary, left_lane_boundary);
  tf2::fromMsg(gate_msg.right_lane_boundary, right_lane_boundary);
  Gate gate(id, left_pole, right_pole, left_lane_boundary, right_lane_boundary, lane_center_point);
  gate.setPreferedPathPoint(prefered_path_point);
  gate.setPreferedPathWeight(prefered_path_weight);
  return gate;
}

navigation_msgs::Gate Gate::toMessage() const {
  navigation_msgs::Gate gate_msg;
  gate_msg.id = id_;
  gate_msg.left_lane_boundary = tf2::toMsg(left_lane_boundary_);
  gate_msg.right_lane_boundary = tf2::toMsg(right_lane_boundary_);
  gate_msg.lane_center_point = lane_center_point_;
  gate_msg.left_pole = left_pole_;
  gate_msg.right_pole = right_pole_;
  gate_msg.prefered_path_point = prefered_path_point_;
  gate_msg.prefered_path_weight = prefered_path_weight_;
  return gate_msg;
}

LineSegment Gate::getLineSegment() const {
  return LineSegment(getLeftPoleProjection(), getRightPoleProjection());
}

LineSegment Gate::getLaneLineSegment() const {
  return LineSegment(getLeftLaneBoundaryProjection(), getRightLaneBoundaryProjection());
}

bool Gate::isPointBehind(Eigen::Vector3d point) const {
  return getLaneLineSegment().isPointOnLeftSide(point.topRows<2>());
}

bool Gate::isPointInFront(Eigen::Vector3d point) const {
  return getLaneLineSegment().isPointOnRightSide(point.topRows<2>());
}

bool operator<(const Gate& first_gate, const Gate& second_gate) {
  return first_gate.getLineSegment().isPointOnLeftSide(second_gate.getCenterProjection()) &&
         second_gate.getLineSegment().isPointOnRightSide(first_gate.getCenterProjection());
}
bool operator>(const Gate& first_gate, const Gate& second_gate) {
  return second_gate < first_gate;
}
