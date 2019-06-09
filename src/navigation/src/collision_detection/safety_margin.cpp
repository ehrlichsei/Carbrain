#include "safety_margin.h"
#include <common/macros.h>

THIRD_PARTY_HEADERS_BEGIN
#include <boost/range/iterator_range.hpp>
#include <boost/range/algorithm/adjacent_find.hpp>
#include <boost/range/algorithm/reverse.hpp>

#include "navigation_msgs/Obstacle.h"
#include <ros/ros.h>
#include <eigen_conversions/eigen_msg.h>
THIRD_PARTY_HEADERS_END

const ParameterString<double> PARAM_VEHICLE_WIDTH("vehicle_width");
const ParameterString<double> PARAM_DISTANCE_TO_FRONT_BUMPER(
    "distance_to_front_bumper");
const ParameterString<double> PARAM_DISTANCE_TO_REAR_BUMPER(
    "distance_to_rear_bumper");
const ParameterString<double> PARAM_VEHICLE_SAFETY_REAR_MARGIN(
    "vehicle_safety_rear_margin");
const ParameterString<double> PARAM_VEHICLE_SAFETY_FRONT_MARGIN(
    "vehicle_safety_front_margin");
const ParameterString<double> PARAM_VEHICLE_SAFETY_WIDTH_MARGIN(
    "vehicle_safety_width_margin");
const ParameterString<double> PARAM_GATE_SAFETY_MARGIN("gate_safety_margin");
const ParameterString<double> PARAM_LARGE_SHRINK_DISTANCE_FOR_WARNINGS(
    "large_shrink_distance_for_warnings");

SafetyMargin::SafetyMargin(ParameterInterface* parameters)
    : parameters_ptr_(parameters) {
  parameters->registerParam(PARAM_VEHICLE_WIDTH);
  parameters->registerParam(PARAM_DISTANCE_TO_FRONT_BUMPER);
  parameters->registerParam(PARAM_DISTANCE_TO_REAR_BUMPER);
  parameters->registerParam(PARAM_VEHICLE_SAFETY_REAR_MARGIN);
  parameters->registerParam(PARAM_VEHICLE_SAFETY_FRONT_MARGIN);
  parameters->registerParam(PARAM_VEHICLE_SAFETY_WIDTH_MARGIN);
  parameters->registerParam(PARAM_GATE_SAFETY_MARGIN);
  parameters->registerParam(PARAM_LARGE_SHRINK_DISTANCE_FOR_WARNINGS);
}


template <Gate::Pole pole_type, typename Range>
double shrinkDistanceToFitBumper(const Range& range, const LineSegment& bumper) {
  Eigen::Vector2d intersection_point;
  const auto it = boost::range::adjacent_find(
      range,
      [&](const auto& a, const auto& b) {
        const LineSegment segment(getPoleProjection<pole_type>(a),
                                  getPoleProjection<pole_type>(b));
        return bumper.intersectsRay(segment, &intersection_point);
      });
  return it == range.end() ? 0.0 : (bumper.getStartPoint() - intersection_point).norm();
}

template <Gate::Pole pole_type, typename Range>
double shrinkDistanceToFitSide(const Range& range, const LineSegment side) {
  double largest_violation_side = 0.0;
  for (const Gate& gate : range) {
    const Eigen::Vector2d safe_pole = getPoleProjection<pole_type>(gate);
    Eigen::Vector2d projected_point;
    if (side.containsProjectedPoint(safe_pole, &projected_point) &&
        isPointOnSide<Gate::other(pole_type)>(side, safe_pole)) {
      const double required_shrink_distance = (safe_pole - projected_point).norm();
      largest_violation_side = std::max(required_shrink_distance, largest_violation_side);
    } else {
      break;
    }
  }
  return largest_violation_side;
}

Vehicle SafetyMargin::buildVehicleWithParams() {
  // car specification
  const double vehicle_width = parameters_ptr_->getParam(PARAM_VEHICLE_WIDTH);
  const double distance_to_front_bumper =
      parameters_ptr_->getParam(PARAM_DISTANCE_TO_FRONT_BUMPER);
  const double distance_to_rear_bumper =
      parameters_ptr_->getParam(PARAM_DISTANCE_TO_REAR_BUMPER);

  // car corridor parameter
  const double vehicle_safety_rear_margin =
      parameters_ptr_->getParam(PARAM_VEHICLE_SAFETY_REAR_MARGIN);
  const double vehicle_safety_front_margin =
      parameters_ptr_->getParam(PARAM_VEHICLE_SAFETY_FRONT_MARGIN);
  const double vehicle_safety_width_margin =
      parameters_ptr_->getParam(PARAM_VEHICLE_SAFETY_WIDTH_MARGIN);

  return Vehicle(vehicle_width + 2 * vehicle_safety_width_margin,
                 distance_to_rear_bumper + vehicle_safety_rear_margin,
                 distance_to_front_bumper + vehicle_safety_front_margin);
}

void SafetyMargin::generateWarningsShrinkDistance(double largest_violation_left_side,
                                                  double largest_violation_right_side,
                                                  double gateWidth,
                                                  double large_shrink_distance) const {

  if (largest_violation_left_side >= large_shrink_distance) {
    ROS_WARN_THROTTLE(1, "Large shrink distance required to fit left side: %f", largest_violation_left_side);
  }
  if (largest_violation_left_side > gateWidth) {
    ROS_WARN_THROTTLE(
        1, "Corridor too small to fit left side. Required shrink distance: %f", largest_violation_left_side);
  }
  if (largest_violation_right_side >= large_shrink_distance) {
    ROS_WARN_THROTTLE(1, "Large shrink distance required to fit right side: %f", largest_violation_right_side);
  }
  if (largest_violation_right_side > gateWidth) {
    ROS_WARN_THROTTLE(
        1, "Corridor too small to fit right side. Required shrink distance: %f", largest_violation_right_side);
  }
}

void SafetyMargin::applySafetyMargin(const DrivingCorridor& safe_corridor,
                                     DrivingCorridor& car_corridor) {
  const double gate_safety_margin = parameters_ptr_->getParam(PARAM_GATE_SAFETY_MARGIN);
  const double large_shrink_distance =
      parameters_ptr_->getParam(PARAM_LARGE_SHRINK_DISTANCE_FOR_WARNINGS);

  const Eigen::AngleAxisd rotation(0.5 * M_PI, Eigen::Vector3d::UnitZ());
  const Vehicle vehicle = buildVehicleWithParams();
  const Vehicle vehicle_gate_frame = vehicle.transformed(Eigen::Affine3d(rotation));

  for (size_t corridor_index = 0; corridor_index < car_corridor.size(); ++corridor_index) {
    auto& gate = car_corridor.at(corridor_index);
    const Eigen::Translation3d translation(gate.getRightPole() - gate.getLeftPole());

    const Vehicle vehicle_left_pole =
        vehicle_gate_frame.transformed(gate.getTransformationFromGateFrame());
    const Vehicle vehicle_right_pole =
        vehicle_left_pole.transformed(Eigen::Affine3d(translation));

    if (gate.isDegenerated()) {
      ROS_WARN_THROTTLE(1, "Safe corridor contains degenerated gate.");
      continue;
    }
    const auto gates_before = boost::make_iterator_range(
        std::prev(safe_corridor.rend(), corridor_index), safe_corridor.rend());
    const auto gates_after = boost::make_iterator_range(
        std::next(safe_corridor.begin(), corridor_index), safe_corridor.end());

    double largest_violation_left_side = 0;
    double largest_violation_right_side = 0;

    // shrink corridor to fit front bumper
    largest_violation_left_side =
        std::max(largest_violation_left_side,
                 shrinkDistanceToFitBumper<Gate::LEFT>(
                     gates_after, vehicle_left_pole.getFrontBumper()));
    largest_violation_right_side = std::max(
        largest_violation_right_side,
        shrinkDistanceToFitBumper<Gate::RIGHT>(
            gates_after, vehicle_right_pole.getFrontBumper().swapEndpoints()));

    // shrink corridor to fit rear bumper
    largest_violation_left_side =
        std::max(largest_violation_left_side,
                 shrinkDistanceToFitBumper<Gate::LEFT>(
                     gates_before, vehicle_left_pole.getRearBumper()));
    largest_violation_right_side = std::max(
        largest_violation_right_side,
        shrinkDistanceToFitBumper<Gate::RIGHT>(
            gates_before, vehicle_right_pole.getRearBumper().swappedEndpoints()));

    // shrink corridor to fit left side
    const LineSegment left_side = vehicle_left_pole.getLeftSide();
    largest_violation_left_side =
        std::max(largest_violation_left_side,
                 shrinkDistanceToFitSide<Gate::LEFT>(gates_after, left_side));
    largest_violation_left_side =
        std::max(largest_violation_left_side,
                 shrinkDistanceToFitSide<Gate::LEFT>(gates_before, left_side));

    // shrink corridor to fit right side
    const LineSegment right_side = vehicle_right_pole.getRightSide();
    largest_violation_right_side =
        std::max(largest_violation_right_side,
                 shrinkDistanceToFitSide<Gate::RIGHT>(gates_after, right_side));
    largest_violation_right_side =
        std::max(largest_violation_right_side,
                 shrinkDistanceToFitSide<Gate::RIGHT>(gates_before, right_side));

    generateWarningsShrinkDistance(largest_violation_left_side,
                                   largest_violation_right_side,
                                   gate.getWidth(),
                                   large_shrink_distance);
    gate.shrinkFromLeftBy(largest_violation_left_side);
    gate.shrinkFromRightBy(largest_violation_right_side);

    // shrink corridor to respect car rotation and other approximation errors
    {
      const double shrink_distance = 2 * gate_safety_margin;
      if (shrink_distance > gate.getWidth()) {
        ROS_WARN_THROTTLE(1, "Corridor too small.");
      }
      gate.shrinkBy(shrink_distance);
    }
  }
}
