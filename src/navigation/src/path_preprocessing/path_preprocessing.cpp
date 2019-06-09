#include "path_preprocessing.h"

THIRD_PARTY_HEADERS_BEGIN
#include <cmath>
#include <iostream>

#include <Eigen/Eigen>
#include <Eigen/Geometry>
#include <Eigen/Core>

#include <ros/console.h>
THIRD_PARTY_HEADERS_END

#include "common/debug.h"
#include "common/math.h"
#include "common/minmax_element.h"
#include "common/pca_eigen.h"
#include "common/polynomialfit.h"

#include "pathutils.h"
#include "gate_utils.h"

using namespace Eigen;

PathPreprocessing::PathPreprocessing(ParameterInterface* parameters)
    : parameters_ptr_(parameters),
      next_gate_id_(0),
      turning_state(DONT_TURN),
      park_at_perpendicular_spot_(false) {
  parameters->registerParam(PARAM_LANE_WIDTH);
  parameters->registerParam(PARAM_TRACKING_DISTANCE_POLYNOMIAL);
  parameters->registerParam(PARAM_TRACKING_DISTANCE_BEHIND);
  parameters->registerParam(PARAM_TRACKING_DISTANCE_IN_FRONT);
  parameters->registerParam(PARAM_CORRIDOR_SIMPLIFICATION_MAX_MERGE_DISTANCE);
  parameters->registerParam(PARAM_ENABLE_PERPENDICULAR_PARKING);
  parameters->registerParam(PARAM_ENABLE_TURNING);
  parameters->registerParam(PARAM_INITIAL_GATE_DISTANCE);
  parameters->registerParam(PARAM_MAX_NUMBER_GATES);
  parameters->registerParam(PARAM_DISTANCE_OF_TURNING_POINT_TO_RIGHT_LANE_MARKING);
  parameters->registerParam(PARAM_STRAIGHT_ON_DISTANCE_AFTER_TURNING);
  parameters->registerParam(PARAM_USE_ADAPTIVE_POLYNOMIAL_DEGREE);
  parameters->registerParam(PARAM_PATH_POLYNOMIAL_DEGREE);
  parameters->registerParam(WEIGHT_MIDDLE_INTERSECTION);
  parameters->registerParam(WEIGHT_OPPOSITE_INTERSECTION);
  parameters->registerParam(WEIGHT_OWN_INTERSECTION);
  parameters->registerParam(THRESHOLD_USE_ALWAYS_DEGREE_1);
  parameters->registerParam(THRESHOLD_USE_NEVER_DEGREE_5);
  parameters->registerParam(THRESHOLD_RELATIVE_ERROR_1_5);
  parameters->registerParam(THRESHOLD_RELATIVE_ERROR_3_5);
  parameters->registerParam(TURNING_ANGLE);
  parameters->registerParam(PERPENDICULAR_PARKING_ANGLE);
  parameters->registerParam(TURNING_ANGLE_START_OFFSET);
  parameters->registerParam(TURNING_ANGLE_STEP_SIZE);
  parameters->registerParam(PARAM_LENGTH_OF_PARKING_SPOT);
  tracked_lane = TrackedLane(parameters->getParam(PARAM_TRACKING_DISTANCE_POLYNOMIAL));
}

bool PathPreprocessing::createDrivingCorridor(const Affine3d& vehicle_to_world_3d,
                                              const LaneUtils::Lanes& lane_paths) {
  turning_enabled = parameters_ptr_->getParam(PARAM_ENABLE_TURNING);
  perpendicular_parking_enabled =
      parameters_ptr_->getParam(PARAM_ENABLE_PERPENDICULAR_PARKING);

  const LaneUtils::Lanes lanes{PathUtils::bin(lane_paths.left, 20),
                               PathUtils::bin(lane_paths.middle, 20),
                               PathUtils::bin(lane_paths.right, 20)};

  Affine3d world_to_path_transform;
  if (!updateLanePoints(vehicle_to_world_3d, lanes, &world_to_path_transform)) {
    ROS_ERROR_THROTTLE(2, "can't create corridor, there are not enought gates");
    return false;
  }

  weight_middle_intersection = parameters_ptr_->getParam(WEIGHT_MIDDLE_INTERSECTION);
  weight_opposite_intersection = parameters_ptr_->getParam(WEIGHT_OPPOSITE_INTERSECTION);
  weight_own_intersection = parameters_ptr_->getParam(WEIGHT_OWN_INTERSECTION);

  Gate::GateList gates;
  gates.reserve(200);

  if (keepTrackedGates(&gates, vehicle_to_world_3d, lane_paths)) {
    createUpdatedGates(&gates, vehicle_to_world_3d, lanes);
  } else {
    common::Vector2dVector tracked_lane_path;
    common::DynamicPolynomial lane_polynomial;
    createLanePolynomial(&tracked_lane_path, &lane_polynomial, &world_to_path_transform);
    createInitialGates(&gates, vehicle_to_world_3d, lanes, lane_polynomial, tracked_lane_path);
  }

  full_corridor_ = DrivingCorridor(std::move(gates));
  full_corridor_.sort(world_to_path_transform);
  const double max_merge_distance =
      parameters_ptr_->getParam(PARAM_CORRIDOR_SIMPLIFICATION_MAX_MERGE_DISTANCE);
  full_corridor_ = full_corridor_.simplified(max_merge_distance);

  addTurningGatesToCorridor();
  addPerpendicularParkingToCorridor();

  if (full_corridor_.size() < 2) {
    ROS_ERROR_THROTTLE(2, "can't create corridor, there are not enought gates");
    return false;
  }
  path_to_world_transform_ = determinePathTransformFullCorridor(vehicle_to_world_3d);
  return true;
}

void PathPreprocessing::createInitialGates(Gate::GateList* gates,
                                           const Affine3d& vehicle_to_world_3d,
                                           const LaneUtils::Lanes& lanes,
                                           const common::DynamicPolynomial& lane_polynomial,
                                           const common::Vector2dVector& tracked_lane_path) {
  Gate::GateList initial_gates;
  const double lane_width_param = parameters_ptr_->getParam(PARAM_LANE_WIDTH);
  const double gate_distance = parameters_ptr_->getParam(PARAM_INITIAL_GATE_DISTANCE);
  LaneUtils::Lanes old_lanes;
  extractOldLanes(&old_lanes);

  const auto minmax = common::minmax_element(tracked_lane_path, common::less_x());
  const double min_x = minmax.first->x();
  const double max_x = minmax.second->x();
  initial_gates.reserve(static_cast<std::size_t>((max_x - min_x) / gate_distance));
  for (double x = min_x; x < max_x; x += gate_distance) {
    const Gate initial_gate = GateUtils::fromPolynomial(
        lane_polynomial, x, path_to_world_transform_, lane_width_param, next_gate_id_);
    next_gate_id_++;
    if (startTurning(initial_gate)) {
      break;
    }
    initial_gates.push_back(initial_gate);
  }

  // adapt gates to percepted lane paths
  double lane_width = lane_width_param;
  for (Gate gate : initial_gates) {
    Vector3d direction;
    GateUtils::adaptGateToLane(
        &gate,
        &direction,
        &lane_width,
        &lane_width,
        lane_width_param,
        (gate.getCenter() - vehicle_to_world_3d.translation()).norm(),
        weight_middle_intersection,
        weight_own_intersection,
        weight_opposite_intersection,
        old_lanes,
        lanes);
    if (startTurning(gate)) {
      break;
    }
    gates->push_back(gate);
  }
}

void PathPreprocessing::createUpdatedGates(Gate::GateList* gates,
                                           const Affine3d& vehicle_to_world_3d,
                                           const LaneUtils::Lanes& lanes) {
  const double lane_width_param = parameters_ptr_->getParam(PARAM_LANE_WIDTH);
  const double gate_distance = parameters_ptr_->getParam(PARAM_INITIAL_GATE_DISTANCE);
  const size_t max_gates = parameters_ptr_->getParam(PARAM_MAX_NUMBER_GATES);
  LaneUtils::Lanes old_lanes;
  extractOldLanes(&old_lanes);

  Vector3d path_direction = GateUtils::calculateNewDirection(*gates, 1);
  Vector3d last_path_direction = path_direction;
  Gate new_gate = gates->back();
  new_gate.setId(next_gate_id_++);
  double current_lane_width_left = lane_width_param;
  double current_lane_width_right = lane_width_param;
  while (gates->size() < max_gates &&
         GateUtils::adaptGateToLane(
             &new_gate,
             &path_direction,
             &current_lane_width_left,
             &current_lane_width_right,
             lane_width_param,
             (new_gate.getCenter() - vehicle_to_world_3d.translation()).norm(),
             weight_middle_intersection,
             weight_own_intersection,
             weight_opposite_intersection,
             old_lanes,
             lanes)) {
    path_direction.normalize();
    path_direction += 3 * last_path_direction;
    path_direction.normalize();
    last_path_direction = path_direction;
    gates->push_back(new_gate);
    new_gate = GateUtils::fromGate(
        new_gate, path_direction, gate_distance, lane_width_param, next_gate_id_);
    next_gate_id_++;
    if (startTurning(new_gate)) {
      break;
    }
  }
}

bool PathPreprocessing::keepTrackedGates(Gate::GateList* gates,
                                         const Affine3d& vehicle_to_world_3d,
                                         const LaneUtils::Lanes& lanes) {
  Vector3d lookahead_point = vehicle_to_world_3d * vehicle_lookahead_point_;
  if (!full_corridor_.isPointApproximateContained(vehicle_to_world_3d.translation()) ||
      !full_corridor_.isPointApproximateContained(lookahead_point)) {
    return false;
  }

  const double tracking_distance_behind =
      parameters_ptr_->getParam(PARAM_TRACKING_DISTANCE_BEHIND);
  const double tracking_distance_in_front =
      parameters_ptr_->getParam(PARAM_TRACKING_DISTANCE_IN_FRONT);
  // merge old gates until first points from perception are present in front of
  // vehicle
  for (const Gate& gate : full_corridor_) {
    const double distance_to_vehicle =
        (gate.getCenter() - vehicle_to_world_3d.translation()).norm();
    const double distance_to_lookahead = (gate.getCenter() - lookahead_point).norm();
    if (distance_to_vehicle > tracking_distance_behind &&
        !gate.isPointInFront(vehicle_to_world_3d.translation())) {
      continue;
    }
    if (startTurning(gate)) {
      break;
    }
    if (distance_to_lookahead < tracking_distance_in_front ||
        gate.isPointBehind(lookahead_point) || !GateUtils::hasIntersection(gate, lanes)) {
      gates->push_back(gate);
    } else {
      // do not continue when first perception point is reached
      break;
    }
  }
  return !gates->empty();
}

void PathPreprocessing::extractOldLanes(LaneUtils::Lanes* old_lanes) {
  (old_lanes->left).reserve(full_corridor_.size());
  old_lanes->middle.reserve(full_corridor_.size());
  old_lanes->right.reserve(full_corridor_.size());
  for (const Gate& gate : full_corridor_) {
    old_lanes->left.push_back(gate.getLeftLaneBoundary());
    old_lanes->middle.push_back(gate.getCenter());
    old_lanes->right.push_back(gate.getRightLaneBoundary());
  }
  old_lanes->left = PathUtils::bin(old_lanes->left, 20);
  old_lanes->middle = PathUtils::bin(old_lanes->middle, 20);
  old_lanes->right = PathUtils::bin(old_lanes->right, 20);
}

bool PathPreprocessing::updateLanePoints(const Affine3d& vehicle_to_world_3d,
                                         const LaneUtils::Lanes& lanes,
                                         Affine3d* world_to_path_transform) {
  const Eigen::Vector2d vehicle_position_world =
      vehicle_to_world_3d.translation().topRows<2>();
  Eigen::Vector3d vehicle_direction_world =
      vehicle_to_world_3d.rotation() * Eigen::Vector3d::UnitX();

  integrateNewLanePaths(vehicle_position_world, lanes);

  if (tracked_lane.getLanePoints().size() < 2) {
    ROS_ERROR_THROTTLE(
        2, "can't create corridor, there are not enought lane points");
    return false;
  }

  path_to_world_transform_ =
      determinePathTransform(vehicle_position_world,
                             vehicle_direction_world.topRows<2>(),
                             tracked_lane.getLanePoints());
  *world_to_path_transform = path_to_world_transform_.inverse();
  return true;
}

void PathPreprocessing::createLanePolynomial(common::EigenAlignedVector<Vector2d>* tracked_lane_path,
                                             common::DynamicPolynomial* lane_polynomial,
                                             Affine3d* world_to_path_transform) const {
  // transform tracked_lane to path coordinates
  tracked_lane_path->reserve(tracked_lane.getLanePoints().size());
  for (const auto& tracked_lane_point : tracked_lane.getLanePoints()) {
    Eigen::Vector3d point_path = *world_to_path_transform * tracked_lane_point;
    tracked_lane_path->push_back(point_path.topRows<2>());
  }
  fitPolynomial(*tracked_lane_path, *lane_polynomial);
}

void PathPreprocessing::integrateNewLanePaths(const Eigen::Vector2d& vehicle_pos,
                                              const LaneUtils::Lanes& lanes) {
  if (lanes.left.size() + lanes.middle.size() + lanes.right.size() < 4) {
    ROS_WARN_THROTTLE(3, "Strange: Less than 4 Points to integrate.");
  }

  const double lane_width = parameters_ptr_->getParam(PARAM_LANE_WIDTH);

  common::EigenAlignedVector<Eigen::Vector3d> new_points;
  if (lanes.left.size() > 10) {
    boost::push_back(new_points, PathUtils::normalShift(lanes.left, 1 * lane_width));
  }
  if (lanes.middle.size() > 10) {
    boost::push_back(new_points, PathUtils::normalShift(lanes.middle, 0 * lane_width));
  }
  if (lanes.right.size() > 10) {
    boost::push_back(new_points, PathUtils::normalShift(lanes.right, -1 * lane_width));
  }

  if (new_points.size() < 4) {
    ROS_WARN_THROTTLE(3, "Strange: Less than 4 Points in new Lane Path.");
    return;
  }
  ROS_DEBUG_STREAM("There are " << new_points.size()
                                << " new points available!\n");
  tracked_lane.setTrackingDistance(parameters_ptr_->getParam(PARAM_TRACKING_DISTANCE_POLYNOMIAL));
  lane_aabb = tracked_lane.integratePoints(vehicle_pos, std::move(new_points));
  ROS_DEBUG_STREAM("The tracked lane now contains "
                   << tracked_lane.getLanePoints().size() << " points.\n");
}

const TrackedLane& PathPreprocessing::getTrackedLane() const {
  return tracked_lane;
}

Affine3d PathPreprocessing::determinePathTransform(
    const Vector2d& /*vehicle_pos*/,
    const Vector2d& vehicle_orientation,
    const common::EigenAlignedVector<Eigen::Vector3d>& points) const {
  const Eigen::MatrixXd points_matrix = common::toMatrix2D(points);
  const Vector2d center = points_matrix.colwise().mean();
  const Vector2d principal_component = common::getPrincipalComponent(points_matrix);
  const Vector2d oriented_principal_component =
      common::ensureSameOrientation(principal_component, vehicle_orientation);
  const double pca_angle =
      common::getAngle(oriented_principal_component, Vector2d::UnitX());
  DUMP_D(pca_angle);
  DUMP(oriented_principal_component);
  DUMP(vehicle_orientation);
  return Translation3d(to3D(center)) * AngleAxisd(pca_angle, Vector3d::UnitZ());
}

Affine3d PathPreprocessing::determinePathTransformFullCorridor(const Affine3d& vehicle_to_world_3d) const {
  common::EigenAlignedVector<Vector3d> center_points;
  center_points.reserve(full_corridor_.size());
  for (const auto& gate : full_corridor_) {
    center_points.push_back(gate.getCenter());
  }

  return determinePathTransform(
      vehicle_to_world_3d.translation().topRows<2>(),
      (vehicle_to_world_3d.rotation() * Vector3d::UnitX()).topRows<2>(),
      center_points);
}

const Affine3d& PathPreprocessing::getPathTransform() const {
  return path_to_world_transform_;
}

const DrivingCorridor& PathPreprocessing::getFullCorridor() const {
  return full_corridor_;
}

void PathPreprocessing::fitPolynomial(const common::EigenAlignedVector<Eigen::Vector2d>& path,
                                      common::DynamicPolynomial& polynomial) const {

  if (!parameters_ptr_->getParam(PARAM_USE_ADAPTIVE_POLYNOMIAL_DEGREE)) {
    polynomial = common::fitToPoints(
        path, parameters_ptr_->getParam(PARAM_PATH_POLYNOMIAL_DEGREE));
    return;
  }

  common::DynamicPolynomial poly1 =
      common::fitToPoints(path, common::PolynomialDegrees::Linear);
  float absolute_error_1 = common::computeAbsoluteFittingError(path, poly1);

  common::DynamicPolynomial poly3 =
      common::fitToPoints(path, common::PolynomialDegrees::Cubic);
  float absolute_error_3 = common::computeAbsoluteFittingError(path, poly3);

  common::DynamicPolynomial poly5 =
      common::fitToPoints(path, common::PolynomialDegrees::Quintic);
  float absolute_error_5 = common::computeAbsoluteFittingError(path, poly5);

  float threshold_1_always = parameters_ptr_->getParam(THRESHOLD_USE_ALWAYS_DEGREE_1);
  float threshold_5_max = parameters_ptr_->getParam(THRESHOLD_USE_NEVER_DEGREE_5);
  float threshold_relative_error_1_5 =
      parameters_ptr_->getParam(THRESHOLD_RELATIVE_ERROR_1_5);
  float threshold_relative_error_3_5 =
      parameters_ptr_->getParam(THRESHOLD_RELATIVE_ERROR_3_5);

  int use_degree = 0;
  if (absolute_error_1 < threshold_1_always ||
      absolute_error_1 / absolute_error_5 < threshold_relative_error_1_5) {
    polynomial = poly1;
    use_degree = 1;
  } else if (absolute_error_3 / absolute_error_5 < threshold_relative_error_3_5 ||
             absolute_error_5 > threshold_5_max) {
    polynomial = poly3;
    use_degree = 3;
  } else {
    polynomial = poly5;
    use_degree = 5;
  }

  ROS_DEBUG("[polynomial degree]: error, [1]:%f, [3]:%f, [5]:%f -> using [%d]",
            absolute_error_1,
            absolute_error_3,
            absolute_error_5,
            use_degree);
}

void PathPreprocessing::reset() {
  turning_state = DONT_TURN;
  park_at_perpendicular_spot_ = false;
  tracked_lane.clearLanePoints();
  full_corridor_.clear();
}

void PathPreprocessing::setVehicleLookaheadPoint(const Vector3d& point) {
  vehicle_lookahead_point_ = point;
}

void PathPreprocessing::setTurning(bool turn_left, Eigen::Affine3d turning_point) {
  start_turning = turning_point.translation();
  start_turning_orientation = turning_point.rotation();
  if (turn_left) {
    turning_state = TURN_LEFT;
  } else {
    turning_state = TURN_RIGHT;
  }
}

void PathPreprocessing::resetTurning() { turning_state = DONT_TURN; }

bool PathPreprocessing::startTurning(const Gate& gate) const {
  if ((!turning_enabled || turning_state == DONT_TURN) &&
      (!perpendicular_parking_enabled || !park_at_perpendicular_spot_)) {
    return false;
  }

  return !gate.getLineSegment().isPointOnLeftSide(start_turning.topRows<2>());
}

bool PathPreprocessing::pointBehindStartGateParking(const Eigen::Vector3d& point) const {
  if (!perpendicular_parking_enabled || !park_at_perpendicular_spot_) {
    return false;
  }
  const int gate_id = 0;  // gate id is irrelevant
  const Gate first_parking_gate = createFirstParkingGate(gate_id);
  return first_parking_gate.getLineSegment().isPointOnLeftSide(point.topRows<2>());
}

void PathPreprocessing::addTurningGatesToCorridor() {
  if (turning_enabled && turning_state != DONT_TURN) {
    const double gate_distance = parameters_ptr_->getParam(PARAM_INITIAL_GATE_DISTANCE);
    const double lane_width = parameters_ptr_->getParam(PARAM_LANE_WIDTH);
    const double point_width =
        parameters_ptr_->getParam(PARAM_DISTANCE_OF_TURNING_POINT_TO_RIGHT_LANE_MARKING);
    const double straight_on_distance =
        parameters_ptr_->getParam(PARAM_STRAIGHT_ON_DISTANCE_AFTER_TURNING);
    const double start = parameters_ptr_->getParam(TURNING_ANGLE_START_OFFSET);
    const double step = parameters_ptr_->getParam(TURNING_ANGLE_STEP_SIZE);
    const double arc_angle = parameters_ptr_->getParam(TURNING_ANGLE);
    if (turning_state == TURN_LEFT) {
      ROS_INFO_THROTTLE(5, "turning left");
      Vector3d inner_point = start_turning +
                             (2.0 * lane_width - point_width) *
                                 (-start_turning_orientation * -Vector3d::UnitY());
      Vector3d gate_direction =
          start_turning_orientation * (2.0 * lane_width * Vector3d::UnitY());
      for (float i = start; i < arc_angle; i += step) {
        full_corridor_.push_back(Gate(
            next_gate_id_, inner_point, inner_point - AngleAxisd(i, Vector3d::UnitZ()) * gate_direction));
        next_gate_id_++;
      }
      for (float i = gate_distance; i < straight_on_distance; i += gate_distance) {
        full_corridor_.push_back(Gate(
            next_gate_id_,
            inner_point + i * gate_direction,
            inner_point + i * gate_direction -
                AngleAxisd(1.0 / 2.0 * M_PI, Vector3d::UnitZ()) * gate_direction));
        next_gate_id_++;
      }

    } else if (turning_state == TURN_RIGHT) {
      ROS_INFO_THROTTLE(5, "turning right");
      Vector3d inner_point =
          start_turning - point_width * (-start_turning_orientation * -Vector3d::UnitY());
      Vector3d gate_direction =
          start_turning_orientation * (2.0 * lane_width * Vector3d::UnitY());
      for (float i = start; i < arc_angle; i += step) {
        full_corridor_.push_back(Gate(
            next_gate_id_, inner_point - AngleAxisd(-i, Vector3d::UnitZ()) * -gate_direction, inner_point));
        next_gate_id_++;
      }
      for (float i = gate_distance; i < straight_on_distance; i += gate_distance) {
        full_corridor_.push_back(Gate(
            next_gate_id_,
            inner_point - i * gate_direction -
                AngleAxisd(1.0 / 2.0 * M_PI, Vector3d::UnitZ()) * gate_direction,
            inner_point - i * gate_direction));
        next_gate_id_++;
      }
    }
  }
}

void PathPreprocessing::parkPerpendicular(const Eigen::Affine3d& left,
                                          const Eigen::Affine3d& right) {
  park_at_perpendicular_spot_ = true;
  perpendicular_parking_left_entrance_ = left;
  perpendicular_parking_right_entrance_ = right;
  double lane_width = parameters_ptr_->getParam(PARAM_LANE_WIDTH);
  start_turning =
      (perpendicular_parking_right_entrance_.translation() +
       perpendicular_parking_left_entrance_.translation()) /
          2.0 +
      1.5 * lane_width *
          (perpendicular_parking_left_entrance_.rotation() * Vector3d::UnitY());
}

Gate PathPreprocessing::createFirstParkingGate(const int gate_id) const {
  const double lane_width = parameters_ptr_->getParam(PARAM_LANE_WIDTH);
  const double start = parameters_ptr_->getParam(TURNING_ANGLE_START_OFFSET);
  const Eigen::Vector3d gate_direction =
      perpendicular_parking_left_entrance_.rotation() *
      (2.0 * lane_width * -Eigen::Vector3d::UnitX());
  return Gate(gate_id,
              start_turning,
              start_turning + AngleAxisd(start, Eigen::Vector3d::UnitZ()) * gate_direction);
}

void PathPreprocessing::addPerpendicularParkingToCorridor() {
  if (perpendicular_parking_enabled && park_at_perpendicular_spot_) {
    const double gate_distance = parameters_ptr_->getParam(PARAM_INITIAL_GATE_DISTANCE);
    const double length_of_parking_lot =
        parameters_ptr_->getParam(PARAM_LENGTH_OF_PARKING_SPOT);
    ROS_INFO_THROTTLE(5, "park perpendicular");
    const double start = parameters_ptr_->getParam(TURNING_ANGLE_START_OFFSET);
    const double step = parameters_ptr_->getParam(TURNING_ANGLE_STEP_SIZE);
    const double arc_angle = parameters_ptr_->getParam(PERPENDICULAR_PARKING_ANGLE);
    const double lane_width = parameters_ptr_->getParam(PARAM_LANE_WIDTH);
    Vector3d gate_direction = perpendicular_parking_left_entrance_.rotation() *
                              (2.0 * lane_width * -Eigen::Vector3d::UnitX());
    const int gate_id = 0;  // gate id is irrelevant
    Gate first_parking_gate = createFirstParkingGate(gate_id);
    while (!full_corridor_.empty() &&
           (first_parking_gate.getLineSegment().isPointOnLeftSide(
                full_corridor_.back().getRightPole().topRows<2>()) ||
            first_parking_gate.getLineSegment().isPointOnLeftSide(
                full_corridor_.back().getLeftPole().topRows<2>()))) {
      full_corridor_.pop_back();
    }
    for (double i = start; i < arc_angle; i += step) {
      full_corridor_.push_back(Gate(
          next_gate_id_,
          start_turning,
          start_turning + AngleAxisd(i, Vector3d::UnitZ()) * gate_direction));
      next_gate_id_++;
    }
    for (float i = gate_distance; i < length_of_parking_lot; i += gate_distance) {
      full_corridor_.push_back(Gate(
          next_gate_id_,
          0.0,
          1.0,
          perpendicular_parking_left_entrance_.translation() +
              i * perpendicular_parking_left_entrance_.rotation() * Eigen::Vector3d::UnitX(),
          perpendicular_parking_right_entrance_.translation() +
              i * perpendicular_parking_right_entrance_.rotation() * Eigen::Vector3d::UnitX(),
          0.0));
      next_gate_id_++;
    }
  }
}
