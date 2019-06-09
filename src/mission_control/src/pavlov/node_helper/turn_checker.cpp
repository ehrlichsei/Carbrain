#include "turn_checker.h"

const ParameterString<double> TurnChecker::PARAM_TURN_ARROW_TO_JUNCTION_DISTANCE_THRESHOLD(
    "turn_arrow_to_junction_distance_threshold");
const ParameterString<double> TurnChecker::PARAM_TURN_ARROW_TO_JUNCTION_ASSUMED_DISTANCE(
    "turn_arrow_to_junction_assumed_distance");

TurnChecker::TurnChecker(ParameterInterface* parameter_interface)
    : parameter_interface_(parameter_interface) {
  parameter_interface->registerParam(PARAM_TURN_ARROW_TO_JUNCTION_DISTANCE_THRESHOLD);
  parameter_interface->registerParam(PARAM_TURN_ARROW_TO_JUNCTION_ASSUMED_DISTANCE);
}

Eigen::Affine3d TurnChecker::getLatestWorldToTurnJunctionTransform() {
  return latest_world_to_turn_junction_transform;
}

bool TurnChecker::findTurnJunctionPose(const perception_msgs::ArrowMarking& msg,
                                       Eigen::Affine3d& out_turn_junction_pose) {
  Eigen::Affine3d arrow_pose;
  tf2::fromMsg(msg.pose, arrow_pose);

  return findTurnJunctionPose(arrow_pose, out_turn_junction_pose);
}

bool TurnChecker::findTurnJunctionPose(const Eigen::Affine3d& arrow_to_world_transform,
                                       Eigen::Affine3d& out_junction_to_world_transform) {

  const double arrow_to_junction_distance_threshold =
      parameter_interface_->getParam(PARAM_TURN_ARROW_TO_JUNCTION_DISTANCE_THRESHOLD);
  bool found_turn_junction = false;
  auto world_to_arrow_transform = arrow_to_world_transform.inverse();
  for (auto& junction : junctions_.sub_messages) {
    Eigen::Vector3d arrow_pos_in_world = arrow_to_world_transform.translation();
    Eigen::Vector3d junction_pos_in_world;
    tf2::fromMsg(junction.pose.position, junction_pos_in_world);
    Eigen::Vector3d junction_pos_in_arrow = world_to_arrow_transform * junction_pos_in_world;
    // Check that the junction lies in a half circle behind the arrow.
    if (((junction_pos_in_world - arrow_pos_in_world).norm() < arrow_to_junction_distance_threshold) &&
        (-junction_pos_in_arrow[0] < arrow_to_junction_distance_threshold)) {
      // Corresponding tracked junction with stop or holdline found.
      // Simply take the pose from the tracked junction.
      tf2::fromMsg(junction.pose, out_junction_to_world_transform);
      found_turn_junction = true;
      break;
    }
  }
  if (!found_turn_junction) {
    // TODO: Add magic perception service call for junction without stop or hold
    // lines here. Call it if no matching junction was found.
    // This is a workaround for the missing detection of intersections without
    // stop and hold lines.
    // Just translate the pose in x direction. I. e. assume that the junction is
    // in a configured distance.
    const float arrow_to_junction_assumed_distance =
        parameter_interface_->getParam(PARAM_TURN_ARROW_TO_JUNCTION_ASSUMED_DISTANCE);
    Eigen::Vector3d shift;
    shift << arrow_to_junction_assumed_distance, 0, 0;
    out_junction_to_world_transform = arrow_to_world_transform;
    out_junction_to_world_transform.translate(shift);
    found_turn_junction = true;
  }
  if (found_turn_junction) {
    // For debugging.
    latest_world_to_turn_junction_transform = out_junction_to_world_transform.inverse();
  }
  return found_turn_junction;
}

void TurnChecker::setJunctions(const navigation_msgs::Junctions& junctions) {
  junctions_ = junctions;
}
