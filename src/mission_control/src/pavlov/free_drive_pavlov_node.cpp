#include "free_drive_pavlov_node.h"
#include "common/best_score.h"
#include "common/node_creation_makros.h"

std::shared_ptr<FreeDrivePavlovNode::ParkingStateMachine> createParkingStateMachine(
    const std::shared_ptr<DataParking>& data_parking, StateMachineLogger& logger) {
  return std::make_shared<FreeDrivePavlovNode::ParkingStateMachine>(
      SM::ParkingSM(data_parking),
      std::make_shared<SM::DataQRCode>(),
      data_parking,
      SM::QRCodeFSM<DataParking, SM::ParkingSM>(),
      logger);
}

FreeDrivePavlovNode::FreeDrivePavlovNode(ros::NodeHandle& node_handle)
    : NodeBase(node_handle),
      number_of_park_attempts_ptr(std::make_shared<int>(0)),
      car_controller(node_handle_),
      diagnostics_iface(node_handle_),
      data_parking_(std::make_shared<DataParking>(
          &parameter_handler_, car_controller, diagnostics_iface, number_of_park_attempts_ptr)),
      free_drive_state_machine(createParkingStateMachine(data_parking_, logger_)) {
  parameter_handler_.registerParam(PARAM_ENABLE_QR_CODE_DETECTION);
}

void FreeDrivePavlovNode::startModule() {
  pavlov_node_helper_.startModule(free_drive_state_machine, node_handle_);
  start_line_subscriber = node_handle_.subscribe(
      "startline", 10, &FreeDrivePavlovNode::handleStartLine, this);
  path_subscriber = node_handle_.subscribe(
      "safe_target_path", 1, &FreeDrivePavlovNode::handlePath, this);
  parking_spot_subscriber = node_handle_.subscribe(
      "free_parking_spots", 1, &FreeDrivePavlovNode::handleParkingSpots, this);
  reset_subscriber =
      node_handle_.subscribe("auto_reset", 1, &FreeDrivePavlovNode::handleReset, this);
  if (!parameter_handler_.getParam(PARAM_ENABLE_QR_CODE_DETECTION)) {
    free_drive_state_machine->process_event(EventStart());
  }
}

void FreeDrivePavlovNode::stopModule() {
  pavlov_node_helper_.stopModule();
  start_line_subscriber.shutdown();
  parking_spot_subscriber.shutdown();
  path_subscriber.shutdown();
  parking_lot_found_subscriber.shutdown();
}

const std::string FreeDrivePavlovNode::getName() {
  return std::string("free_drive_pavlov");
}

void FreeDrivePavlovNode::handleStartLine(const perception_msgs::StartLines::ConstPtr& lines_msg) {
  if (lines_msg->sub_messages.empty()) {
    return;
  }
  const auto certaintyAsScore = [](const auto& msg) { return msg.certainty; };
  const auto highest_prob_msg =
      *common::max_score(lines_msg->sub_messages, certaintyAsScore);
  if (highest_prob_msg.certainty < 0.75) {
    return;
  }

  ROS_INFO_THROTTLE(0.5, "Received Start Line!!!");
  Eigen::Vector3d start_line_position = Eigen::Vector3d::Zero();
  tf2::fromMsg(highest_prob_msg.pose.position, start_line_position);
  Eigen::Affine3d vehicle_pose;
  if (!pavlov_node_helper_.vehicleToWorld(
          ros::Time(0), Eigen::Affine3d::Identity(), vehicle_pose)) {
    return;
  }
  const double dist_before_vehicle = (vehicle_pose.inverse() * start_line_position).x();
  free_drive_state_machine->process_event(EventStartLineDetected(dist_before_vehicle));
}

void FreeDrivePavlovNode::handleReset(const std_msgs::Empty::ConstPtr&) {
  ROS_INFO_STREAM("Resetting node.");

  free_drive_state_machine->process_event(EventReset());
  data_parking_->getCarController().resetCarController();
  free_drive_state_machine = createParkingStateMachine(data_parking_, logger_);
  free_drive_state_machine->process_event(EventStart());
}

void FreeDrivePavlovNode::handleParkingSpots(
    const perception_msgs::PerpendicularParkingSpots::ConstPtr& parking_spots_msg) {
  Eigen::Affine3d vehicle_pose;
  if (!pavlov_node_helper_.vehicleToWorld(
          ros::Time(0), Eigen::Affine3d::Identity(), vehicle_pose)) {
    return;
  }
  if (parking_spots_msg->sub_messages.empty()) {
    return;
  }
  const auto parking_spot =
      *common::min_score(parking_spots_msg->sub_messages,
                         [&](const auto& spot) {
                           Eigen::Vector3d left_position;
                           tf2::fromMsg(spot.entrance_pose_left.position, left_position);
                           return (vehicle_pose.translation() - left_position).norm();
                         });
  free_drive_state_machine->process_event(
      EventFoundParkingSpot(vehicle_pose.translation(), parking_spot));
}

void FreeDrivePavlovNode::handlePath(const nav_msgs::Path::ConstPtr& path_msg) {
  Eigen::Affine3d path_pose, vehicle_pose;
  if (!pavlov_node_helper_.vehicleToWorld(Eigen::Affine3d::Identity(), vehicle_pose)) {
    return;
  }
  if (!pavlov_node_helper_.pathToWorld(
          path_msg->header.stamp, Eigen::Affine3d::Identity(), path_pose)) {
    return;
  }
  free_drive_state_machine->process_event(EventPathMsgUpdateReversePath(path_msg, path_pose));
  free_drive_state_machine->process_event(
      EventPathMsgStopping(path_msg, path_pose, vehicle_pose));
}



CREATE_NODE(FreeDrivePavlovNode)
