#include "carcontroller.h"
#include <common/tf2_eigen_addon.h>

CarController::CarController(ros::NodeHandle node_handle)
    : max_speed_id_counter(0) {
  stopping_client_ = node_handle.serviceClient<longitudinal_controller_msgs::StopAt>(
      "stop_at");
  drive_past_next_road_closure_client_ =
      node_handle.serviceClient<navigation_msgs::DrivePastNextRoadClosure>(
          "set_drive_past_next_road_closure");
  look_at_client_ =
      node_handle.serviceClient<perception_msgs::LookAt>("look_at");
  turn_at_client_ =
      node_handle.serviceClient<navigation_msgs::TurnAt>("turn_at");
  set_pavlov_blinker_command_client_ =
      node_handle.serviceClient<navigation_msgs::PavlovBlinkerCommand>(
          "blinker_command_to_blinker_node");
  set_respect_no_passing_zone_service_ =
      node_handle.serviceClient<navigation_msgs::SetRespectNoPassingZones>(
          "set_respect_no_passing_zones");
  acc_activation_ =
      node_handle.serviceClient<std_srvs::SetBool>("activate_acc");
  acc_publisher = node_handle.advertise<navigation_msgs::AutomaticCruiseControlUpdate>(
      "acc_update", 1);
  search_parking_lot_client_ =
      node_handle.serviceClient<perception_msgs::SearchParkingSpots>(
          "parking_service");
  plan_path_parking_lot_client_ =
      node_handle.serviceClient<navigation_msgs::ParkPerpendicular>(
          "park_perpendicular_at");
  out_of_start_box_client_ =
      node_handle.serviceClient<std_srvs::SetBool>("out_of_start_box_service");
  reverse_out_of_parking_spot_client_ =
      node_handle.serviceClient<navigation_msgs::ReverseOutOfParkingSpot>(
          "reverse_out_of_parking_spot_service");
  velocity_publisher = node_handle.advertise<std_msgs::Float32>("desired_speed", 20, true);
  reset_lane_detection_client_ =
      node_handle.serviceClient<std_srvs::Empty>("reset_lane_detection");
  reset_path_preprocessing_client_ =
      node_handle.serviceClient<std_srvs::Empty>("reset_path_preprocessing");
  reset_environmental_model_client_ =
      node_handle.serviceClient<std_srvs::Empty>("reset_environmental_model");
  activate_qr_code_detection_client_ = node_handle.serviceClient<std_srvs::SetBool>(
      "activate_qr_code_detection_service");
  light_publisher =
      node_handle.advertise<controller_msgs::LightsCommand>("blinker_command", 1);
}



unsigned long CarController::stopAtDistance(const double distance, const unsigned long id) {
  longitudinal_controller_msgs::StopAt stop_msg;
  stop_msg.request.distance = distance;
  stop_msg.request.stop = true;
  stop_msg.request.id = id;

  if (stopping_client_.call(stop_msg)) {
    ROS_INFO_STREAM(
        "CarController: Called stop car on high level controller with distance "
        << distance << ".");
    return stop_msg.response.id;
  } else {
    ROS_WARN_STREAM_THROTTLE(
        1, "Failed to call stop car on high level controller with distance " << distance << ".");
  }
  return id;
}

void CarController::startDriving(const unsigned long id) {
  longitudinal_controller_msgs::StopAt dont_stop_msg;
  dont_stop_msg.request.distance = 0;
  dont_stop_msg.request.stop = false;
  dont_stop_msg.request.id = id;
  if (stopping_client_.call(dont_stop_msg)) {
    ROS_INFO_STREAM(
        "CarController: Called start driving on high level controller");
  } else {
    ROS_WARN_THROTTLE(1,
                      "Failed to call start driving on high level controller.");
  }
}

unsigned long CarController::setMaxSpeedInModelSizeKmPerH(const double speed_limit_in_km_per_h,
                                                          const unsigned long id) {
  double max_speed_in_m_per_s =
      speed_limit_in_km_per_h / 36.0;  // Using model factor 1 / 10
  return setMaxSpeedInMPerS(max_speed_in_m_per_s, id);
}

template <typename A, typename B>
static bool compareSecond(const std::pair<A, B> &a, const std::pair<A, B> &b) {
  return a.second < b.second;
}

unsigned long CarController::setMaxSpeedInMPerS(const double desired_speed_in_m_per_s,
                                                const unsigned long id) {
  unsigned long assigned_id;
  if (id == 0) {
    assigned_id = ++max_speed_id_counter;
  } else {
    assigned_id = id;
  }
  max_speed_limitations[assigned_id] = desired_speed_in_m_per_s;

  std_msgs::Float32 msg;
  msg.data = getMinMaxSpeedOrInfinity();
  velocity_publisher.publish(msg);

  ROS_INFO_STREAM("Update max speed to " << msg.data << ".");
  return assigned_id;
}

void CarController::setMaxSpeedAfterQRCode() {
  std_msgs::Float32 msg;
  msg.data = std::numeric_limits<double>::infinity();
  velocity_publisher.publish(msg);
  ROS_INFO("set max speed to infinity after qr code");
}

void CarController::clearMaxSpeed(const unsigned long id) {
  max_speed_limitations.erase(id);
  std_msgs::Float32 msg;
  msg.data = getMinMaxSpeedOrInfinity();
  velocity_publisher.publish(msg);
  ROS_INFO_STREAM("Clear max speed to " << msg.data << ".");
}

void CarController::deActivateACC(const bool activate) {
  std_srvs::SetBool msg;
  msg.request.data = activate;
  if (acc_activation_.call(msg)) {
    ROS_INFO_STREAM(
        "CarController: Called deActivateACC at service with activate = "
        << std::boolalpha << activate);
  } else {
    ROS_WARN_STREAM("Failed to call deActivateACC service with activate = "
                    << std::boolalpha << activate);
  }
}

void CarController::publishACCMessage(const double speed, const double distance) {
  navigation_msgs::AutomaticCruiseControlUpdate msg;
  msg.header.stamp = ros::Time::now();
  msg.speed = speed;
  msg.distance = distance;
  acc_publisher.publish(msg);
  ROS_INFO_STREAM_THROTTLE(
      1,
      "(Throttled) CarController: Published ACC with speed: "
          << speed << " and distance " << distance << ".");
}

void CarController::setDrivePastNextRoadClosure(const bool should_drive_past) {
  navigation_msgs::DrivePastNextRoadClosure msg;
  msg.request.drive_past_next_road_closure = should_drive_past;
  if (drive_past_next_road_closure_client_.call(msg)) {
    ROS_INFO_STREAM(
        "CarController: Called drive past next road closure on collision "
        "detection: "
        << should_drive_past);
  } else {
    ROS_WARN_THROTTLE(
        1,
        "Failed to call drive past next road closure on collision detection.");
  }
}

void CarController::startLookAt(const tf2::Stamped<Eigen::Affine3d> pose,
                                const Eigen::Vector3d rect) {
  perception_msgs::LookAt msg;
  msg.request.do_look = true;
  msg.request.pose = tf2::toMsg(pose);
  tf::vectorEigenToMsg(rect, msg.request.rect);
  if (look_at_client_.call(msg)) {
    ROS_INFO_STREAM(
        "CarController: Called look at service with do_look: true.");
  } else {
    ROS_WARN_THROTTLE(1, "Failed to call look at service.");
  }
}

void CarController::stopLookAt() {
  perception_msgs::LookAt msg;
  msg.request.do_look = false;
  if (look_at_client_.call(msg)) {
    ROS_INFO_STREAM(
        "CarController: Called look at service with do_look: false.");
  } else {
    ROS_WARN_THROTTLE(1, "Failed to call look at service.");
  }
}


void CarController::searchParkingSpot(const bool search) {
  perception_msgs::SearchParkingSpots msg;
  msg.request.search = search;
  if (search_parking_lot_client_.call(msg)) {
    ROS_INFO_STREAM_THROTTLE(
        0.5, "CarController: Called search parking spot service with" << search);
  } else {
    ROS_WARN_THROTTLE(1, "Failed to call search parking spot service.");
  }
}


void CarController::parkPerpendicular(const perception_msgs::PerpendicularParkingSpot &parking_spot) {
  navigation_msgs::ParkPerpendicular msg;
  msg.request.parking_spot = parking_spot;
  if (plan_path_parking_lot_client_.call(msg)) {
    ROS_INFO_STREAM("CarController: Called parking service");
  } else {
    ROS_WARN_THROTTLE(1, "Failed to call parking service");
  }
}

void CarController::straightPathOutOfStartBox(const bool enable) {
  std_srvs::SetBool msg;
  msg.request.data = enable;
  if (out_of_start_box_client_.call(msg)) {
    ROS_INFO_STREAM(
        "CarController: Called out_of_start_box_service with enable = " << std::boolalpha
                                                                        << enable);
  } else {
    ROS_WARN_STREAM_THROTTLE(
        1, "Failed to call out_of_start_box_service with enable = " << std::boolalpha << enable);
  }
}

void CarController::activateQrCodeDetection(const bool activate) {
  std_srvs::SetBool msg;
  msg.request.data = activate;
  if (activate_qr_code_detection_client_.call(msg)) {
    ROS_INFO_STREAM(
        "CarController: Called activate_qr_code_detection with activate = "
        << std::boolalpha << activate);
  } else {
    ROS_WARN_STREAM_THROTTLE(
        1, "Failed to call activate_qr_code_detection with activate = " << std::boolalpha << activate);
  }
}

void CarController::reverseOutOfParkingSpot(const nav_msgs::Path &path_reverse_out_of_parking_spot,
                                            const bool enable) {
  navigation_msgs::ReverseOutOfParkingSpot msg;
  msg.request.path_reverse_out_of_parking_spot = path_reverse_out_of_parking_spot;
  msg.request.enable = enable;
  if (reverse_out_of_parking_spot_client_.call(msg)) {
    ROS_INFO_STREAM(
        "CarController: Called reverse_out_of_parking_spot_service");
  } else {
    ROS_WARN_THROTTLE(1, "Failed to call reverse_out_of_parking_spot_service");
  }
}

void CarController::publishReversePathToWorldTransform(const Eigen::Affine3d &path_to_world_transform,
                                                       const ros::Time &stamp) {
  tf2::Stamped<Eigen::Affine3d> path_to_world_transform_stamped(
      path_to_world_transform, stamp, "world");

  geometry_msgs::TransformStamped path_to_world_transform_msg =
      tf2::toMsg(path_to_world_transform_stamped, "out_of_parking_spot_path");

  tf2_broadcaster_.sendTransform(path_to_world_transform_msg);
}

void CarController::resetCarController() {
  startDriving(0);

  max_speed_limitations.clear();
  std_msgs::Float32 msg;
  msg.data = std::numeric_limits<double>::infinity();
  velocity_publisher.publish(msg);

  stopLookAt();
  setRespectNoPassingZone(false);
  setDrivePastNextRoadClosure(true);
  resetPavlovBlinkerCommand();
}

void CarController::resetEnvironmentalModel() {
  std_srvs::Empty msg;
  if (reset_environmental_model_client_.call(msg)) {
    ROS_INFO_STREAM("CarController: Called reset environmental_model service.");
  } else {
    ROS_INFO_STREAM(
        "CarController: Failed to call reset environmental_model service.");
  }
}

void CarController::turnAt(const Eigen::Affine3d &pose, const TurnDirection direction) {
  navigation_msgs::TurnAt msg;
  msg.request.pose = tf2::toMsg(pose);
  switch (direction) {
    case TurnDirection::LEFT:
      msg.request.direction = navigation_msgs::TurnAt::Request::LEFT;
      break;
    case TurnDirection::RIGHT:
      msg.request.direction = navigation_msgs::TurnAt::Request::RIGHT;
      break;
    default:
      ROS_ERROR_STREAM(
          "Could not call turn at service with unknown turn direction " << direction << ".");
      return;
  }
  if (turn_at_client_.call(msg)) {
    ROS_INFO_STREAM("CarController: Called turn at service with direction "
                    << direction << ".");
  } else {
    ROS_WARN_STREAM_THROTTLE(
        1, "Failed to call turn at service with direction " << direction << ".");
  }
}

void CarController::resetTurning() {
  navigation_msgs::TurnAt msg;
  msg.request.direction = navigation_msgs::TurnAt::Request::DO_NOT_TURN;
  if (turn_at_client_.call(msg)) {
    ROS_INFO_STREAM("CarController: Called reset turn at service.");
  } else {
    ROS_WARN_STREAM_THROTTLE(1, "Failed to call reset turn at service.");
  }
}

void CarController::resetLaneDetection() {
  std_srvs::Empty msg;
  if (reset_lane_detection_client_.call(msg)) {
    ROS_INFO_STREAM("CarController: Called reset lane detection service.");
  } else {
    ROS_INFO_STREAM(
        "CarController: Failed to call reset lane detection service.");
  }
}

void CarController::resetPathPreprocessing() {
  std_srvs::Empty msg;
  if (reset_path_preprocessing_client_.call(msg)) {
    ROS_INFO_STREAM("CarController: Called reset path preprocessing service.");
  } else {
    ROS_INFO_STREAM(
        "CarController: Failed to call reset path preprocessing service.");
  }
}

void CarController::setPavlovBlinkerCommand(controller_msgs::BlinkerCommand::_command_type command) {
  navigation_msgs::PavlovBlinkerCommand msg;
  msg.request.pavlov_blinker_command.command = command;
  msg.request.enable = true;
  if (set_pavlov_blinker_command_client_.call(msg)) {
    ROS_INFO_STREAM(
        "CarController: Called setPavlovBlinkerCommand service with command: " << command);
  } else {
    ROS_INFO_STREAM(
        "CarController: Failed to call setPavlovBlinkerCommand service");
  }
}
void CarController::resetPavlovBlinkerCommand() {
  navigation_msgs::PavlovBlinkerCommand msg;
  msg.request.pavlov_blinker_command.command = controller_msgs::BlinkerCommand::NONE;
  msg.request.enable = false;
  if (set_pavlov_blinker_command_client_.call(msg)) {
    ROS_INFO_STREAM(
        "CarController: Called resetPavlovBlinkerCommand service with "
        "command: ");
  } else {
    ROS_INFO_STREAM(
        "CarController: Failed to call resetPavlovBlinkerCommand service");
  }
}


double CarController::getMinMaxSpeedOrInfinity() {
  const auto min_max_speed = std::min_element(max_speed_limitations.cbegin(),
                                              max_speed_limitations.cend(),
                                              compareSecond<unsigned long, double>);
  if (min_max_speed != max_speed_limitations.cend()) {
    return min_max_speed->second;
  } else {
    return std::numeric_limits<double>::infinity();
  }
}

void CarController::blinkInParkingSpot(const bool blink) {
  controller_msgs::BlinkerCommand blinker_msg;
  if (blink) {
    ROS_DEBUG_THROTTLE(1, "BLINK_BOTH");
    blinker_msg.command = controller_msgs::BlinkerCommand::BOTH;
  } else {
    ROS_DEBUG_THROTTLE(1, "STOP_BLINKING");
    blinker_msg.command = controller_msgs::BlinkerCommand::NONE;
  }
  controller_msgs::LightsCommand lights_msg;
  lights_msg.blinker = blinker_msg;
  light_publisher.publish(lights_msg);
}

void CarController::blinkLeftBeforeParkingSpot(const bool blink) {
  controller_msgs::BlinkerCommand blinker_msg;
  if (blink) {
    ROS_DEBUG_THROTTLE(1, "BLINK_LEFT");
    blinker_msg.command = controller_msgs::BlinkerCommand::LEFT;
  } else {
    ROS_DEBUG_THROTTLE(1, "STOP_BLINKING");
    blinker_msg.command = controller_msgs::BlinkerCommand::NONE;
  }
  controller_msgs::LightsCommand lights_msg;
  lights_msg.blinker = blinker_msg;
  light_publisher.publish(lights_msg);
}

void CarController::setHighBeam(const bool high_beam) {
  controller_msgs::LightsCommand lights_msg;
  lights_msg.blinker.command = controller_msgs::BlinkerCommand::NONE;
  if (high_beam) {
    ROS_DEBUG_THROTTLE(1, "activate high_beam");
    lights_msg.high_beam.data = true;
  } else {
    ROS_DEBUG_THROTTLE(1, "deactivate high_beam");
    lights_msg.high_beam.data = false;
  }
  light_publisher.publish(lights_msg);
}

void CarController::setRespectNoPassingZone(bool respect_no_passing_zone) {
  navigation_msgs::SetRespectNoPassingZones srv;
  srv.request.respect_no_passing_zones = respect_no_passing_zone;
  if (set_respect_no_passing_zone_service_.call(srv)) {
    ROS_INFO_STREAM_THROTTLE(1,
                             "CarController: Called respect no passing zone "
                             "with respect_no_passing_zone = "
                                 << std::boolalpha << respect_no_passing_zone);
  } else {
    ROS_WARN_THROTTLE(1, "Failed to call set respect no passing zone.");
  }
}
