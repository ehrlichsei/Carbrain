#include "obstacle_pavlov_node.h"
#include "boost_sml_helper.h"
#include "common/eigen_utils.h"
#include "common/math.h"
#include "common/node_creation_makros.h"
#include "navigation/no_passing_zone.h"

std::shared_ptr<ObstaclePavlovNode::StateMachine> createObstacleStateMachine(
    const std::shared_ptr<SharedData>& shared_data, StateMachineLogger& logger) {
  auto data_junction = std::make_shared<SM::DataJunctionSM>();
  auto data_road_closure = std::make_shared<SM::DataRoadClosure>();
  auto data_passing = std::make_shared<SM::DataPassing>();
  auto data_turn = std::make_shared<SM::DataTurn>();
  auto data_crosswalk = std::make_shared<SM::DataCrosswalk>();
  auto data_speed_limit = std::make_shared<SM::DataSpeedLimit>();
  auto data_qr_code = std::make_shared<SM::DataQRCode>();
  return std::make_shared<ObstaclePavlovNode::StateMachine>(
      shared_data,
      data_junction,
      data_road_closure,
      data_passing,
      data_turn,
      data_crosswalk,
      data_speed_limit,
      data_qr_code,
      SM::PassingSM(shared_data, data_passing),
      SM::InNoPassingZone(shared_data),
      SM::CrosswalkSM(shared_data, data_crosswalk),
      SM::JunctionSM(shared_data, data_junction),
      SM::RoadClosureSM(shared_data, data_road_closure),
      SM::SpeedLimitSM(shared_data, data_speed_limit),
      SM::TurningSM(shared_data, data_turn),
      SM::Driving(),
      SM::QRCodeFSM<SharedData, SM::Driving>(),
      logger);
}

ObstaclePavlovNode::ObstaclePavlovNode(ros::NodeHandle& node_handle)
    : NodeBase(node_handle),
      turn_checker(&parameter_handler_),
      logger_(),
      car_controller(node_handle_),
      diagnostics_iface(node_handle_),
      shared_data_(std::make_shared<SharedData>(
          &parameter_handler_, car_controller, diagnostics_iface)),
      obstacle_state_machine(createObstacleStateMachine(shared_data_, logger_)) {

  parameter_handler_.registerParam(PARAM_JUNCTION_AREA_OF_INTEREST_RIGHT_MIN_X);
  parameter_handler_.registerParam(PARAM_JUNCTION_AREA_OF_INTEREST_RIGHT_MIN_Y);
  parameter_handler_.registerParam(PARAM_JUNCTION_AREA_OF_INTEREST_RIGHT_MAX_X);
  parameter_handler_.registerParam(PARAM_JUNCTION_AREA_OF_INTEREST_RIGHT_MAX_Y);

  parameter_handler_.registerParam(PARAM_JUNCTION_AREA_OF_INTEREST_CENTER_MIN_X);
  parameter_handler_.registerParam(PARAM_JUNCTION_AREA_OF_INTEREST_CENTER_MIN_Y);
  parameter_handler_.registerParam(PARAM_JUNCTION_AREA_OF_INTEREST_CENTER_MAX_X);
  parameter_handler_.registerParam(PARAM_JUNCTION_AREA_OF_INTEREST_CENTER_MAX_Y);

  parameter_handler_.registerParam(PARAM_TURN_LEFT_PASSED_THRES_X);
  parameter_handler_.registerParam(PARAM_TURN_LEFT_PASSED_THRES_Y);
  parameter_handler_.registerParam(PARAM_TURN_RIGHT_PASSED_THRES_X);
  parameter_handler_.registerParam(PARAM_TURN_RIGHT_PASSED_THRES_Y);

  parameter_handler_.registerParam(PARAM_NO_PASSING_FINSIH_OVERTAKING);
  parameter_handler_.registerParam(PARAM_NO_PASSING_LENGTH_AREA_OF_INTEREST);

  parameter_handler_.registerParam(PARAM_CROSSWALK_PASSED_THRESH_X);

  parameter_handler_.registerParam(PARAM_VEHICLE_WIDTH);

  parameter_handler_.registerParam(PARAM_ENABLE_QR_CODE_DETECTION);
}

void ObstaclePavlovNode::startModule() {
  pavlov_node_helper_.startModule<StateMachine>(obstacle_state_machine, node_handle_);
  timer_debug_msg_ = node_handle_.createTimer(
      ros::Duration(0.2), &ObstaclePavlovNode::timerDebugMsgCallback, this);
  loadParameters();
  crosswalks_subscriber = node_handle_.subscribe(
      TOPIC_CROSSWALKS, 1, &ObstaclePavlovNode::handleCrosswalks, this);
  junctions_subscriber = node_handle_.subscribe(
      TOPIC_JUNCTIONS, 1, &ObstaclePavlovNode::handleJunctions, this);
  road_closures_subscriber = node_handle_.subscribe(
      TOPIC_ROAD_CLOSURES, 1, &ObstaclePavlovNode::handleRoadClosures, this);
  arrow_markings_subscriber = node_handle_.subscribe(
      TOPIC_ARROW_MARKINGS, 1, &ObstaclePavlovNode::handleArrowMarkings, this);
  full_corridor_subscriber = node_handle_.subscribe(
      "full_corridor", 1, &ObstaclePavlovNode::handleFullCorridor, this);
  obstacles_subscriber = node_handle_.subscribe(
      "obstacles", 1, &ObstaclePavlovNode::handleObstacles, this);
  speed_limit_subscriber = node_handle_.subscribe(
      TOPIC_SPEED_LIMIT_MARKINGS, 1, &ObstaclePavlovNode::handleSpeedLimitMarkings, this);
  unidentified_subscriber = node_handle_.subscribe(
      "unidentified", 1, &ObstaclePavlovNode::handleUnidentified, this);
  no_passing_zone_subscriber = node_handle_.subscribe(
      "no_passing_zones", 1, &ObstaclePavlovNode::handleNoPassingZones, this);
  reset_subscriber =
      node_handle_.subscribe("auto_reset", 1, &ObstaclePavlovNode::handleReset, this);
  if (!parameter_handler_.getParam(PARAM_ENABLE_QR_CODE_DETECTION)) {
    obstacle_state_machine->process_event(EventStart());
  }

  state_machine_publisher_ =
      node_handle_.advertise<mission_control_msgs::StateMachineStates>(
          "current_states", 1);
}

void ObstaclePavlovNode::stopModule() {
  pavlov_node_helper_.stopModule();
  crosswalks_subscriber.shutdown();
  junctions_subscriber.shutdown();
  road_closures_subscriber.shutdown();
  arrow_markings_subscriber.shutdown();
  speed_limit_subscriber.shutdown();
  full_corridor_subscriber.shutdown();
  obstacles_subscriber.shutdown();
  unidentified_subscriber.shutdown();
  diagnostics_publisher.shutdown();
  reset_subscriber.shutdown();

  driving_corridor_checker.reset();
}

const std::string ObstaclePavlovNode::getName() {
  return std::string("obstacle_pavlov");
}

void ObstaclePavlovNode::loadParameters() {
  // Areas of interest
  junction_right_roi = Eigen::AlignedBox2d(
      Eigen::Vector2d(
          parameter_handler_.getParam(PARAM_JUNCTION_AREA_OF_INTEREST_RIGHT_MIN_X),
          parameter_handler_.getParam(PARAM_JUNCTION_AREA_OF_INTEREST_RIGHT_MIN_Y)),
      Eigen::Vector2d(
          parameter_handler_.getParam(PARAM_JUNCTION_AREA_OF_INTEREST_RIGHT_MAX_X),
          parameter_handler_.getParam(PARAM_JUNCTION_AREA_OF_INTEREST_RIGHT_MAX_Y)));

  junction_center_roi = Eigen::AlignedBox2d(
      Eigen::Vector2d(
          parameter_handler_.getParam(PARAM_JUNCTION_AREA_OF_INTEREST_CENTER_MIN_X),
          parameter_handler_.getParam(PARAM_JUNCTION_AREA_OF_INTEREST_CENTER_MIN_Y)),
      Eigen::Vector2d(
          parameter_handler_.getParam(PARAM_JUNCTION_AREA_OF_INTEREST_CENTER_MAX_X),
          parameter_handler_.getParam(PARAM_JUNCTION_AREA_OF_INTEREST_CENTER_MAX_Y)));

  // Turn passing points
  turn_left_passed_thres[0] = parameter_handler_.getParam(PARAM_TURN_LEFT_PASSED_THRES_X);
  turn_left_passed_thres[1] = parameter_handler_.getParam(PARAM_TURN_LEFT_PASSED_THRES_Y);
  turn_left_passed_thres[2] = 0;
  turn_right_passed_thres[0] = parameter_handler_.getParam(PARAM_TURN_RIGHT_PASSED_THRES_X);
  turn_right_passed_thres[1] = parameter_handler_.getParam(PARAM_TURN_RIGHT_PASSED_THRES_Y);
  turn_right_passed_thres[2] = 0;
}

void ObstaclePavlovNode::handleReset(const std_msgs::Empty::ConstPtr&) {
  ROS_INFO_STREAM("Resetting node.");
  driving_corridor_checker.reset();
  obstacle_state_machine->process_event(EventReset());
  shared_data_->getCarController().resetCarController();
  obstacle_state_machine = createObstacleStateMachine(shared_data_, logger_);
  obstacle_state_machine->process_event(EventStart());
  loadParameters();
}

void ObstaclePavlovNode::handleCrosswalks(const navigation_msgs::Crosswalks::ConstPtr& msg) {
  if (msg->sub_messages.empty()) {
    obstacle_state_machine->process_event(EventNoCrosswalk());
    return;
  }
  const auto sorted_crosswalks = sortMessages(*msg);
  if (sorted_crosswalks.sub_messages.empty()) {
    return;
  }
  const auto first_crosswalk = sorted_crosswalks.sub_messages[0];

  double distance = calcEuclidianDistanceInFront(first_crosswalk.pose.position);
  Eigen::Affine3d crosswalk_pose;
  tf2::fromMsg(first_crosswalk.pose, crosswalk_pose);
  if (first_crosswalk.pedestrian_waiting) {
    obstacle_state_machine->process_event(
        EventCrosswalkWithPedestriansDetected(distance, first_crosswalk.id));
    return;
  } else {
    const double passed_thresh_x =
        parameter_handler_.getParam(PARAM_CROSSWALK_PASSED_THRESH_X);
    using namespace passing_condition;
    Eigen::Affine3d vehicle_pose;
    if (!pavlov_node_helper_.vehicleToWorld(Eigen::Affine3d::Identity(), vehicle_pose)) {
      return;
    }
    if (passing_point_checker.checkPoint(
            vehicle_pose,
            crosswalk_pose,
            both(vehicleCoordinatesInEntityCosy(haveXGreaterThan(passed_thresh_x)),
                 entityCoordinatesInVehicleCosy(haveXSmallerThan(-passed_thresh_x))))) {
      obstacle_state_machine->process_event(EventCrosswalkPassed());
    } else {
      obstacle_state_machine->process_event(
          EventCrosswalkFreeDetected(distance, first_crosswalk.id));
    }
    return;
  }
}

void ObstaclePavlovNode::handleJunctions(const navigation_msgs::Junctions::ConstPtr& msg) {
  turn_checker.setJunctions(*msg);
  if (msg->sub_messages.empty()) {
    obstacle_state_machine->process_event(EventNoJunction());
    return;
  }
  const auto sorted_junctions = sortMessages(*msg);
  for (auto& junction : sorted_junctions.sub_messages) {
    double distance = calcEuclidianDistanceInFront(junction.stopping_point);
    if (distance < -0.2) {
      continue;
    }
    bool obstacle_waiting = junction.obstacle_waiting;
    switch (junction.junction_type) {
      case perception_msgs::Junction::TYPE_STOPLINE_LEFT: {
        break;
      }
      case perception_msgs::Junction::TYPE_STOPLINE_RIGHT: {
        obstacle_state_machine->process_event(
            EventStopJunction(distance, junction.id));
        if (obstacle_waiting) {
          obstacle_state_machine->process_event(EventJunctionBlockedByObstacle());
        } else {
          obstacle_state_machine->process_event(EventEmptyJunction());
        }
        return;
      }
      case perception_msgs::Junction::TYPE_GIVEWAY_LEFT: {
        break;
      }
      case perception_msgs::Junction::TYPE_GIVEWAY_RIGHT: {
        obstacle_state_machine->process_event(
            EventHoldJunction(distance, junction.id));
        if (obstacle_waiting) {
          obstacle_state_machine->process_event(EventJunctionBlockedByObstacle());
        } else {
          obstacle_state_machine->process_event(EventEmptyJunction());
        }
        return;
      }
      default: {
        ROS_ERROR_STREAM("Unhandled junction state "
                         << junction.junction_type << " will not be handled.");
        break;
      }
    }

    ROS_INFO_STREAM_THROTTLE(1,
                             "(Throttled) Received junction "
                                 << junction.id << " with distance " << distance
                                 << ", obstacle waiting: " << obstacle_waiting);
    // For debugging:
    publishPoseTransform(junction.pose, "junction");
  }
}

void ObstaclePavlovNode::handleRoadClosures(const navigation_msgs::RoadClosures::ConstPtr& msg) {
  if (msg->sub_messages.empty()) {
    obstacle_state_machine->process_event(EventNoRoadClosure());
    return;
  }


  const auto sorted_road_closures = [&] {
    navigation_msgs::RoadClosures r = *msg;
    boost::sort(r.sub_messages, [this](const auto& a, const auto& b) {
      return std::abs(this->calcEuclidianDistanceInFront(a.hull_polygon[0])) <
             std::abs(this->calcEuclidianDistanceInFront(b.hull_polygon[0]));
    });
    return r;
  }();
  for (navigation_msgs::RoadClosure road_closure : sorted_road_closures.sub_messages) {
    double distance = calcEuclidianDistanceInFront(road_closure.hull_polygon[0]);
    if (distance < -0.2) {
      continue;
    }
    if (road_closure.has_obstacle) {
      obstacle_state_machine->process_event(
          EventRoadClosureBlockedDetected(distance, road_closure.id));
    } else {
      obstacle_state_machine->process_event(EventRoadClosureFreeDetected());
    }

    ROS_INFO_STREAM_THROTTLE(1,
                             "(Throttled) Received road closure with distance "
                                 << distance << ". Has obstacle: " << std::boolalpha
                                 << static_cast<bool>(road_closure.has_obstacle));
  }
}

void ObstaclePavlovNode::handleArrowMarkings(const perception_msgs::ArrowMarkings::ConstPtr& msg) {
  if (msg->sub_messages.empty()) {
    obstacle_state_machine->process_event(EventNoTurn());
    return;
  }
  const auto sorted_arrows = sortMessages(*msg);

  for (auto& arrow : sorted_arrows.sub_messages) {
    TurnDirection direction;
    if (arrow.direction == perception_msgs::ArrowMarking::LEFT) {
      direction = TurnDirection::LEFT;
    } else if (arrow.direction == perception_msgs::ArrowMarking::RIGHT) {
      direction = TurnDirection::RIGHT;
    } else {
      ROS_WARN_STREAM("Arrow with unknown direction "
                      << static_cast<int>(arrow.direction) << " will be ignored.");
      continue;
    }
    Eigen::Affine3d junction_to_world;
    if (turn_checker.findTurnJunctionPose(arrow, junction_to_world)) {
      double distance = calcEuclidianDistanceInFront(junction_to_world.translation());

      ROS_INFO_STREAM_THROTTLE(1, "Detected turn in " << distance << " m." << std::endl);

      using namespace passing_condition;
      PassingCondition check_turn_completed;
      if (direction == TurnDirection::LEFT) {
        check_turn_completed = both(
            vehicleCoordinatesInEntityCosy(haveXGreaterThan(turn_left_passed_thres[0])),
            vehicleCoordinatesInEntityCosy(haveYGreaterThan(turn_left_passed_thres[1])));
      } else if (direction == TurnDirection::RIGHT) {
        check_turn_completed = both(
            vehicleCoordinatesInEntityCosy(haveXGreaterThan(turn_right_passed_thres[0])),
            vehicleCoordinatesInEntityCosy(haveYSmallerThan(-turn_right_passed_thres[1])));
      }
      Eigen::Affine3d vehicle_pose;
      if (!pavlov_node_helper_.vehicleToWorld(Eigen::Affine3d::Identity(), vehicle_pose)) {
        return;
      }
      if (passing_point_checker.checkPoint(vehicle_pose, junction_to_world, check_turn_completed)) {
        ROS_INFO_STREAM("Passed turn point.");
        obstacle_state_machine->process_event(EventTurnPassed(arrow.id));
      } else if (distance > 0) {
        obstacle_state_machine->process_event(EventTurnDetected(
            arrow.id, junction_to_world, distance, direction, msg->header.stamp));
      }

      publishPoseTransform(turn_checker.getLatestWorldToTurnJunctionTransform(),
                           "turn");  // For debugging
      return;
    }
  }
}

void ObstaclePavlovNode::handleSpeedLimitMarkings(const perception_msgs::SpeedLimitMarkings::ConstPtr& msg) {
  if (msg->sub_messages.empty()) {
    return;
  }
  const auto sorted_speed_limits = sortMessages(*msg);
  const auto next_speed_limit = sorted_speed_limits.sub_messages.front();
  long id = next_speed_limit.id;
  Eigen::Affine3d limit_pose;
  tf2::fromMsg(next_speed_limit.pose, limit_pose);
  using namespace passing_condition;
  auto check_car_has_passed_marking =
      entityCoordinatesInVehicleCosy(haveXSmallerThan(0.3));
  Eigen::Affine3d vehicle_pose;
  if (!pavlov_node_helper_.vehicleToWorld(Eigen::Affine3d::Identity(), vehicle_pose)) {
    return;
  }
  if (!next_speed_limit.limit_relieved) {
    // Start of limit zone.
    if (passing_point_checker.checkPoint(vehicle_pose, limit_pose, check_car_has_passed_marking)) {
      ROS_INFO_STREAM("Passed speed limit start.");
      obstacle_state_machine->process_event(
          EventSpeedLimitStartPassed(id, next_speed_limit.speed_limit));
    }
  } else {
    // End of limit zone.
    if (passing_point_checker.checkPoint(vehicle_pose, limit_pose, check_car_has_passed_marking)) {
      ROS_INFO_STREAM("Passed speed limit end.");
      obstacle_state_machine->process_event(EventSpeedLimitEndPassed(id));
    }
  }
}

void ObstaclePavlovNode::handleFullCorridor(const navigation_msgs::DrivingCorridor::ConstPtr& full_corridor_msg) {
  DrivingCorridor full_corridor = DrivingCorridor::fromMessage(full_corridor_msg);
  Eigen::Affine3d vehicle_pose;
  if (!pavlov_node_helper_.vehicleToWorld(Eigen::Affine3d::Identity(), vehicle_pose)) {
    return;
  }
  const double vehicle_width = parameter_handler_.getParam(PARAM_VEHICLE_WIDTH);
  const double no_passing_finish_overtaking =
      parameter_handler_.getParam(PARAM_NO_PASSING_FINSIH_OVERTAKING);
  const double y_addition_to_pose = vehicle_width / no_passing_finish_overtaking;
  if (full_corridor.isPointOnRightLane(
          vehicle_pose * Eigen::Vector3d(0, y_addition_to_pose, 0))) {
    obstacle_state_machine->process_event(EventOnRightLane());
  } else {
    obstacle_state_machine->process_event(EventNotOnRightLane());
  }
  driving_corridor_checker.handleFullCorridor(full_corridor_msg);
}

void ObstaclePavlovNode::handleObstacles(const navigation_msgs::Obstacles::ConstPtr& obstacles) {
  driving_corridor_checker.handleObstacles(*obstacles);
  Eigen::Affine3d vehicle_pose;
  if (!pavlov_node_helper_.vehicleToWorld(Eigen::Affine3d::Identity(), vehicle_pose)) {
    return;
  }
  const double no_passing_length_area_of_interest =
      parameter_handler_.getParam(PARAM_NO_PASSING_LENGTH_AREA_OF_INTEREST);
  double distance_to_next_obstacle = 0;
  double velocity_next_obstacle = 0;
  if (driving_corridor_checker.hasObstacleOnRightLane(vehicle_pose *
                                                          Eigen::Vector3d(0.25, 0, 0),  // start searching for obstacles 20cm ahead of the vehicle pose (is still behind the front of the vehicle)
                                                      no_passing_length_area_of_interest,
                                                      distance_to_next_obstacle,
                                                      velocity_next_obstacle)) {
    obstacle_state_machine->process_event(EventObstacleAheadDetectedRight(
        velocity_next_obstacle, distance_to_next_obstacle));
  } else {
    obstacle_state_machine->process_event(EventNoObstacleAheadDetectedRight());
  }
}


void ObstaclePavlovNode::handleUnidentified(const perception_msgs::Unidentifieds::ConstPtr& unidentifieds) {
  if (unidentifieds->sub_messages.empty()) {
    obstacle_state_machine->process_event(
        EventNoUnidentifiedObject(unidentifieds->header.stamp));
    return;
  }
  Eigen::Affine3d vehicle_pose;
  if (!pavlov_node_helper_.vehicleToWorld(Eigen::Affine3d::Identity(), vehicle_pose)) {
    return;
  }
  std::vector<double> distance;
  // std::vector<Eigen::Vector3d> start_positions;
  for (const auto& unidentified : unidentifieds->sub_messages) {
    boost::transform(unidentified.hull_polygon,
                     std::back_inserter(distance),
                     [&vehicle_pose](const auto& geom_point) {
                       Eigen::Vector3d start_position;
                       tf2::fromMsg(geom_point, start_position);
                       return (start_position - vehicle_pose.translation()).norm();
                     });
  }
  obstacle_state_machine->process_event(
      EventUnidentifiedObject(*std::min_element(distance.begin(), distance.end())));
}

void ObstaclePavlovNode::handleNoPassingZones(const navigation_msgs::NoPassingZones::ConstPtr& no_passing_zones) {
  if (no_passing_zones->sub_messages.empty()) {
    obstacle_state_machine->process_event(
        EventNoPassingMessageEmpty(no_passing_zones->header.stamp));
    return;
  }
  Eigen::Affine3d vehicle_pose;
  if (!pavlov_node_helper_.vehicleToWorld(Eigen::Affine3d::Identity(), vehicle_pose)) {
    return;
  }
  double dist_to_next_obstacle = 0;
  double velocity_next_obstacle = 0;
  for (const auto& zone : no_passing_zones->sub_messages) {
    Eigen::Vector3d start = Eigen::Vector3d::Zero(), end = Eigen::Vector3d::Zero();
    tf2::fromMsg(zone.start, start);
    tf2::fromMsg(zone.end, end);
    if ((vehicle_pose.inverse() * end).x() > 0.1 ||
        driving_corridor_checker.hasObstacleOnRightLane(
            end, NoPassingZone::safety_margin_after_zone, dist_to_next_obstacle, velocity_next_obstacle)) {
      obstacle_state_machine->process_event(EventNoPassingZone(start, end, vehicle_pose));
      return;
    }
  }
  obstacle_state_machine->process_event(
      EventNoPassingMessageEmpty(no_passing_zones->header.stamp));
}



double ObstaclePavlovNode::calcEuclidianDistanceInFront(const Eigen::Vector3d& p_world) {
  Eigen::Vector3d p_vehicle = Eigen::Vector3d::Zero();
  if (pavlov_node_helper_.worldToVehicle(p_world, p_vehicle)) {
    return p_vehicle.norm() * common::sgn(p_vehicle.x());
  }
  return std::numeric_limits<double>::infinity();
}


double ObstaclePavlovNode::calcEuclidianDistanceInFront(const geometry_msgs::Point& p) {
  Eigen::Vector3d p_world = Eigen::Vector3d::Zero();
  tf2::fromMsg(p, p_world);
  return calcEuclidianDistanceInFront(p_world);
}


void ObstaclePavlovNode::publishPoseTransform(const geometry_msgs::Pose& pose,
                                              const std::string& name) {
  Eigen::Affine3d tf;
  tf2::fromMsg(pose, tf);
  tf2::Stamped<Eigen::Affine3d> stamped_tf(tf, ros::Time::now(), "world");
  geometry_msgs::TransformStamped msg = tf2::toMsg(stamped_tf, name);
  tf_broadcaster.sendTransform(msg);
}

void ObstaclePavlovNode::publishPoseTransform(const Eigen::Affine3d& world_to_pose,
                                              const std::string& name) {
  tf2::Stamped<Eigen::Affine3d> stamped_tf(
      world_to_pose, ros::Time::now(), "world");
  geometry_msgs::TransformStamped msg = tf2::toMsg(stamped_tf, name);
  tf_broadcaster.sendTransform(msg);
}

void ObstaclePavlovNode::timerDebugMsgCallback(const ros::TimerEvent&) {
  publishCurrentStates();
}

void ObstaclePavlovNode::publishCurrentStates() {
  mission_control_msgs::StateMachineStates current_states;
  sml_helper::call_all_states<ObstaclePavlovNode::StateMachine, SM::ObstacleStateMachine>(
      current_states, obstacle_state_machine);


  state_machine_publisher_.publish(current_states);
}

CREATE_NODE(ObstaclePavlovNode)
