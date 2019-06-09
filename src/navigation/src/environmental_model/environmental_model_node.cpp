#include "environmental_model_node.h"
#include <common/tf2_eigen_addon.h>
#include "common/node_creation_makros.h"
#include "road_elements/road_element.h"
#include "common/angle_conversions.h"
#include <common/macros.h>
#include "common/console_colors.h"
#include "road_element_visitors/create_messages_visitor.h"
#include "road_element_visitors/road_element_visitor.h"
#include "road_element_visitors/look_at_visitor.h"
#include "road_element_visitors/look_at_result_visitor.h"
#include "road_element_visitors/no_passing_zone_visitor.h"
#include "road_element_visitors/consistency_check_visitor.h"


namespace environmental_model {

EnvironmentalModelNode::EnvironmentalModelNode(ros::NodeHandle& node_handle)
    : NodeBase(node_handle),
      tf_listener(tf_buffer),
      environmental_model_(&parameter_handler_),
      obstacle_look_at_client("look_for_obstacles", false),
      pedestrian_look_at_client("look_for_pedestrians", false),
      driving_corridor_(std::make_shared<DrivingCorridor>()),
      time_last_road_watcher_msg_(0),
      look_at_visitor_parameter_(std::make_shared<LookAtVisitorParameter>(&parameter_handler_)) {
}

void EnvironmentalModelNode::startModule() {
  reset_subscriber = node_handle_.subscribe(
      "auto_reset", 1, &EnvironmentalModelNode::handleReset, this);
  full_corridor_subscriber = node_handle_.subscribe(
      "full_corridor", 1, &EnvironmentalModelNode::handleFullCorridor, this);
  crosswalks_publisher =
      node_handle_.advertise<navigation_msgs::Crosswalks>("crosswalk", 1);
  junctions_publisher =
      node_handle_.advertise<navigation_msgs::Junctions>("junction", 1);
  road_closures_publisher =
      node_handle_.advertise<navigation_msgs::RoadClosures>("road_closure", 1);
  arrow_markings_publisher =
      node_handle_.advertise<perception_msgs::ArrowMarkings>("arrow_marking", 1);
  obstacles_publisher =
      node_handle_.advertise<perception_msgs::Obstacles>("obstacle", 1);
  nav_obstacles_publisher =
      node_handle_.advertise<navigation_msgs::Obstacles>("nav_obstacle", 1);
  start_line_publisher =
      node_handle_.advertise<perception_msgs::StartLines>("startline", 1);
  speed_limit_publisher = node_handle_.advertise<perception_msgs::SpeedLimitMarkings>(
      "speed_limit_marking", 1);
  unidentified_publisher =
      node_handle_.advertise<perception_msgs::Unidentifieds>("unidentified", 1);
  no_passing_zone_publisher = node_handle_.advertise<navigation_msgs::NoPassingZones>(
      "no_passing_zones", 1);
  tracking_element_publisher = node_handle_.advertise<navigation_msgs::TrackingElements>(
      "tracking_elements", 1);
  look_at_regions_obstacle_publisher =
      node_handle_.advertise<perception_msgs::LookAtRegions>("obstacles_roi", 1);
  look_at_regions_pedestrian_publisher =
      node_handle_.advertise<perception_msgs::LookAtRegions>("pedestrians_roi", 1);
  road_objects_subscriber = node_handle_.subscribe(
      "road_objects", 1, &EnvironmentalModelNode::handleRoadWatcherMsg, this);

  ir_front_sub_ = node_handle_.subscribe(
      "infrared_sensor_front", 1, &EnvironmentalModelNode::handleTOFSensor, this);
  ir_middle_sub_ = node_handle_.subscribe(
      "infrared_sensor_middle", 1, &EnvironmentalModelNode::handleTOFSensor, this);
  ir_back_sub_ = node_handle_.subscribe(
      "infrared_sensor_back", 1, &EnvironmentalModelNode::handleTOFSensor, this);

  ir_ahead_sub_ = node_handle_.subscribe(
      "infrared_sensor_ahead", 1, &EnvironmentalModelNode::handleTOFAheadMsg, this);

  reset_server_ = node_handle_.advertiseService(
      "reset", &EnvironmentalModelNode::resetService, this);
}

void EnvironmentalModelNode::stopModule() {
  reset_subscriber.shutdown();
  full_corridor_subscriber.shutdown();
  road_objects_subscriber.shutdown();
  crosswalks_publisher.shutdown();
  junctions_publisher.shutdown();
  road_closures_publisher.shutdown();
  arrow_markings_publisher.shutdown();
  obstacles_publisher.shutdown();
  speed_limit_publisher.shutdown();
  unidentified_publisher.shutdown();
  no_passing_zone_publisher.shutdown();
  tracking_element_publisher.shutdown();
  ir_front_sub_.shutdown();
  ir_middle_sub_.shutdown();
  ir_back_sub_.shutdown();
  reset_server_.shutdown();
}

void EnvironmentalModelNode::handleReset(const std_msgs::Empty::ConstPtr&) {
  environmental_model_.reset();
  environmental_model_.updateParameter();
  look_at_visitor_parameter_->updateParameter(&parameter_handler_);
}

bool EnvironmentalModelNode::resetService(std_srvs::Empty::Request&,
                                          std_srvs::Empty::Response&) {
  environmental_model_.reset();
  return true;
}

void EnvironmentalModelNode::handleFullCorridor(
    const navigation_msgs::DrivingCorridor::ConstPtr& full_corridor_msg) {
  driving_corridor_ =
      std::make_shared<DrivingCorridor>(DrivingCorridor::fromMessage(full_corridor_msg));
  environmental_model_.updateFullCorridor(*driving_corridor_);
}

void EnvironmentalModelNode::handleNoPassingZones(const perception_msgs::NoPassingZones& no_passing_zones,
                                                  const Eigen::Affine3d& vehicle_pose) {
  NoPassingZoneVisitor no_passing_zone_visitor;
  environmental_model_.visitElements(no_passing_zone_visitor);
  const auto current_zones = environmental_model_.getNoPassingZones().calcNoPassingZones(
      vehicle_pose, no_passing_zones, no_passing_zone_visitor.getNoPassingZones());
  no_passing_zone_publisher.publish(current_zones);
}


void EnvironmentalModelNode::handleRoadWatcherMsg(const perception_msgs::RoadObjects::ConstPtr& road_objects_msg) {
  ros::WallTime start = ros::WallTime::now();
  auto stamp = road_objects_msg->arrow_markings.header.stamp;
  std::vector<std::shared_ptr<RoadElement>> new_road_elements;
  insertElementsFromMsg<Crosswalk>(new_road_elements, road_objects_msg->crosswalks);
  insertElementsFromMsg<Junction>(new_road_elements, road_objects_msg->junctions);
  insertElementsFromMsg<RoadClosure>(new_road_elements, road_objects_msg->road_closures);
  insertElementsFromMsg<Arrow>(new_road_elements, road_objects_msg->arrow_markings);
  insertElementsFromMsg<SpeedLimit>(new_road_elements, road_objects_msg->speed_limit_markings);
  insertElementsFromMsg<Obstacle>(new_road_elements, road_objects_msg->obstacles);
  insertElementsFromMsg<StartLine>(new_road_elements, road_objects_msg->start_lines);
  // new_road_elements.insert(
  //  new_road_elements.end(), unidentifieds.begin(), unidentifieds.end());
  Eigen::Affine3d vehicle_pose = Eigen::Affine3d::Identity();
  if (!frameToWorld(Eigen::Affine3d::Identity(), vehicle_pose, "vehicle")) {
    return;
  }
  const double time_diff_sec = (stamp - time_last_road_watcher_msg_).nsec * 1e-9;
  environmental_model_.predict(time_diff_sec, vehicle_pose);
  time_last_road_watcher_msg_ = stamp;

  environmental_model_.update(new_road_elements, road_objects_msg->unidentifieds, vehicle_pose);


  LookAtVisitor look_at_visitor(
      look_at_visitor_parameter_, road_objects_msg->obstacles, driving_corridor_, vehicle_pose);

  environmental_model_.visitElements(look_at_visitor);

  look_at_regions_obstacle_publisher.publish(look_at_visitor.getObstacleRois());
  look_at_regions_pedestrian_publisher.publish(look_at_visitor.getPedestriansRois());
  if (look_at_visitor.hasObstacleRois()) {
    obstacle_look_at_client.sendGoal(
        look_at_visitor.getObstacleActionGoal(),
        boost::bind(&EnvironmentalModelNode::obstacleLookAtResult, this, _1, _2),
        boost::bind(&EnvironmentalModelNode::obstacleLookAtActive, this),
        boost::bind(&EnvironmentalModelNode::obstacleLookAtFeedback, this, _1));
  }
  if (look_at_visitor.hasPedestrianRois()) {
    pedestrian_look_at_client.sendGoal(
        look_at_visitor.getPedestrianActionGoal(),
        boost::bind(&EnvironmentalModelNode::pedestrianLookAtResult, this, _1, _2),
        boost::bind(&EnvironmentalModelNode::pedestrianLookAtActive, this),
        boost::bind(&EnvironmentalModelNode::pedestrianLookAtFeedback, this, _1));
  }
  environmental_model_.deleteAllObstaclesInRoi(look_at_visitor.getObstacleRois());

  const int elapsed = (ros::WallTime::now() - start).toNSec() / 1000;
  ROS_DEBUG_STREAM("handleRoadWatcherMsg took " << COLOR_LIGHT_GREY << elapsed
                                                << COLOR_DEBUG << " us");
  // Publish updated state.

  publishRoadElements(stamp);
  handleNoPassingZones(road_objects_msg->no_passing_zones, vehicle_pose);
}

void EnvironmentalModelNode::handleTOFSensor(const sensor_msgs::Range::ConstPtr& tof) {
  Eigen::Affine3d tof_pose(Eigen::Affine3d::Identity());
  if (!frameToWorld(Eigen::Affine3d::Identity(), tof_pose, tof->header.frame_id)) {
    return;
  }
  if (!driving_corridor_ || driving_corridor_->empty()) {
    return;
  }
  const Eigen::Vector3d reflection_point = tof_pose * Eigen::Vector3d(tof->range, 0, 0);
  const Gate neares_gate =
      *common::min_score(*driving_corridor_,
                         [&tof_pose](const Gate& gate) {
                           return (gate.getCenter() - tof_pose.translation()).norm();
                         });
  const Eigen::Vector3d ray_direction = (tof_pose.linear() * Eigen::Vector3d::UnitX());
  const Eigen::Vector3d corridor_direction =
      neares_gate.getVectorLeftToRight().unitOrthogonal();
  const double max_degree = 30;
  if (std::abs(corridor_direction.dot(ray_direction)) >
      std::cos((90 - max_degree) * 2 * M_PI / 360.0)) {
    ROS_WARN_STREAM_THROTTLE(0.5,
                             "can't use tof message because difference of car "
                             "direction and lane direction is too high. "
                             "std::abs(direction.dot(ray_direction)) is "
                                 << std::abs(corridor_direction.dot(ray_direction)));
    return;
  }

  const double dist_to_border =
      (neares_gate.getRightLaneBoundary() - reflection_point)
          .dot(neares_gate.getVectorLeftToRight().normalized());
  TOFMeasurement measurement{tof_pose, reflection_point, dist_to_border};
  environmental_model_.receivedTOFMeasurement(measurement);
}

void EnvironmentalModelNode::handleTOFAheadMsg(const sensor_msgs::Range::ConstPtr& tof_ahead) {
  if (tof_ahead->range > 3.0) {
    return;
  }
  Eigen::Affine3d tof_pose(Eigen::Affine3d::Identity());
  if (!frameToWorld(Eigen::Affine3d::Identity(), tof_pose, tof_ahead->header.frame_id)) {
    return;
  }
  if (!driving_corridor_ || driving_corridor_->empty()) {
    return;
  }
  const Eigen::Vector3d reflection_point =
      tof_pose * Eigen::Vector3d(tof_ahead->range, 0, 0);
  const Gate neares_gate =
      *common::min_score(*driving_corridor_,
                         [&reflection_point](const Gate& gate) {
                           return (gate.getCenter() - reflection_point).norm();
                         });
  const double max_param_dist_to_border = 0.1;
  if (1 - neares_gate.toParam(reflection_point) < max_param_dist_to_border ||
      neares_gate.toParam(reflection_point) < max_param_dist_to_border) {
    return;
  }

  TOFMeasurementAhead measurement{tof_pose, reflection_point};
  environmental_model_.receivedTOFMeasurementAhead(measurement);
}

const std::string EnvironmentalModelNode::getName() {
  return std::string("environmental_model");
}


bool EnvironmentalModelNode::frameToWorld(const Eigen::Affine3d& frame_pose,
                                          Eigen::Affine3d& world_pose,
                                          const std::string& frame_id) {
  try {
    auto frame_to_world = tf_buffer.lookupTransform("world", frame_id, ros::Time(0));
    tf2::doTransform(frame_pose, world_pose, frame_to_world);
    return true;
  } catch (const tf2::TransformException& ex) {
    ROS_WARN_STREAM("Can NOT transform " << frame_id << " to world: " << ex.what());
  }
  return false;
}

void EnvironmentalModelNode::obstacleLookAtResult(
    const actionlib::SimpleClientGoalState& /*state*/,
    const perception_msgs::LookForObstaclesResult::ConstPtr& result) {
  if (result == nullptr) {
    return;
  }
  LookAtResultVisitor look_at_result_visitor_;
  look_at_result_visitor_.setObstacles(result->obstacles);
  environmental_model_.visitElements(look_at_result_visitor_);
}

void EnvironmentalModelNode::obstacleLookAtActive() {}

void EnvironmentalModelNode::obstacleLookAtFeedback(
    const perception_msgs::LookForObstaclesFeedback::ConstPtr& /*feedback*/) {}

void EnvironmentalModelNode::pedestrianLookAtResult(
    const actionlib::SimpleClientGoalState& /*state*/,
    const perception_msgs::LookForPedestriansResult::ConstPtr& result) {
  if (result == nullptr) {
    return;
  }
  LookAtResultVisitor look_at_result_visitor_;
  look_at_result_visitor_.setPedestrians(result->pedestrians);
  environmental_model_.visitElements(look_at_result_visitor_);
}

void EnvironmentalModelNode::pedestrianLookAtActive() {}

void EnvironmentalModelNode::pedestrianLookAtFeedback(
    const perception_msgs::LookForPedestriansFeedback::ConstPtr& /*feedback*/) {}

void EnvironmentalModelNode::publishRoadElements(const ros::Time& stamp) {
  CreateMessagesVisitor create_messages_visitor(stamp);
  environmental_model_.visitElements(create_messages_visitor);
  crosswalks_publisher.publish(create_messages_visitor.crosswalks);
  junctions_publisher.publish(create_messages_visitor.junctions);
  road_closures_publisher.publish(create_messages_visitor.road_closures);
  arrow_markings_publisher.publish(create_messages_visitor.arrow_markings);
  obstacles_publisher.publish(create_messages_visitor.obstacles);
  nav_obstacles_publisher.publish(create_messages_visitor.navigation_obstacles);
  start_line_publisher.publish(create_messages_visitor.start_lines);
  speed_limit_publisher.publish(create_messages_visitor.speed_limits);
  unidentified_publisher.publish(create_messages_visitor.unidentifieds);

  navigation_msgs::TrackingElements tracking_elements;
  tracking_elements.header.stamp = stamp;
  tracking_elements.header.frame_id = "world";
  environmental_model_.collectMsgs(tracking_elements);
  tracking_element_publisher.publish(tracking_elements);
}

}  // namespace environmental_model

CREATE_NODE(environmental_model::EnvironmentalModelNode)
