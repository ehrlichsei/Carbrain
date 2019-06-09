#include "message_handler.h"
#include <common/macros.h>
#include <perception_message_conversions.h>

THIRD_PARTY_HEADERS_BEGIN
#include <perception_msgs/RoadObjects.h>
#include <tf2_eigen/tf2_eigen.h>
#include <boost/range/algorithm/transform.hpp>
THIRD_PARTY_HEADERS_END

#include "common/lift.h"

namespace road_object_detection {

inline int8_t toMsg(const Obstacle::DetectionState detection_state) {
  return static_cast<int8_t>(detection_state);
}

template <class T>
using deriveMsgType = decltype(toMsg(std::declval<T>()));

template <typename T>
auto toMsg(const T &vector) {
  std::vector<deriveMsgType<typename T::value_type>> message_vector;
  message_vector.reserve(vector.size());
  boost::transform(vector, std::back_inserter(message_vector), LIFT(toMsg));
  return message_vector;
}

template <class Message, class RoadObject>
Message createMsg(const RoadObject &road_object) {
  Message message;
  message.header.frame_id = "world";
  message.header.stamp = road_object.timestamp;
  message.certainty = static_cast<float>(road_object.score);
  message.id = road_object.id;
  message.base_hull_polygon = toMsg(road_object.base_hull_polygon_in_world);
  return message;
}

MessageHandler::MessageHandler(ros::NodeHandle &node_handle)
    : node_handle_(node_handle) {}

void MessageHandler::shutdown() {
  helper_unidentified_msg.shutdown();
  helper_obstacle_msg.shutdown();
  helper_junction_msg.shutdown();
  helper_crosswalk_msg.shutdown();
  helper_road_closure_msg.shutdown();
  helper_speed_limit_marking_msg.shutdown();
  helper_arrow_marking_msg.shutdown();
  helper_start_stop_line_msg.shutdown();
  helper_pedestrian_msg.shutdown();
  helper_no_passing_zone_msg.shutdown();
}

void MessageHandler::advertise() {
  helper_unidentified_msg.advertise(node_handle_, "unidentified");
  helper_obstacle_msg.advertise(node_handle_, "obstacle");
  helper_junction_msg.advertise(node_handle_, "junction");
  helper_crosswalk_msg.advertise(node_handle_, "crosswalk");
  helper_road_closure_msg.advertise(node_handle_, "road_closure");
  helper_speed_limit_marking_msg.advertise(node_handle_, "speed_limit_marking");
  helper_arrow_marking_msg.advertise(node_handle_, "arrow_marking");
  helper_start_stop_line_msg.advertise(node_handle_, "startline");
  helper_pedestrian_msg.advertise(node_handle_, "pedestrian");
  helper_no_passing_zone_msg.advertise(node_handle_, "no_passing_zone");

  road_objects_publisher =
      node_handle_.advertise<perception_msgs::RoadObjects>("road_objects", 1);
}

void MessageHandler::publishRoadObjects(RoadObjects &road_objects, const ros::Time &stamp) {
  RoadObjectVisitor::visit(road_objects);
  publish(stamp);
}

void MessageHandler::publish(const ros::Time &time_stamp) {
  perception_msgs::RoadObjects road_objects;
  road_objects.unidentifieds = helper_unidentified_msg.generateAndClearMessage(time_stamp);
  road_objects.obstacles = helper_obstacle_msg.generateAndClearMessage(time_stamp);
  road_objects.junctions = helper_junction_msg.generateAndClearMessage(time_stamp);
  road_objects.crosswalks = helper_crosswalk_msg.generateAndClearMessage(time_stamp);
  road_objects.road_closures = helper_road_closure_msg.generateAndClearMessage(time_stamp);
  road_objects.speed_limit_markings =
      helper_speed_limit_marking_msg.generateAndClearMessage(time_stamp);
  road_objects.arrow_markings = helper_arrow_marking_msg.generateAndClearMessage(time_stamp);
  road_objects.start_lines = helper_start_stop_line_msg.generateAndClearMessage(time_stamp);
  road_objects.no_passing_zones =
      helper_no_passing_zone_msg.generateAndClearMessage(time_stamp);

  helper_unidentified_msg.publishMessage(road_objects.unidentifieds);
  helper_obstacle_msg.publishMessage(road_objects.obstacles);
  helper_junction_msg.publishMessage(road_objects.junctions);
  helper_crosswalk_msg.publishMessage(road_objects.crosswalks);
  helper_road_closure_msg.publishMessage(road_objects.road_closures);
  helper_speed_limit_marking_msg.publishMessage(road_objects.speed_limit_markings);
  helper_arrow_marking_msg.publishMessage(road_objects.arrow_markings);
  helper_start_stop_line_msg.publishMessage(road_objects.start_lines);
  helper_no_passing_zone_msg.publishMessage(road_objects.no_passing_zones);

  helper_pedestrian_msg.publishMessage(
      helper_pedestrian_msg.generateAndClearMessage(time_stamp));


  road_objects_publisher.publish(road_objects);
}

void MessageHandler::visit(Unidentified &road_object) {
  auto message = createMsg<perception_msgs::Unidentified>(road_object);

  message.hull_polygon = toMsg(road_object.base_hull_polygon_in_world);

  helper_unidentified_msg.addSubMessage(message);
}

void MessageHandler::visit(Obstacle &road_object) {
  auto message = createMsg<perception_msgs::Obstacle>(road_object);

  message.vertices = toMsg(road_object.base_hull_polygon_in_world);
  message.vertices_detected = toMsg(road_object.vertices_detection_state);

  helper_obstacle_msg.addSubMessage(message);
}

void MessageHandler::visit(Junction &road_object) {
  auto message = createMsg<perception_msgs::Junction>(road_object);

  message.pose = toMsg(road_object.pose_in_world);
  message.junction_type = road_object.junction_type;

  helper_junction_msg.addSubMessage(message);
}

void MessageHandler::visit(Crosswalk &road_object) {
  auto message = createMsg<perception_msgs::Crosswalk>(road_object);

  message.pose = toMsg(road_object.pose_in_world);

  helper_crosswalk_msg.addSubMessage(message);
}

void MessageHandler::visit(RoadClosure &road_object) {
  auto message = createMsg<perception_msgs::RoadClosure>(road_object);

  message.hull_polygon = toMsg(road_object.base_hull_polygon_in_world);

  helper_road_closure_msg.addSubMessage(message);
}

void MessageHandler::visit(SpeedLimitMarking &road_object) {
  auto message = createMsg<perception_msgs::SpeedLimitMarking>(road_object);

  message.pose = toMsg(road_object.pose_in_world);
  message.speed_limit = road_object.speed_limit;
  message.limit_relieved =
      static_cast<perception_msgs::SpeedLimitMarking::_limit_relieved_type>(
          road_object.limit_relieved);

  helper_speed_limit_marking_msg.addSubMessage(message);
}

void MessageHandler::visit(StartLine &road_object) {
  auto message = createMsg<perception_msgs::StartLine>(road_object);

  message.pose = toMsg(road_object.pose_in_world);

  helper_start_stop_line_msg.addSubMessage(message);
}

void MessageHandler::visit(ArrowMarking &road_object) {
  auto message = createMsg<perception_msgs::ArrowMarking>(road_object);

  message.pose = toMsg(road_object.pose_in_world);
  message.direction = static_cast<int>(road_object.arrow_type);

  helper_arrow_marking_msg.addSubMessage(message);
}

void MessageHandler::visit(Pedestrian &road_object) {
  auto message = createMsg<perception_msgs::Pedestrian>(road_object);

  message.pose = toMsg(road_object.pose_in_world);

  helper_pedestrian_msg.addSubMessage(message);
}

void MessageHandler::visit(NoPassingZone &road_object) {
  auto message = createMsg<perception_msgs::NoPassingZone>(road_object);

  message.pose = toMsg(road_object.pose_in_world);
  message.start = toMsg(road_object.start_point_world);
  message.end = toMsg(road_object.end_point_world);

  helper_no_passing_zone_msg.addSubMessage(message);
}

}  // namespace road_object_detection
