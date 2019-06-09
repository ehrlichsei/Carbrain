#include "obstacles_server.h"
#include "../../road_object_detection/classifiers/obstacle_classifier_new.h"
#include "message_converter.h"
#include "../../road_object_detection/road_objects/road_object.h"

namespace look_at {

inline int8_t toMsg(const road_object_detection::Obstacle::DetectionState detection_state) {
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

ObstaclesServer::ObstaclesServer(const ros::NodeHandle &node_handle,
                                 ParameterInterface *parameters,
                                 const std::string &action_name)
    : ServerInterface(node_handle, parameters),
      action_name_(action_name),
      server_(node_handle, action_name, false),
      parameters_ptr_(parameters) {
  classifier_ = std::make_unique<road_object_detection::ObstacleClassifierNew>(
      camera_transformation_.get(), parameters);
  look_at_ = std::make_unique<LookAt>(
      parameters, camera_transformation_.get(), &tf_helper_, ego_vehicle_.get());
  server_.registerGoalCallback(boost::bind(&ObstaclesServer::goalCB, this));
  server_.registerPreemptCallback(boost::bind(&ObstaclesServer::preemptCB, this));
  server_.start();
  sync_.registerCallback(boost::bind(
      &ObstaclesServer::regionsOfInterestCallback, this, _1, _2, _3, _4, _5));
}

void ObstaclesServer::regionsOfInterestCallback(const sensor_msgs::ImageConstPtr &image_msg,
                                                const nav_msgs::PathConstPtr &right_points,
                                                const nav_msgs::PathConstPtr &middle_points,
                                                const nav_msgs::PathConstPtr &left_points,
                                                const nav_msgs::PathConstPtr &no_passing_points) {
  if (!server_.isActive()) {
    ROS_INFO_THROTTLE(2, "%s-action is not active in callback.", action_name_.c_str());
    return;
  }
  // call interface and process data
  process(image_msg, right_points, middle_points, left_points, no_passing_points);

  // for message conversion
  MessageConverter<perception_msgs::Obstacle, road_object_detection::Obstacle, std::vector<geometry_msgs::Point>, WorldPoints> message_converter;

  // create message which is the result
  actionlib::SimpleActionServer<perception_msgs::LookForObstaclesAction>::Result obstacles;
  obstacles.obstacles.sub_messages.reserve(classifications_.size());
  for (auto &result : classifications_) {
    world_coordinates_helper_->calcWorldCoordinates(result.detected_objects);

    for (const auto &obstacle : result.detected_objects) {
      const auto value =
          dynamic_cast<road_object_detection::Obstacle *>(obstacle.get());
      obstacles.obstacles.sub_messages.push_back(
          message_converter.createMsg(*value, result.id));
      obstacles.obstacles.sub_messages.back().vertices =
          message_converter.toMsg(value->base_hull_polygon_in_world);
      obstacles.obstacles.sub_messages.back().vertices_detected =
          toMsg(value->vertices_detection_state);
    }
    obstacles.obstacles.header.stamp = image_msg->header.stamp;
  }

  server_.setSucceeded(obstacles);
}

void ObstaclesServer::goalCB() {
  regions_to_classify_ = extractROIs(server_.acceptNewGoal()->obstacle_regions);
}

}  // namespace look_at
