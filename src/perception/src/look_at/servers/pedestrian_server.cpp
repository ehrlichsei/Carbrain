#include "pedestrian_server.h"
#include "../../road_object_detection/classifiers/pedestrian_classifier.h"
#include "message_converter.h"
THIRD_PARTY_HEADERS_BEGIN

THIRD_PARTY_HEADERS_END

namespace look_at {

const std::string PedestrianServer::NAMESPACE("pedestrian_server");
const ParameterString<int> PedestrianServer::ROI_OFFSET_U(NAMESPACE +
                                                          "/roi_offset_u");
const ParameterString<int> PedestrianServer::ROI_OFFSET_V(NAMESPACE +
                                                          "/roi_offset_v");

PedestrianServer::PedestrianServer(const ros::NodeHandle &node_handle,
                                   common::node_base::ParameterInterface *parameters,
                                   const std::string &action_name)
    : ServerInterface(node_handle, parameters),
      action_name_(action_name),
      server_(node_handle, action_name, false),
      parameters_ptr_(parameters) {
  classifier_ = std::make_unique<road_object_detection::PedestrianClassifier>(
      camera_transformation_.get(), parameters);
  look_at_ = std::make_unique<LookAt>(
      parameters, camera_transformation_.get(), &tf_helper_, ego_vehicle_.get());
  server_.registerGoalCallback(boost::bind(&PedestrianServer::goalCB, this));
  server_.registerPreemptCallback(boost::bind(&PedestrianServer::preemptCB, this));
  server_.start();
  sync_.registerCallback(boost::bind(
      &PedestrianServer::regionsOfInterestCallback, this, _1, _2, _3, _4, _5));

  parameters->registerParam(ROI_OFFSET_U);
  parameters->registerParam(ROI_OFFSET_V);
  look_at_->setROIOffset(cv::Size{parameters->getParam(ROI_OFFSET_U),
                                  parameters->getParam(ROI_OFFSET_V)});
}

void PedestrianServer::regionsOfInterestCallback(const sensor_msgs::ImageConstPtr &image_msg,
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
  MessageConverter<perception_msgs::Pedestrian, road_object_detection::Pedestrian, std::vector<geometry_msgs::Point>, WorldPoints> message_converter;

  // create message which is the result
  actionlib::SimpleActionServer<perception_msgs::LookForPedestriansAction>::Result pedestrians;
  pedestrians.pedestrians.sub_messages.reserve(classifications_.size());
  for (auto &result : classifications_) {
    world_coordinates_helper_->calcWorldCoordinates(result.detected_objects);

    for (const auto &pedestrian : result.detected_objects) {
      const auto value =
          dynamic_cast<road_object_detection::Pedestrian *>(pedestrian.get());
      pedestrians.pedestrians.sub_messages.push_back(
          message_converter.createMsg(*value, result.id));
      (pedestrians.pedestrians.sub_messages.back()).pose = tf2::toMsg(value->pose_in_world);
    }
    pedestrians.pedestrians.header.stamp = image_msg->header.stamp;
  }

  server_.setSucceeded(pedestrians);
}

void PedestrianServer::goalCB() {
  regions_to_classify_ = extractROIs(server_.acceptNewGoal()->pedestrian_regions);
}

}  // namespace look_at
