#ifndef SERVER_INTERFACE_H
#define SERVER_INTERFACE_H

#include "../look_at.h"
#include "common/node_base.h"

#include "../../road_object_detection/road_object_visitors/message_handler.h"

THIRD_PARTY_HEADERS_BEGIN
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>

#include <nav_msgs/Path.h>
#include <perception_msgs/LookAtRegions.h>
#include <sensor_msgs/Image.h>
THIRD_PARTY_HEADERS_END

namespace look_at {

class ServerInterface {
 public:
  ServerInterface(const ros::NodeHandle &node_handle, ParameterInterface *parameters);

  virtual ~ServerInterface() = default;

  virtual void advertise();

  virtual void shutdown();

  virtual void goalCB() = 0;

  virtual void preemptCB() = 0;

  virtual bool isDebug() const = 0;

 protected:
  ros::NodeHandle nh_;
  std::unique_ptr<common::CameraTransformation> camera_transformation_;
  std::unique_ptr<EgoVehicle> ego_vehicle_;
  std::unique_ptr<road_object_detection::WorldCoordinatesHelper> world_coordinates_helper_;
  tf_helper::TFHelper<double> tf_helper_;

  RegionsToClassify regions_to_classify_;

  // subscribers
  message_filters::Subscriber<sensor_msgs::Image> image_raw_sub_;
  //  message_filters::Subscriber<perception_msgs::LookAtRegions> roi_sub_;
  message_filters::Subscriber<nav_msgs::Path> right_points_sub_;
  message_filters::Subscriber<nav_msgs::Path> middle_points_sub_;
  message_filters::Subscriber<nav_msgs::Path> left_points_sub_;
  message_filters::Subscriber<nav_msgs::Path> no_passing_points_sub_;
  message_filters::TimeSynchronizer<sensor_msgs::Image, nav_msgs::Path, nav_msgs::Path, nav_msgs::Path, nav_msgs::Path> sync_;

  std::unique_ptr<road_object_detection::Classifier> classifier_;

  std::unique_ptr<LookAt> look_at_;

  ClassificationResults classifications_;

  void process(const sensor_msgs::ImageConstPtr &image_msg,
               const nav_msgs::PathConstPtr &right_points,
               const nav_msgs::PathConstPtr &middle_points,
               const nav_msgs::PathConstPtr &left_points,
               const nav_msgs::PathConstPtr &no_passing_points);

  RegionsToClassify extractROIs(const perception_msgs::LookAtRegions &regions) const;

 private:
  static const tf_helper::FrameIDs FRAME_IDS;

  tf2_ros::Buffer tf2_buffer_;
  tf2_ros::TransformListener tf2_listener_;
};

}  // namespace look_at

#endif  // SERVER_INTERFACE_H
