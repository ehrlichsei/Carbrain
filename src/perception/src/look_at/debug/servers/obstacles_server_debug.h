#ifndef OBSTACLES_SERVER_DEBUG_H
#define OBSTACLES_SERVER_DEBUG_H

#include "../../servers/obstacles_server.h"
#include "debug_images_publisher.h"

namespace look_at {
class ObstaclesServerDebug : public DebugImagesPublisher, public ObstaclesServer {
 public:
  ObstaclesServerDebug(const ros::NodeHandle &node_handle,
                       ParameterInterface *parameters,
                       const std::string &action_name);

  virtual void regionsOfInterestCallback(const sensor_msgs::ImageConstPtr &image_msg,
                                         const nav_msgs::PathConstPtr &right_points,
                                         const nav_msgs::PathConstPtr &middle_points,
                                         const nav_msgs::PathConstPtr &left_points,
                                         const nav_msgs::PathConstPtr &no_passing_points) override;

  virtual void advertise() override;

  virtual void shutdown() override;

  virtual bool isDebug() const override { return true; }

 private:
  ros::Publisher rospub_field_of_vision_;
};
}  // namespace look_at

#endif  // OBSTACLES_SERVER_DEBUG_H
