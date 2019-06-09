#ifndef OBSTACLES_SERVER_H
#define OBSTACLES_SERVER_H

#include "common/parameter_interface.h"
#include "server_interface.h"

THIRD_PARTY_HEADERS_BEGIN
#include <perception_msgs/LookForObstaclesAction.h>

#include <actionlib/server/simple_action_server.h>
THIRD_PARTY_HEADERS_END

namespace look_at {
class ObstaclesServer : public ServerInterface {
 public:
  ObstaclesServer(const ros::NodeHandle &node_handle,
                  ParameterInterface *parameters,
                  const std::string &action_name);
  ObstaclesServer() = delete;

  virtual ~ObstaclesServer() override = default;

  virtual void regionsOfInterestCallback(const sensor_msgs::ImageConstPtr &image_msg,
                                         const nav_msgs::PathConstPtr &right_points,
                                         const nav_msgs::PathConstPtr &middle_points,
                                         const nav_msgs::PathConstPtr &left_points,
                                         const nav_msgs::PathConstPtr &no_passing_points);

  virtual void goalCB() override;

  virtual void preemptCB() override { this->server_.setPreempted(); }

  virtual bool isDebug() const override { return false; }

 protected:
  std::string action_name_;

  actionlib::SimpleActionServer<perception_msgs::LookForObstaclesAction> server_;
  ParameterInterface *parameters_ptr_;
};

}  // namespace look_at

#endif  // OBSTACLES_SERVER_H
