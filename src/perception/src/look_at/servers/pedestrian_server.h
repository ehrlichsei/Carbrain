#ifndef PEDESTRIAN_SERVER_H
#define PEDESTRIAN_SERVER_H

#include <common/macros.h>
#include "server_interface.h"

THIRD_PARTY_HEADERS_BEGIN
#include <perception_msgs/LookForPedestriansAction.h>

#include <actionlib/server/simple_action_server.h>
THIRD_PARTY_HEADERS_END

namespace look_at {
class PedestrianServer : public ServerInterface {
 public:
  PedestrianServer(const ros::NodeHandle &node_handle,
                   ParameterInterface *parameters,
                   const std::string &action_name);

  PedestrianServer() = delete;

  virtual ~PedestrianServer() override = default;

  virtual void regionsOfInterestCallback(const sensor_msgs::ImageConstPtr &image_msg,
                                         const nav_msgs::PathConstPtr &right_points,
                                         const nav_msgs::PathConstPtr &middle_points,
                                         const nav_msgs::PathConstPtr &left_points,
                                         const nav_msgs::PathConstPtr &no_passing_points);

  virtual void goalCB() override;

  virtual void preemptCB() override { this->server_.setPreempted(); }

  virtual bool isDebug() const override { return false; }

  static const std::string NAMESPACE;

  static const ParameterString<int> ROI_OFFSET_U;
  static const ParameterString<int> ROI_OFFSET_V;

 protected:
  std::string action_name_;

  actionlib::SimpleActionServer<perception_msgs::LookForPedestriansAction> server_;
  ParameterInterface *parameters_ptr_;
};

}  // namespace look_at

#endif  // PEDESTRIAN_SERVER_H
