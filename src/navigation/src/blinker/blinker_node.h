#ifndef BLINKER_NODE_H
#define BLINKER_NODE_H
#include <common/macros.h>

#include "common/node_base.h"

#include "blinker.h"
#include "blinker_decision.h"
#include "navigation/driving_corridor.h"

THIRD_PARTY_HEADERS_BEGIN
#include <nav_msgs/Path.h>
#include <navigation_msgs/DrivingCorridor.h>
#include <navigation_msgs/PavlovBlinkerCommand.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <Eigen/Core>
//#include <navigation_msgs/PavlovBlinkerCommand/Request.h>
//#include <navigation_msgs/PavlovBlinkerCommand/Response.h>
#include <controller_msgs/LightsCommand.h>
THIRD_PARTY_HEADERS_END

/*!
 * \brief Takes care of activating the blinker before switching lanes
 */
class BlinkerNode : public NodeBase {
 public:
  /*!
   * \brief BlinkerNode the constructor.
   * \param node_handle the NodeHandle to be used.
   */
  BlinkerNode(ros::NodeHandle& node_handle);
  /*!
   * \brief returns the name of the node. It is needed in main and onInit
   * (nodelet) method.
   * \return the name of the node
   */
  static const std::string getName();

  bool setPavlovBlinkerCommand(navigation_msgs::PavlovBlinkerCommand::Request& req,
                               navigation_msgs::PavlovBlinkerCommand::Response&);

 private:
  // NodeBase interface
  /*!
   * \brief startModule is called, if the node shall be turned active. In here
   * the subrscribers an publishers are started.
   */
  void startModule() override;
  /*!
   * \brief stopModule is called, if the node shall be turned inactive. In this
   * function subscribers and publishers are shutdown.
   */
  void stopModule() override;

  void blinkerCommand(const BlinkerDecision blinker_decision);
  void changeBlinker(const BlinkerDecision blinker_decision);

  void safeCorridorCallback(const navigation_msgs::DrivingCorridor::ConstPtr& safe_corridor_msg);

  void timerCallback(const ros::TimerEvent&);

  void resetTimer(const double duration);

  bool vehicleToWorld(const Eigen::Affine3d& vehicle_pose, Eigen::Affine3d& world_pose);

  const double timer_duration = 1.75;

  controller_msgs::BlinkerCommand pavlov_blinker_command;

  ros::ServiceServer set_pavlov_blinker_command_server;

  tf2_ros::TransformListener tf_listener;
  tf2_ros::Buffer tf_buffer;

  /*!
   * \brief blinker contains the ROS-indipendent implementation of this node.
   */
  Blinker blinker;
  ros::Subscriber safe_corridor_subscriber;
  ros::Timer timer;
  /*!
   * publisher for blinking commands
   */
  ros::Publisher blinker_publisher;
  BlinkerDecision old_blinker_decision = BlinkerDecision::NO_DECISION;
  bool pavlov_blinker_command_enabled;
  int counter_same_decision_ = 0;
};

#endif  // BLINKER_NODE_H
