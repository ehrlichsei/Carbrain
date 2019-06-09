#ifndef FITNESS_MONITOR_NODE_H
#define FITNESS_MONITOR_NODE_H
#include <common/macros.h>

THIRD_PARTY_HEADERS_BEGIN
#include "state_estimation_msgs/State.h"
#include <lateral_controller_msgs/DrivingError.h>
#include "perception_msgs/StartLines.h"
#include "geometry_msgs/PoseStamped.h"
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
THIRD_PARTY_HEADERS_END

#include "common/node_base.h"

#include "fitness_monitor.h"
#include "fitness_calculator.h"
#include "round_detection.h"

/*!
 * \brief monitores min/max/avg. velocity etc.
 */
class FitnessMonitorNode : public NodeBase {
 public:
  /*!
   * \brief FitnessMonitorNode the constructor.
   * \param node_handle the NodeHandle to be used.
   */
  FitnessMonitorNode(ros::NodeHandle &node_handle);
  /*!
   * \brief returns the name of the node. It is needed in main and onInit
   * (nodelet) method.
   * \return the name of the node
   */
  static const std::string getName();


  void startlineCallback(const perception_msgs::StartLines::ConstPtr &lines_msg);

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

  Eigen::Affine3d getVehicleToWorldTransformation(const ros::Time& transform_time) const;

  tf2_ros::Buffer tf2_buffer_;
  tf2_ros::TransformListener tf2_listener_;

  /*!
   * \brief fitness_monitor contains the ROS-indipendent implementation of this
   * node.
   */
  FitnessMonitor fitness_monitor_;
  FitnessCalculator fitness_calculator_;
  RoundDetection round_detection_;

  ros::Subscriber state_estimation_subscriber_;
  ros::Subscriber start_line_subscriber_;
  ros::Subscriber lateral_error_subscriber_;
  void stateEstimationCallback(const state_estimation_msgs::State::ConstPtr &state_msg);
  void lateralErrorCallback(const lateral_controller_msgs::DrivingError error);
};

#endif  // FITNESS_MONITOR_NODE_H
