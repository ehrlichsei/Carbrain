#ifndef PARKINGLOT_ALIGNER_NODE_H
#define PARKINGLOT_ALIGNER_NODE_H

#include "common/node_base.h"
#include "navigation/driving_corridor.h"

#include "parkinglot_aligner.h"

THIRD_PARTY_HEADERS_BEGIN
#include <tf2_ros/transform_broadcaster.h>
THIRD_PARTY_HEADERS_END

/*!
 * \brief ParkinglotAligner
 */
class ParkinglotAlignerNode : public NodeBase {
 public:
  /*!
   * \brief ParkinglotAlignerNode the constructor.
   * \param node_handle the NodeHandle to be used.
   */
  ParkinglotAlignerNode(ros::NodeHandle &node_handle);
  /*!
   * \brief returns the name of the node. It is needed in main and onInit
   * (nodelet) method.
   * \return the name of the node
   */
  static const std::string getName();

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

  /*!
   * \brief parkinglot_aligner contains the ROS-indipendent implementation of
   * this node.
   */
  ParkinglotAligner parkinglot_aligner_;

  void fullCorridorCallback(const navigation_msgs::DrivingCorridor::ConstPtr& full_corridor_msg);
  void parkinglotPointCallback(const geometry_msgs::Point::ConstPtr &parkinglot_point_msg);

  ros::Subscriber full_corridor_subsriber_;
  ros::Subscriber parkinglot_point_subsriber_;

  tf2_ros::TransformListener transform_listener;
  tf2_ros::Buffer transform_buffer;

  bool getCarPosition(Eigen::Vector3d &carPosition);


  void publishParkinglotToWorldTransform(const Eigen::Affine3d &parkinglot_to_world_transform,
                                         const ros::Time &stamp);


  tf2_ros::TransformBroadcaster tf2_broadcaster_;
};

#endif  // PARKINGLOT_ALIGNER_NODE_H
