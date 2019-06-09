#ifndef LOCALIZATION_NODE_H
#define LOCALIZATION_NODE_H
#include <common/macros.h>

THIRD_PARTY_HEADERS_BEGIN
#include <std_msgs/Empty.h>
#include <tf2_ros/transform_broadcaster.h>
THIRD_PARTY_HEADERS_END

#include "common/node_base.h"

#include "localization.h"

/*!
 * \brief Self-localization via integration of state_estimation data. This class contains
 * the ROS-specific code.
 */
class LocalizationNode : public NodeBase {
 public:
  /*!
   * \brief LocalizationNode the constructor.
   * \param node_handle the NodeHandle to be used.
   */
  LocalizationNode(ros::NodeHandle& node_handle);
  /*!
   * \brief returns the name of the node. It is needed in main and onInit (nodelet) method.
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

  // Callbacks
  /*!
   * \brief stateEstimationCallback The callback function which is called if a state
   * message has been received. It updates the 'vehicle'->'world' transformation and
   * publishs the new transformation and a pose (for synchronisation purposes).
   * \param state_msg the reveived state message.
   */
  void stateEstimationCallback(const state_estimation_msgs::State::ConstPtr& state_msg);

  /*!
   * \brief resetLocationCallback The callback function which is called if a reset signal
   *  has been received. It resets the 'vehicle'->'world' transformation.
   * \param reset_signal the reset signal
   */
  void resetLocationCallback(const std_msgs::Empty::ConstPtr& reset_signal);

  /*!
   * \brief localization_ contains the ROS-indipendent implementation of this node.
   */
  Localization localization_;

  ros::Subscriber state_estimation_subscriber_;
  ros::Subscriber reset_location_subscriber_;
  ros::Publisher pose_publisher_;
  tf2_ros::TransformBroadcaster transform_broadcaster_;
};

#endif  // LOCALIZATION_NODE_H
