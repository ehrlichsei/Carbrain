#ifndef STEERING_CONTROLLER_NODE_DEBUG_H
#define STEERING_CONTROLLER_NODE_DEBUG_H

#include "steering_controller_node.h"
#include <common/macros.h>

THIRD_PARTY_HEADERS_BEGIN
#include "lateral_controller_msgs/DrivingSteeringAngle.h"
THIRD_PARTY_HEADERS_END

#include "common/node_base.h"
#include "steering_controller.h"

/*!
 * \brief The SteeringController is a pid controller for the steering angle of
 * the car.
 */
class SteeringControllerNodeDebug : public SteeringControllerNode {
 public:
  /*!
   * \brief SteeringControllerNode the constructor.
   * \param node_handle the NodeHandle to be used.
   */
  SteeringControllerNodeDebug(ros::NodeHandle& node_handle);

 private:
  // NodeBase interface
  /*!
   * \brief startModule is called, if the node shall be turned active. In here
   * the subscribers and publishers are started.
   */
  virtual void startModule() override;
  /*!
   * \brief stopModule is called, if the node shall be turned inactive. In this
   * function subscribers and publishers are shutdown.
   */
  virtual void stopModule() override;

  /*!
   * \brief handleStateMeasure callback of the state measure topic
   * \param msg the new state message.
   */
  void handleState(const state_estimation_msgs::State& msg);

  /*!
   * \brief time_of_last_measurement to calculate delta t
   */
  ros::Time time_of_last_measurement;

  /*!
   * \brief state_estimation_subscriber_ the ros subscriber for the
   * state estimation topic
   */
  ros::Subscriber state_estimation_subscriber_;
};

#endif  // STEERING_CONTROLLER_NODE_DEBUG_H
