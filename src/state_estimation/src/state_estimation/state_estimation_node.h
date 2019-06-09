#ifndef STATE_ESTIMATION_NODE_H
#define STATE_ESTIMATION_NODE_H
#include <common/macros.h>

THIRD_PARTY_HEADERS_BEGIN
#include <common/node_base.h>
#include <ros/publisher.h>
#include <state_estimation_msgs/ProcessSensorData.h>
#include <memory>
THIRD_PARTY_HEADERS_END

#include "state_estimation.h"

/*!
 * \brief Filters the incoming sensor data with a kalman filter to estimate the
 * current state of the car.
 */
class StateEstimationNode : public NodeBase {
 public:
  /*!
   * \brief StateEstimationNode the constructor
   * \param node_handle the NodeHandle to be used.
   */
  StateEstimationNode(ros::NodeHandle& node_handle);
  /*!
   * \brief StateEstimationNode the constructor
   * \param node_handle the NodeHandle to be used.
   * \param state_estimation_ the state_estimation to be used
   */
  StateEstimationNode(ros::NodeHandle& node_handle,
                      std::unique_ptr<StateEstimation> state_estimation_);

  virtual ~StateEstimationNode() override;

  /*!
   * \brief returns the name of the node. It is needed in main and onInit
   * (nodelet) method.
   * \return the name of the node
   */
  static const std::string getName();

  /**
   * @brief stateEstimationPublisherLoop runs in a dedicated thread and
   * publishes new state estimation
   * information to a ros topic as it becomes available.
   */
  void stateEstimationPublisherLoop();

  /**
   * @brief processSensorData is the service callback that performs the
   * processing
   * of sensor data for the integration tests. It runs the state estimation on
   * all incoming sensor
   * data samples and returns all intermediate estimation results as response.
   */
  bool processSensorData(state_estimation_msgs::ProcessSensorDataRequest& request,
                         state_estimation_msgs::ProcessSensorDataResponse& response);


 protected:
  // NodeBase interface
  /*!
   * \brief startModule is called, if the node shall be turned active. In here
   * the subscribers an publishers are started.
   */
  virtual void startModule() override;
  /*!
   * \brief stopModule is called, if the node shall be turned inactive. In this
   * function subscribers and publishers are shutdown.
   */
  virtual void stopModule() override;

  /**
   * @brief isInTestMode returns true if the node parameter
   * "integration_test_mode" exists and is set to true.
   *
   * @return true if the integration test mode was requested, false if not.
   */
  bool isInTestMode() const;

  /*!
   * \brief state_estimation contains the ROS-indipendent implementation of this
   * node.
   */
  std::unique_ptr<StateEstimation> state_estimation_;

  /*!
   * \brief ros service server to control the state estimation integration tests
   */
  ros::ServiceServer test_service_server;

  /*!
   * \brief state_estimation_publisher_thread_ is the boost thread that runs the
   * stateEstimationPublisherLoop in which new state estimation
   * information is published to a ros topic as it becomes available.
   */
  boost::thread state_estimation_publisher_thread_;

  /*!
   * \brief state_estimation_publisher_ is the ros publisher that publishes new
   * state estimation
   * information to a ros topic as it becomes available.
   */
  ros::Publisher state_estimation_publisher_;

 private:
  /*!
   * \brief init init implements initialization of the node. It's only used in
   * the constructors.
   */
  void init();
};

SensorMeasurements fromMsg(const controller_msgs::StateMeasure& msg);

#endif  // STATE_ESTIMATION_NODE_H
