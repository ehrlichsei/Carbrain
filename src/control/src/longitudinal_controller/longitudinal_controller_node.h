#ifndef LONGITUDINAL_CONTROLLER_NODE_H
#define LONGITUDINAL_CONTROLLER_NODE_H

#include <common/macros.h>

THIRD_PARTY_HEADERS_BEGIN
#include <Eigen/Dense>
#include <boost/optional.hpp>

#include <nav_msgs/Path.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Float32.h>
#include <std_srvs/SetBool.h>
#include <tf2_ros/transform_listener.h>

#include "navigation_msgs/AutomaticCruiseControlUpdate.h"
#include "state_estimation_msgs/State.h"

#include "longitudinal_controller_msgs/DrivingSpeed.h"
#include "longitudinal_controller_msgs/StopAt.h"
THIRD_PARTY_HEADERS_END

#include "common/node_base.h"
#include "longitudinal_control.h"

/*!
 * \brief The high level controller generates a speed and steering command based
 * on localization data and the given target path.
 */
class LongitudinalControllerNode : public NodeBase {

 public:
  /*!
   * \brief HighLevelControllerNode the constructor.
   * \param node_handle the NodeHandle to be used.
   */
  LongitudinalControllerNode(ros::NodeHandle& node_handle);

  /*!
   * \brief returns the name of the node. It is needed in main and onInit
   * (nodelet) method.
   * \return the name of the node
   */
  static const std::string getName();

 protected:
  friend class LongitudinalControlDebug;
  friend class CurvatureControllerDebug;

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

  static const ParameterString<int> NUMBER_OF_PATH_REGRESSION_POINTS;
  static const ParameterString<int> MAX_PATH_INPUT_DELAY;
  static const ParameterString<double> PATH_TIMEOUT;

  // NodeBase interface
  /*!
   * \brief Callbackfunktion für path update
   *
   * Wird ein neuer Pfad empfangen wird dieses in lokalen Variablen der Quer-
   * und Längsregelungsklassen gespeichert.
   *
   * \param safe_target_path Pfad
   */
  void safeTargetPathCallback(const nav_msgs::Path::ConstPtr& safe_target_path);

  /*!
   * \brief Callbackfunktion für Positionsupdate
   *
   * Wird ein neuer Fahrzeugzustand empfangen ruft diese Funktion den Quer- und
   * Längsregler auf und published anschließend eine Message für das
   *ControllerInterface.
   *
   * \param state_msg  Zustand des Fahrzeugs
   */
  void stateEstimationCallback(const state_estimation_msgs::State::ConstPtr& state_msg);

  void autoResetCallback(const std_msgs::Empty& auto_reset);

  /*!
   * \brief desiredSpeedCallback callback to set the desired speed of the
   * longitudinal controller
   * \param desired_speed the new desired speed.
   */
  void desiredSpeedCallback(const std_msgs::Float32& desired_speed);

  /*!
   * \brief accUpdateCallback callback for the ACC update of speed and distance
   * to obstacle
   * \param acc_update the new speed and distance to the obstacle.
   */
  void accUpdateCallback(const navigation_msgs::AutomaticCruiseControlUpdate::ConstPtr& acc_update);

  /*!
   * \brief accDeActivationCallback callback for activation/deactivation service
   * \param request whether to activate or deactivate acc.
   * \param response whether activation/deactivation has succeeded.
   * \return whether the call succeeded.
   */
  bool accDeActivationCallback(std_srvs::SetBool::Request& request,
                               std_srvs::SetBool::Response& response);

  /*!
   * \brief stopAtCallback stop at service callback
   * \param request the request.
   * \param response the respone.
   * \return whether the call succeeded.
   */
  bool stopAtCallback(longitudinal_controller_msgs::StopAt::Request& request,
                      longitudinal_controller_msgs::StopAt::Response& response);

  /*!
   * \brief pollParams poll the parameters
   */
  void pollParams(const ros::TimerEvent&);


  /*!
   * \brief getVehiclePose
   *
   * returns the current vehicle pose (of the front-axle) at the time time_stamp
   * relative to frame_id
   *
   * \param frame_id the frame to get the relative pose to.
   * \return the pose if obtainable or nothing.
   */
  boost::optional<Eigen::Affine2d> getVehiclePose(const std::string& frame_id);


  /*!
   * \brief longitudinal_control_ contains the ROS-indipendent implementation of
   *        the longitudinal controller.
   */
  std::unique_ptr<LongitudinalControl> longitudinal_control_;

  /*!
   * \brief safe_target_path_ the path
   */
  boost::optional<common::Path<>::RawPath> target_path_;
  boost::optional<std_msgs::Header> target_path_header_;


  tf2_ros::Buffer tf2_buffer_;
  tf2_ros::TransformListener tf2_listener_;

  ros::ServiceServer acc_activation_;
  ros::ServiceServer stopping_server_;

  ros::Subscriber state_estimation_subscriber_;  // kalmanfilter
  ros::Subscriber safe_target_points_subscriber_;
  ros::Subscriber auto_reset_subscriber_;
  ros::Subscriber velocity_subscriber_;
  ros::Subscriber acc_update_subscriber_;

  /*!
   * \brief velocity_command_publisher_ publisher of the control outputs
   */
  ros::Publisher velocity_command_publisher_;
  /*!
   * \brief stopping_completed_publisher_ publisher to inform the client who
   * asked the controller to stop the vehicle that the stopping is completed and
   * the vehicle is standing
   */
  ros::Publisher stopping_completed_publisher_;

  //! debug publishers
  ros::Publisher lotfusspunkt_publisher_;

  /*!
   * \brief param_poll_timer calls parameter polling function
   */
  ros::Timer param_poll_timer;
};

#endif  // LONGITUDINAL_CONTROLLER_NODE_H
