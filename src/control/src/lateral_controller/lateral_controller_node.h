#ifndef LATERAL_CONTROLLER_NODE_H
#define LATERAL_CONTROLLER_NODE_H

#include <common/macros.h>

THIRD_PARTY_HEADERS_BEGIN
#include <boost/optional.hpp>
#include <Eigen/Dense>

#include <tf2_ros/transform_listener.h>
#include <nav_msgs/Path.h>
#include <std_msgs/Float32.h>
#include <std_srvs/SetBool.h>
#include <std_srvs/Empty.h>
#include <geometry_msgs/PointStamped.h>

#include "state_estimation_msgs/State.h"
#include "navigation_msgs/AutomaticCruiseControlUpdate.h"

#include "lateral_controller_msgs/DrivingError.h"
#include "lateral_controller_msgs/DrivingSteeringAngle.h"
THIRD_PARTY_HEADERS_END

#include "common/node_base.h"
#include "lateral_control.h"


/*!
 * \brief The lateral controller generates a steering command based
 * on localization data and the given target path.
 */
class LateralControllerNode : public NodeBase {
 public:
  /*!
   * \brief LateralControllerNode the constructor.
   * \param node_handle the NodeHandle to be used.
   */
  LateralControllerNode(ros::NodeHandle& node_handle);
  /*!
   * \brief returns the name of the node. It is needed in main and onInit
   * (nodelet) method.
   * \return the name of the node
   */
  static const std::string getName();

 private:
  static const ParameterString<int> NUMBER_OF_PATH_REGRESSION_POINTS;
  static const ParameterString<double> REVERSE_DRIVING_SPEED_THRESHOLD;
  /*!
   * \brief MAX_PATH_INPUT_DELAY the maximum input delay of safe_target_path.
   * This is (ros::Time::now - stamp) in microseconds (us). If this gets
   * exceeded a warning is thrown.
   */
  static const ParameterString<int> MAX_PATH_INPUT_DELAY;
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
   * Wird ein neuer Fahrzeugzustand empfangen ruft diese Funktion den
   * Längsregler auf und published anschließend eine Message für das
   * ControllerInterface.
   *
   * \param state_msg  Zustand des Fahrzeugs
   */
  void stateEstimationCallback(const state_estimation_msgs::State::ConstPtr& state_msg);

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
   * \param target_frame_id the target frame id
   * \param source_frame_id the source frame id
   * \return the vehicle pose or nothing.
   */
  boost::optional<Eigen::Affine2d> getVehiclePose(const std::string& target_frame_id,
                                                  const std::string& source_frame_id);


  void publishSteeringAngles(double angle_front, double angle_back, const ros::Time& stamp);

  /*!
   * \brief lateral_control_ contains the ROS-indipendent implementation of
   *        the steering controller.
   */
  LateralControl lateral_control_;


  /*!
   * \brief safe_target_path_ the path
   */
  boost::optional<common::Path<>::RawPath> target_path_;
  boost::optional<std_msgs::Header> target_path_header_;


  tf2_ros::Buffer tf2_buffer_;
  tf2_ros::TransformListener tf2_listener_;


  ros::Subscriber state_estimation_subscriber_;  // kalmanfilter
  ros::Subscriber safe_target_points_subscriber_;
  ros::Subscriber auto_reset_subscriber_;
  ros::Subscriber velocity_subscriber_;

  /*!
   * \brief front_steering_angle_command_publisher_ publisher of the control
   * outputs for the front axle
   */
  ros::Publisher front_steering_angle_command_publisher_;

  /*!
   * \brief back_steering_angle_command_publisher_ publisher of the control
   * outputs for the rear axle
   */
  ros::Publisher back_steering_angle_command_publisher_;

  /*!
   * \brief lookahead_point_publisher publisher of the lookahead position in the
   * vehicle coordinate system
   */
  ros::Publisher lookahead_point_publisher;

  //! debug publishers
  ros::Publisher lotfusspunkt_publisher_;
  ros::Publisher distance_error_publisher_;

  /*!
   * \brief param_poll_timer calls parameter polling function
   */
  ros::Timer param_poll_timer;
};

#endif  // LATERAL_CONTROLLER_NODE_H
