#ifndef OBSTACLE_PAVLOV_NODE_H
#define OBSTACLE_PAVLOV_NODE_H

#include <common/macros.h>
#include "common/node_base.h"
#include "node_helper/pavlov_node_helper.h"
#include "node_helper/state_machine_logger.h"
#include "obstacle_fsm/hl_fsm.h"
#include "obstacle_fsm/shared_data.h"
THIRD_PARTY_HEADERS_BEGIN
#include <queue>
#include "navigation_msgs/Junctions.h"
THIRD_PARTY_HEADERS_END

#define TOPIC_CROSSWALKS "crosswalks"
#define TOPIC_JUNCTIONS "junctions"
#define TOPIC_ROAD_CLOSURES "road_closures"
#define TOPIC_ARROW_MARKINGS "arrow_markings"
#define TOPIC_SPEED_LIMIT_MARKINGS "speed_limit_marking"

const ParameterString<double> PARAM_JUNCTION_AREA_OF_INTEREST_RIGHT_MIN_X(
    "junction_roi_right_min_x");
const ParameterString<double> PARAM_JUNCTION_AREA_OF_INTEREST_RIGHT_MIN_Y(
    "junction_roi_right_min_y");
const ParameterString<double> PARAM_JUNCTION_AREA_OF_INTEREST_RIGHT_MAX_X(
    "junction_roi_right_max_x");
const ParameterString<double> PARAM_JUNCTION_AREA_OF_INTEREST_RIGHT_MAX_Y(
    "junction_roi_right_max_y");

const ParameterString<double> PARAM_JUNCTION_AREA_OF_INTEREST_CENTER_MIN_X(
    "junction_roi_center_min_x");
const ParameterString<double> PARAM_JUNCTION_AREA_OF_INTEREST_CENTER_MIN_Y(
    "junction_roi_center_min_y");
const ParameterString<double> PARAM_JUNCTION_AREA_OF_INTEREST_CENTER_MAX_X(
    "junction_roi_center_max_x");
const ParameterString<double> PARAM_JUNCTION_AREA_OF_INTEREST_CENTER_MAX_Y(
    "junction_roi_center_max_y");
const ParameterString<double> PARAM_NO_PASSING_LENGTH_AREA_OF_INTEREST(
    "no_passing_length_area_of_interest");
const ParameterString<double> PARAM_TURN_LEFT_PASSED_THRES_X(
    "turn_left_passed_thres_x");
const ParameterString<double> PARAM_TURN_LEFT_PASSED_THRES_Y(
    "turn_left_passed_thres_y");
const ParameterString<double> PARAM_TURN_RIGHT_PASSED_THRES_X(
    "turn_right_passed_thres_x");
const ParameterString<double> PARAM_TURN_RIGHT_PASSED_THRES_Y(
    "turn_right_passed_thres_y");
const ParameterString<double> PARAM_NO_PASSING_DISTANCE_TO_OBSTACLE(
    "no_passing_distance_to_obstacle");

const ParameterString<double> PARAM_NO_PASSING_FINSIH_OVERTAKING(
    "no_passing_finish_overtaking");
const ParameterString<double> PARAM_CROSSWALK_PASSED_THRESH_X(
    "crosswalk_passed_thres_x");
const ParameterString<double> PARAM_VEHICLE_WIDTH("vehicle_width");
const ParameterString<bool> PARAM_ENABLE_QR_CODE_DETECTION(
    "enable_qr_code_detection");


class ObstaclePavlovNode : public NodeBase {
 public:
  using StateMachine =
      sml::sm<SM::ObstacleStateMachine, sml::logger<StateMachineLogger>, sml::defer_queue<std::queue>, sml::process_queue<std::queue>>;
  /*!
   * \brief ObstaclePavlovNode the constructor.
   * \param node_handle the NodeHandle to be used.
   */
  ObstaclePavlovNode(ros::NodeHandle& node_handle);
  /*!
   * \brief returns the name of the node. It is needed in main and onInit
   * (nodelet) method.
   * \return the name of the node
   */
  static const std::string getName();

 private:
  template <class MSG>
  MSG sortMessages(const MSG& messages) {
    MSG msg = messages;
    boost::sort(msg.sub_messages, [this](const auto& a, const auto& b) {
      return std::abs(this->calcEuclidianDistanceInFront(a.pose.position)) <
             std::abs(this->calcEuclidianDistanceInFront(b.pose.position));
    });
    return msg;
  }
  void loadParameters();
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

  void handleCrosswalks(const navigation_msgs::Crosswalks::ConstPtr& msg);
  void handleJunctions(const navigation_msgs::Junctions::ConstPtr& msg);
  void handleRoadClosures(const navigation_msgs::RoadClosures::ConstPtr& msg);
  void handleArrowMarkings(const perception_msgs::ArrowMarkings::ConstPtr& msg);
  void handleFullCorridor(const navigation_msgs::DrivingCorridor::ConstPtr& full_corridor_msg);
  void handleObstacles(const navigation_msgs::Obstacles::ConstPtr& obstacles);
  void handleSpeedLimitMarkings(const perception_msgs::SpeedLimitMarkings::ConstPtr& msg);
  void handleUnidentified(const perception_msgs::Unidentifieds::ConstPtr& unidentifieds);
  void handleNoPassingZones(const navigation_msgs::NoPassingZones::ConstPtr& no_passing_zones);
  void handleReset(const std_msgs::Empty::ConstPtr&);

  double calcEuclidianDistanceInFront(const Eigen::Vector3d& p_world);
  double calcEuclidianDistanceInFront(const geometry_msgs::Point& p);

  // Debug publishing
  void publishPoseTransform(const geometry_msgs::Pose& pose, const std::string& name);
  void publishPoseTransform(const Eigen::Affine3d& world_to_pose, const std::string& name);
  void publishTurnTransform(const Eigen::Affine3d& tf);
  void publishLookAtTransform(const tf2::Stamped<Eigen::Affine3d>& tf);
  void timerDebugMsgCallback(const ros::TimerEvent&);

  /*!
   * \brief carCorridorCallback gets called by the ros callback handler when the
   * navigation module
   * publishes a new driving corridor. The received gates are stored for later
   * distance calculations
   */
  void carCorridorCallback(const navigation_msgs::DrivingCorridor::ConstPtr car_corridor_msg);

  void publishCurrentStates();
  PassingPointChecker passing_point_checker;
  tf2_ros::TransformBroadcaster tf_broadcaster;
  DrivingCorridorChecker driving_corridor_checker;
  TurnChecker turn_checker;

  Eigen::Vector3d turn_left_passed_thres;
  Eigen::Vector3d turn_right_passed_thres;

  Eigen::AlignedBox2d junction_right_roi;
  Eigen::AlignedBox2d junction_center_roi;

  ros::Subscriber crosswalks_subscriber;
  ros::Subscriber junctions_subscriber;
  ros::Subscriber road_closures_subscriber;
  ros::Subscriber arrow_markings_subscriber;
  ros::Subscriber speed_limit_subscriber;

  ros::Subscriber full_corridor_subscriber;
  ros::Subscriber obstacles_subscriber;

  ros::Subscriber unidentified_subscriber;
  ros::Subscriber no_passing_zone_subscriber;
  ros::Publisher diagnostics_publisher;
  ros::Subscriber reset_subscriber;

  StateMachineLogger logger_;
  CarController car_controller;
  DiagnosticsInterface diagnostics_iface;
  std::shared_ptr<SharedData> shared_data_;
  // state machine
  std::shared_ptr<StateMachine> obstacle_state_machine;

  PavlovNodeHelper pavlov_node_helper_;

  ros::Publisher state_machine_publisher_;
  ros::Timer timer_debug_msg_;
};

#endif  // OBSTACLE_PAVLOV_NODE_H
