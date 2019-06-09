#include <common/macros.h>

THIRD_PARTY_HEADERS_BEGIN
#include <tf2_eigen/tf2_eigen.h>
#include "control/LateralControllerConfig.h"
#include <std_msgs/Time.h>
#include <ros/console.h>
THIRD_PARTY_HEADERS_END

#include "lateral_controller_node.h"
#include "common/node_creation_makros.h"
#include "common/math.h"
#include "common/eigen_utils.h"
#include "common/path.h"
#include "common/types.h"
#include "common/path_conversion.h"

#include "high_level_controller_helper.h"

const ParameterString<int> LateralControllerNode::NUMBER_OF_PATH_REGRESSION_POINTS(
    "number_of_path_regression_points");
const ParameterString<double> LateralControllerNode::REVERSE_DRIVING_SPEED_THRESHOLD(
    "reverse_driving_speed_threshold");
const ParameterString<int> LateralControllerNode::MAX_PATH_INPUT_DELAY(
    "max_path_input_delay");

LateralControllerNode::LateralControllerNode(ros::NodeHandle &node_handle)
    : NodeBase(node_handle),
      lateral_control_(&parameter_handler_),
      tf2_buffer_(ros::Duration(30)),
      tf2_listener_(tf2_buffer_) {
  parameter_handler_.addDynamicReconfigureServer<control::LateralControllerConfig>(node_handle_);
  parameter_handler_.registerParam(NUMBER_OF_PATH_REGRESSION_POINTS);
  parameter_handler_.registerParam(REVERSE_DRIVING_SPEED_THRESHOLD);
  parameter_handler_.registerParam(MAX_PATH_INPUT_DELAY);
}

void LateralControllerNode::startModule() {

  state_estimation_subscriber_ =
      node_handle_.subscribe("state_estimation",
                             1,
                             &LateralControllerNode::stateEstimationCallback,
                             this,
                             common::noDelayTransport());
  safe_target_points_subscriber_ =
      node_handle_.subscribe("safe_target_path",
                             1,
                             &LateralControllerNode::safeTargetPathCallback,
                             this,
                             common::noDelayTransport());

  param_poll_timer = node_handle_.createTimer(
      ros::Duration(1, 0), &LateralControllerNode::pollParams, this);

  lotfusspunkt_publisher_ =
      node_handle_.advertise<geometry_msgs::PoseStamped>("debug/lotfusspunkt", 1);
  distance_error_publisher_ = node_handle_.advertise<lateral_controller_msgs::DrivingError>(
      "debug/distance_error", 1);
  front_steering_angle_command_publisher_ =
      node_handle_.advertise<lateral_controller_msgs::DrivingSteeringAngle>(
          "steering_angle_command_front", 1);
  back_steering_angle_command_publisher_ =
      node_handle_.advertise<lateral_controller_msgs::DrivingSteeringAngle>(
          "steering_angle_command_back", 1);
  lookahead_point_publisher =
      node_handle_.advertise<geometry_msgs::PointStamped>("lookahead_point", 1);

  lateral_control_.updateParams();
}

void LateralControllerNode::stopModule() {
  state_estimation_subscriber_.shutdown();
  safe_target_points_subscriber_.shutdown();
  auto_reset_subscriber_.shutdown();

  param_poll_timer.stop();

  publishSteeringAngles(0.0, 0.0, ros::Time::now());

  lotfusspunkt_publisher_.shutdown();
  distance_error_publisher_.shutdown();
  front_steering_angle_command_publisher_.shutdown();
  back_steering_angle_command_publisher_.shutdown();
  lookahead_point_publisher.shutdown();

  target_path_ = boost::none;
  target_path_header_ = boost::none;
}

const std::string LateralControllerNode::getName() {
  return std::string("lateral_controller");
}


void LateralControllerNode::safeTargetPathCallback(const nav_msgs::Path::ConstPtr &safe_target_path) {
  const auto delay = (ros::Time::now() - safe_target_path->header.stamp).toNSec() / 1000;
  if (delay > parameter_handler_.getParam(MAX_PATH_INPUT_DELAY)) {
    ROS_WARN_THROTTLE(
        1, "safe_target_path is too old: %lu us [throttled to 1 Hz]", delay);
  }
  common::Vector2dVector poses;
  tf2::fromMsg(*safe_target_path, poses);
  try {
    target_path_ = common::Path<>::RawPath(std::move(poses));
    target_path_header_ = safe_target_path->header;
  } catch (const std::runtime_error &e) {
    ROS_WARN_THROTTLE(4, "safeTargetPathCallback: %s", e.what());
  }
}

void LateralControllerNode::stateEstimationCallback(const state_estimation_msgs::State::ConstPtr &state_msg) {

  if (!target_path_) {
    ROS_DEBUG_THROTTLE(
        1, "Cannot generate velocity command: Path not initialized!");
    return;
  }

  std::string source_frame_id;
  bool driving_reverse;
  if (state_msg->speed_x < parameter_handler_.getParam(REVERSE_DRIVING_SPEED_THRESHOLD)) {
    driving_reverse = true;
    source_frame_id = "vehicle";
    ROS_INFO_THROTTLE(8,
                      "stanley driving in reverse with controlled rear axle");
  } else {
    driving_reverse = false;
    source_frame_id = "front_axle";
  }

  boost::optional<Eigen::Affine2d> vehicle_pose =
      getVehiclePose(target_path_header_->frame_id, source_frame_id);

  if (vehicle_pose) {

    //! Bestimmen der Sollgrößen @note Hier ist der Reglercode!
    try {
      // this might throw
      const common::Path<> path(*target_path_,
                                vehicle_pose->translation(),
                                parameter_handler_.getParam(NUMBER_OF_PATH_REGRESSION_POINTS));

      //! generate set points
      double steering_angle_front = 0.0;
      double steering_angle_back = 0.0;
      double lookahead_distance = 0.0;
      lateral_control_.generateSteeringAngle(path,
                                             vehicle_pose.get(),
                                             state_msg->speed_x,
                                             state_msg->speed_y,
                                             state_msg->yaw_rate,
                                             driving_reverse,
                                             steering_angle_front,
                                             steering_angle_back,
                                             lookahead_distance);

      //! creating command message
      publishSteeringAngles(
          steering_angle_front, steering_angle_back, state_msg->header.stamp);

      boost::optional<Eigen::Affine2d> vehicle_frame =
          getVehiclePose(target_path_header_->frame_id, "vehicle");
      if (vehicle_frame) {
        const Eigen::Vector2d lookahead_position_in_path = path(lookahead_distance);
        const Eigen::Vector2d lookahead_position_in_vehicle =
            vehicle_frame->inverse() * lookahead_position_in_path;
        geometry_msgs::PointStamped lookahead_point_msg;
        lookahead_point_msg.header.stamp = state_msg->header.stamp;
        lookahead_point_msg.header.frame_id = "vehicle";
        lookahead_point_msg.point.x = lookahead_position_in_vehicle.x();
        lookahead_point_msg.point.y = lookahead_position_in_vehicle.y();
        lookahead_point_msg.point.z = 0.0;
        lookahead_point_publisher.publish(lookahead_point_msg);
      }

      //! publish perpendicular foot for debug purposes
      lotfusspunkt_publisher_.publish(createLotfussPunkt(*target_path_header_, path));

      /*! Publish the distance error to allow parameter evaluation,
       * especially to make rqt_autotune possible!
       */
      lateral_controller_msgs::DrivingError dist_error_command;
      dist_error_command.header.stamp = state_msg->header.stamp;

      dist_error_command.error = (path(0.0) - vehicle_pose->translation()).norm();

      distance_error_publisher_.publish(dist_error_command);
    } catch (const std::runtime_error &e) {
      ROS_WARN_THROTTLE(4, "%s", e.what());
      publishSteeringAngles(0.0, 0.0, state_msg->header.stamp);
    }
  }
}

void LateralControllerNode::pollParams(const ros::TimerEvent &) {
  lateral_control_.updateParams();
}


boost::optional<Eigen::Affine2d> LateralControllerNode::getVehiclePose(
    const std::string &target_frame_id, const std::string &source_frame_id) {

  // using time_stamp can result in long waiting times (about 8ms) so we use the
  // latest available.
  geometry_msgs::TransformStamped path_to_vehicle;
  try {
    path_to_vehicle = tf2_buffer_.lookupTransform(target_frame_id,
                                                  target_path_header_->stamp,
                                                  source_frame_id,
                                                  ros::Time(0),
                                                  "world",
                                                  ros::Duration(0, 100'000));
  } catch (const tf2::TransformException &ex) {
    ROS_WARN("Can NOT look up %s pose in %s : %s",
             source_frame_id.c_str(),
             target_frame_id.c_str(),
             ex.what());
    return boost::none;
  }

  return to2D(tf2::transformToEigen(path_to_vehicle));
}

void LateralControllerNode::publishSteeringAngles(double angle_front,
                                                  double angle_back,
                                                  const ros::Time &stamp) {
  lateral_controller_msgs::DrivingSteeringAngle front_steering_angle_command;
  front_steering_angle_command.header.stamp = stamp;
  front_steering_angle_command.steering_angle = angle_front;
  front_steering_angle_command_publisher_.publish(front_steering_angle_command);

  lateral_controller_msgs::DrivingSteeringAngle back_steering_angle_command;
  back_steering_angle_command.header.stamp = stamp;
  back_steering_angle_command.steering_angle = angle_back;
  back_steering_angle_command_publisher_.publish(back_steering_angle_command);
}


CREATE_NODE(LateralControllerNode)
