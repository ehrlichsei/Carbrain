#include <common/macros.h>

THIRD_PARTY_HEADERS_BEGIN
#include <tf2_eigen/tf2_eigen.h>
#include "control/LongitudinalControllerConfig.h"
#include <std_msgs/UInt64.h>
THIRD_PARTY_HEADERS_END

#include "common/node_creation_makros.h"
#include "common/math.h"
#include "common/eigen_utils.h"
#include "common/path.h"
#include "common/path_conversion.h"

#include "longitudinal_controller_node.h"
#include "debug/longitudinal_controller_node_debug.h"

#include "high_level_controller_helper.h"

const ParameterString<int> LongitudinalControllerNode::NUMBER_OF_PATH_REGRESSION_POINTS(
    "number_of_path_regression_points");
const ParameterString<int> LongitudinalControllerNode::MAX_PATH_INPUT_DELAY(
    "max_path_input_delay");
const ParameterString<double> LongitudinalControllerNode::PATH_TIMEOUT(
    "path_timeout");

LongitudinalControllerNode::LongitudinalControllerNode(ros::NodeHandle &node_handle)
    : NodeBase(node_handle),
      longitudinal_control_(std::make_unique<LongitudinalControl>(&parameter_handler_)),
      tf2_buffer_(ros::Duration(30)),
      tf2_listener_(tf2_buffer_) {
  parameter_handler_.addDynamicReconfigureServer<control::LongitudinalControllerConfig>(node_handle_);
  parameter_handler_.registerParam(NUMBER_OF_PATH_REGRESSION_POINTS);
  parameter_handler_.registerParam(MAX_PATH_INPUT_DELAY);
  parameter_handler_.registerParam(PATH_TIMEOUT);
}

void LongitudinalControllerNode::startModule() {

  state_estimation_subscriber_ =
      node_handle_.subscribe("state_estimation",
                             1,
                             &LongitudinalControllerNode::stateEstimationCallback,
                             this,
                             common::noDelayTransport());
  safe_target_points_subscriber_ =
      node_handle_.subscribe("safe_target_path",
                             1,
                             &LongitudinalControllerNode::safeTargetPathCallback,
                             this,
                             common::noDelayTransport());

  auto_reset_subscriber_ = node_handle_.subscribe(
      "auto_reset", 1, &LongitudinalControllerNode::autoResetCallback, this);
  velocity_subscriber_ = node_handle_.subscribe(
      "desired_speed", 1, &LongitudinalControllerNode::desiredSpeedCallback, this);
  acc_update_subscriber_ = node_handle_.subscribe(
      "acc_update", 1, &LongitudinalControllerNode::accUpdateCallback, this);

  acc_activation_ = node_handle_.advertiseService(
      "activate_acc", &LongitudinalControllerNode::accDeActivationCallback, this);
  stopping_server_ = node_handle_.advertiseService(
      "stop_at", &LongitudinalControllerNode::stopAtCallback, this);

  param_poll_timer = node_handle_.createTimer(
      ros::Duration(1, 0), &LongitudinalControllerNode::pollParams, this);

  velocity_command_publisher_ =
      node_handle_.advertise<longitudinal_controller_msgs::DrivingSpeed>(
          "velocity_command", 1);
  lotfusspunkt_publisher_ =
      node_handle_.advertise<geometry_msgs::PoseStamped>("debug/lotfusspunkt", 1);
  stopping_completed_publisher_ =
      node_handle_.advertise<std_msgs::UInt64>("stopping_completed", 1);

  longitudinal_control_->updateParams();

  // don't start driving until navigation publishes desired speed or the car
  // leaves manual mode (allows starting out of startbox)
  longitudinal_control_->setDesiredSpeed(0.0);
}

void LongitudinalControllerNode::stopModule() {
  state_estimation_subscriber_.shutdown();
  safe_target_points_subscriber_.shutdown();
  auto_reset_subscriber_.shutdown();
  velocity_subscriber_.shutdown();
  acc_update_subscriber_.shutdown();

  param_poll_timer.stop();

  acc_activation_.shutdown();
  stopping_server_.shutdown();


  longitudinal_controller_msgs::DrivingSpeed velocity_command;
  velocity_command.header.stamp = ros::Time::now();
  velocity_command.speed = 0.0;
  velocity_command_publisher_.publish(velocity_command);

  lotfusspunkt_publisher_.shutdown();
  velocity_command_publisher_.shutdown();
  stopping_completed_publisher_.shutdown();

  target_path_ = boost::none;
  target_path_header_ = boost::none;
  longitudinal_control_->reset();
}

const std::string LongitudinalControllerNode::getName() {
  return std::string("longitudinal_controller");
}


void LongitudinalControllerNode::safeTargetPathCallback(const nav_msgs::Path::ConstPtr &safe_target_path) {
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

void LongitudinalControllerNode::desiredSpeedCallback(const std_msgs::Float32 &desired_speed) {
  longitudinal_control_->setDesiredSpeed(desired_speed.data);
}

void LongitudinalControllerNode::accUpdateCallback(
    const navigation_msgs::AutomaticCruiseControlUpdate::ConstPtr &acc_update) {
  longitudinal_control_->updateACC(
      acc_update->distance, acc_update->speed, acc_update->header.stamp);
}

bool LongitudinalControllerNode::accDeActivationCallback(std_srvs::SetBool::Request &request,
                                                         std_srvs::SetBool::Response &response) {
  std::string error_message;
  if (request.data) {
    response.success = longitudinal_control_->activateACC(error_message);
  } else {
    longitudinal_control_->deactivateACC();
    response.success = true;
    error_message = "switched back to curvature controller from ACC";
  }
  response.message = error_message;
  return true;
}

bool LongitudinalControllerNode::stopAtCallback(
    longitudinal_controller_msgs::StopAt::Request &request,
    longitudinal_controller_msgs::StopAt::Response &response) {
  try {
    if (request.stop) {
      if (request.id == 0) {
        response.id = longitudinal_control_->activateStoppingController(request.distance);
      } else {
        longitudinal_control_->setStoppingDistance(request.distance, request.id);
        response.id = request.id;
      }
    } else {
      if (request.id == 0) {
        longitudinal_control_->deactivateAllStoppingControllers();
      } else {
        longitudinal_control_->deactivateStoppingController(request.id);
      }
    }
  } catch (const std::runtime_error &e) {
    ROS_ERROR("%s", e.what());
    return false;
  }
  return true;
}

void LongitudinalControllerNode::pollParams(const ros::TimerEvent &) {
  longitudinal_control_->updateParams();
}

void LongitudinalControllerNode::stateEstimationCallback(
    const state_estimation_msgs::State::ConstPtr &state_msg) {

  boost::optional<Eigen::Affine2d> vehicle_pose_in_path_frame;
  if (target_path_header_) {
    vehicle_pose_in_path_frame = getVehiclePose(target_path_header_->frame_id);
  }
  longitudinal_control_->setVehiclePoseInPathFrame(vehicle_pose_in_path_frame);

  boost::optional<common::Path<>> path;
  if (vehicle_pose_in_path_frame && target_path_) {
    if (target_path_header_ &&
        state_msg->header.stamp - target_path_header_->stamp <
            ros::Duration(parameter_handler_.getParam(PATH_TIMEOUT))) {
      try {
        path = common::Path<>(*target_path_,
                              vehicle_pose_in_path_frame->translation(),
                              parameter_handler_.getParam(NUMBER_OF_PATH_REGRESSION_POINTS));
      } catch (const std::runtime_error &e) {
        ROS_ERROR_THROTTLE(4, "stateEstimationCallback: %s", e.what());
        longitudinal_controller_msgs::DrivingSpeed velocity_command;
        velocity_command.header.stamp = state_msg->header.stamp;
        velocity_command.speed = 0.0;
        velocity_command_publisher_.publish(velocity_command);
        return;
      }
    } else {
      ROS_ERROR_THROTTLE(1,
                         "safe target path in longitudinal controller older "
                         "than %lf seconds: path timeout",
                         parameter_handler_.getParam(PATH_TIMEOUT));
    }
  }
  longitudinal_control_->setPath(path);

  //! generate set points
  const double speed = longitudinal_control_->generateSpeed(
      state_msg->speed_x, state_msg->header.stamp);

  //! creating command message
  longitudinal_controller_msgs::DrivingSpeed velocity_command;
  velocity_command.header.stamp = state_msg->header.stamp;
  velocity_command.speed = speed;
  velocity_command_publisher_.publish(velocity_command);

  //! publish message if stopping completed
  std::vector<unsigned long> stopped_ids;
  longitudinal_control_->stoppingCompleted(stopped_ids, state_msg->speed_x);
  for (auto id : stopped_ids) {
    std_msgs::UInt64 stopping_completion_msg;
    stopping_completion_msg.data = id;
    stopping_completed_publisher_.publish(stopping_completion_msg);
  }

  if (target_path_header_ && path) {
    lotfusspunkt_publisher_.publish(createLotfussPunkt(*target_path_header_, *path));
  }
}

void LongitudinalControllerNode::autoResetCallback(UNUSED const std_msgs::Empty &auto_reset) {
  ROS_WARN("auto reset");
  longitudinal_control_->reset();
}

boost::optional<Eigen::Affine2d> LongitudinalControllerNode::getVehiclePose(const std::string &frame_id) {

  // using time_stamp can result in long waiting times (about 8ms) so we use
  // the latest available.
  geometry_msgs::TransformStamped path_to_vehicle;
  try {
    path_to_vehicle = tf2_buffer_.lookupTransform(frame_id,
                                                  target_path_header_->stamp,
                                                  "front_axle",
                                                  ros::Time(0),
                                                  "world",
                                                  ros::Duration(0, 100'000));
  } catch (const tf2::TransformException &ex) {
    ROS_WARN("Can NOT look up vehicle pose in %s : %s", frame_id.c_str(), ex.what());
    return boost::none;
  }

  return to2D(tf2::transformToEigen(path_to_vehicle));
}


CREATE_NODE_WITH_FANCY_DEBUG(LongitudinalControllerNode, LongitudinalControllerNodeDebug)
