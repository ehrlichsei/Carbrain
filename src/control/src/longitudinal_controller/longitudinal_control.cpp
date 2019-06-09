#include "longitudinal_control.h"

#include <ros/console.h>
#include <boost/algorithm/clamp.hpp>
#include <cmath>

#include "common/math.h"

LongitudinalControl::LongitudinalControl(ParameterInterface *parameters)
    : parameters_ptr_(parameters),
      params(std::make_shared<LongitudinalControlParams>(parameters)),
      v_soll_last_(0.0),
      curvature_controller(std::make_unique<CurvatureController>(params)),
      stopping_controllers(params) {}

void LongitudinalControl::setDesiredSpeed(const double &desired_speed) {
  curvature_controller->setDesiredSpeed(desired_speed);
  driving_direction = common::sgn(desired_speed);
}

void LongitudinalControl::setPath(const boost::optional<common::Path<> > &path) {
  curvature_controller->setPath(path);
}

void LongitudinalControl::setVehiclePoseInPathFrame(const boost::optional<Eigen::Affine2d> &vehicle_pose_in_path_frame) {
  curvature_controller->setVehiclePose(vehicle_pose_in_path_frame);
}
void LongitudinalControl::setStoppingDistance(double distance, unsigned long id) {
  stopping_controllers.setStoppingDistance(distance, id);
}

void LongitudinalControl::updateACC(double distance, double speed, const ros::Time &stamp) {
  if (acc_controller) {
    acc_controller->updateTimeStamp(stamp);
    acc_controller->setDistanceToObstacle(distance);
    acc_controller->setSpeedOfObstacle(speed);
  } else {
    ROS_WARN_DELAYED_THROTTLE(1, "received acc_update but acc is not activated");
  }
}

void LongitudinalControl::deactivateStoppingController(unsigned long id) {
  stopping_controllers.removeStoppingController(id);
  ROS_INFO("deactivated stopping controller with id %ld", id);
}

void LongitudinalControl::deactivateAllStoppingControllers() {
  stopping_controllers.reset();
  ROS_WARN("deactivated all stopping controllers");
}

bool LongitudinalControl::activateACC(std::string &message) {
  if (acc_controller) {
    message = "ACC already activated";
    ROS_WARN("%s", message.c_str());
    return true;  // TODO: should this case be considered a success or a
                  // failure?
  } else {
    acc_controller = ACC(params);
    message = "activated ACC";
    ROS_INFO("%s", message.c_str());
    return true;
  }
}

void LongitudinalControl::deactivateACC() {
  ROS_INFO("deactivated ACC");
  acc_controller = boost::none;
}


unsigned long LongitudinalControl::activateStoppingController(double distance) {
  const unsigned long id = stopping_controllers.addStoppingController(distance);
  ROS_INFO(
      "added new stopping controller with id %ld to stop in a distance of %lf "
      "m",
      id,
      distance);
  return id;
}


double LongitudinalControl::generateSpeed(const double measured_speed, const ros::Time &now) {

  static ros::Time last_time = now;
  const ros::Duration dur = now - last_time;
  if (dur < ros::Duration(0)) {
    ROS_WARN("Jump back in time, skipping control step");
    last_time = now;
    return v_soll_last_;
  }

  stopping_controllers.decrementStoppingDistance(measured_speed * dur.toSec());

  double v_soll = absMin(
      curvature_controller->calculateSpeed(measured_speed, dur, driving_direction),
      stopping_controllers.calculateSpeed(measured_speed, driving_direction));

  if (acc_controller) {
    v_soll = absMin(v_soll, acc_controller->calculateSpeed(measured_speed, driving_direction));
  }

  // don't try to drive slower than the minimal speed possible
  if (std::abs(v_soll) < params->v_min_abs) {
    v_soll = 0.0;
  }

  // limit acceleration
  if (measured_speed >= 0.0) {
    v_soll = boost::algorithm::clamp(v_soll,
                                     v_soll_last_ - params->max_dec * dur.toSec(),
                                     v_soll_last_ + params->max_acc * dur.toSec());
  } else {
    v_soll = boost::algorithm::clamp(v_soll,
                                     v_soll_last_ - params->max_acc * dur.toSec(),
                                     v_soll_last_ + params->max_dec * dur.toSec());
  }

  v_soll_last_ = v_soll;
  last_time = now;

  return v_soll;
}

void LongitudinalControl::stoppingCompleted(std::vector<unsigned long> &stopped_controllers_ids,
                                            double measured_speed) {
  stopping_controllers.getStoppedControllersIds(stopped_controllers_ids, measured_speed);
}

void LongitudinalControl::reset() {
  v_soll_last_ = 0.0;
  curvature_controller->reset();
  stopping_controllers.reset();
  acc_controller = boost::none;
  driving_direction = 1;
  ROS_WARN("reset longitudinal control");
}

void LongitudinalControl::updateParams() {
  *params = LongitudinalControlParams(parameters_ptr_);
}
