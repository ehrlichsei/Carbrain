#include "localization.h"
#include <common/macros.h>

THIRD_PARTY_HEADERS_BEGIN
#include <ros/console.h>
THIRD_PARTY_HEADERS_END

const ParameterString<double> Localization::INITIAL_X("initial_x");
const ParameterString<double> Localization::INITIAL_Y("initial_y");
const ParameterString<double> Localization::INITIAL_YAW("initial_yaw");
const ParameterString<double> Localization::MAX_TIME_BETWEEN_TWO_MESUREMENTS(
    "max_time_between_two_mesurements");

using namespace Eigen;

Localization::Localization(ParameterInterface* parameters)
    : parameters_ptr_(parameters),
      vehicle_world_transform_(Affine3d::Identity(), ros::Time(), "world") {
  parameters_ptr_->registerParam(INITIAL_X);
  parameters_ptr_->registerParam(INITIAL_Y);
  parameters_ptr_->registerParam(INITIAL_YAW);
  parameters_ptr_->registerParam(MAX_TIME_BETWEEN_TWO_MESUREMENTS);
}


void Localization::reset() {
  // Reset to the initial pose
  const Translation3d translation(
      parameters_ptr_->getParam(INITIAL_X), parameters_ptr_->getParam(INITIAL_Y), 0.0);
  const AngleAxisd rotation(parameters_ptr_->getParam(INITIAL_YAW), Vector3d::UnitZ());

  vehicle_world_transform_.setData(Affine3d::Identity() * translation * rotation);

  ROS_WARN("location has been reset");
}


const tf2::Stamped<Affine3d>& Localization::update(const state_estimation_msgs::State::ConstPtr& state_msg) {

  const ros::Time& last_time = vehicle_world_transform_.stamp_;
  // Get the duration from last message
  const double delta_t = (state_msg->header.stamp - last_time).toSec();

  // Integrate state_estimation data if this is not the first callback, or we
  // lost connection to
  // state_estimation for a too long time.
  if (!last_time.isZero() &&
      delta_t < parameters_ptr_->getParam(MAX_TIME_BETWEEN_TWO_MESUREMENTS)) {
    const Vector3d velocity(state_msg->speed_x, state_msg->speed_y, 0.0);
    const Translation3d delta_translation(velocity * delta_t);
    const AngleAxisd delta_rotation(state_msg->yaw_rate * delta_t, Vector3d::UnitZ());

    vehicle_world_transform_ *= delta_translation;
    vehicle_world_transform_ *= delta_rotation;
  } else {
    reset();
  }
  vehicle_world_transform_.stamp_ = state_msg->header.stamp;

  return vehicle_world_transform_;
}
