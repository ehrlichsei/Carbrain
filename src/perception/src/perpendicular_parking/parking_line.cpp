#include "parking_line.h"

THIRD_PARTY_HEADERS_BEGIN
#include <ros/console.h>
THIRD_PARTY_HEADERS_END
#include "common/eigen_utils.h"

namespace perpendicular_parking {

const std::string ParkingLine::NAMESPACE("parking_line");

const ParameterString<double> ParkingLine::MAX_TIME_WITH_NO_OBSERVATION(
    ParkingLine::NAMESPACE + "/max_time_with_no_observation");
const ParameterString<double> ParkingLine::INIT_COVARIANCE(ParkingLine::NAMESPACE +
                                                           "/init_covariance");
const ParameterString<double> ParkingLine::MAX_POSITION_DELTA(
    ParkingLine::NAMESPACE + "/max_position_delta");
const ParameterString<double> ParkingLine::MAX_ANGLE_DELTA(ParkingLine::NAMESPACE +
                                                           "/max_angle_delta");

ParkingLine::ParkingLine(const ros::Time &init_stamp,
                         const State &state,
                         const Type &type,
                         const std::size_t id,
                         const ParameterInterface *const parameters_ptr)
    : id_(id),
      type_(type),
      state_(state),
      belief_(0.0),
      last_update_time_(init_stamp),
      MAX_TIME_WITH_NO_OBSERVATION_(parameters_ptr->getParam(MAX_TIME_WITH_NO_OBSERVATION)),
      INIT_COVARIANCE_(parameters_ptr->getParam(INIT_COVARIANCE)),
      MAX_POSITION_DELTA_(parameters_ptr->getParam(MAX_POSITION_DELTA)),
      MAX_ANGLE_DELTA_(parameters_ptr->getParam(MAX_ANGLE_DELTA)) {
  covariance_ = INIT_COVARIANCE_ * Eigen::Matrix3d::Identity();
  pose_ = Eigen::Translation3d{to3D(state.head<2>())} *
          Eigen::AngleAxisd{state.z(), WorldPoint::UnitZ()};
  ROS_DEBUG("parkingline #%zu is initialized!", id_);
}

void ParkingLine::update(const ros::Time &stamp, const State &state) {
  if (last_update_time_.toSec() - stamp.toSec() > MAX_TIME_WITH_NO_OBSERVATION_) {
    ROS_DEBUG(
        "parkingline #%zu hasn't been seen for too long, re-initializing!", id_);
    covariance_ = INIT_COVARIANCE_ * Eigen::Matrix3d::Identity();
    state_ = state;
  } else {
    // recursive least squares with weighting and measurement matrices as
    // identities
    covariance_ =
        covariance_ -
        covariance_ * (Eigen::Matrix3d::Identity() + covariance_).inverse() * covariance_;
    state_ = state_ + covariance_ * (state - state_);
  }
  // set last update time to current stamp
  last_update_time_ = stamp;
  // set pose
  pose_ = Eigen::Translation3d{to3D(state.head<2>())} *
          Eigen::AngleAxisd{state.z(), WorldPoint::UnitZ()};

  // compute belief in line
  belief_ = 1.0 / (covariance_.eigenvalues().norm() + 1.0);
  ROS_DEBUG("parkingline #%zu is updated; state: (%f,%f,%f); belief: %f!",
            id_,
            state_.x(),
            state_.y(),
            state_.z(),
            belief_);
}

bool ParkingLine::operator==(const ParkingLine &other) const {
  return (to2D(state_) - to2D(other.state())).norm() < MAX_POSITION_DELTA_ &&
         std::fabs(state_.z() - other.state().z()) < MAX_ANGLE_DELTA_;
}

void ParkingLine::registerParams(common::ParameterInterface *parameters_ptr) {
  parameters_ptr->registerParam(MAX_TIME_WITH_NO_OBSERVATION);
  parameters_ptr->registerParam(INIT_COVARIANCE);
  parameters_ptr->registerParam(MAX_POSITION_DELTA);
  parameters_ptr->registerParam(MAX_ANGLE_DELTA);
}

Startline::Startline(const ros::Time &init_stamp,
                     const State &state,
                     const std::size_t id,
                     const ParameterInterface *const parameters_ptr)
    : ParkingLine(init_stamp, state, Type::START, id, parameters_ptr) {}

const std::string Endline::NAMESPACE("endline");

const ParameterString<double> Endline::MIN_FREE_SPACE(Endline::NAMESPACE +
                                                      "/min_free_space");
const ParameterString<double> Endline::MIN_FREE_SPACE_BELIEF(
    Endline::NAMESPACE + "/min_free_space_belief");

Endline::Endline(const ros::Time &init_stamp,
                 const State &state,
                 const std::size_t id,
                 const ParameterInterface *const parameters_ptr)
    : ParkingLine(init_stamp, state, Type::END, id, parameters_ptr),
      MIN_FREE_SPACE_(parameters_ptr->getParam(MIN_FREE_SPACE)),
      MIN_FREE_SPACE_BELIEF_(parameters_ptr->getParam(MIN_FREE_SPACE_BELIEF)),
      free_space_cov_(INIT_COVARIANCE_) {}

void Endline::update(const ros::Time &stamp,
                     const State &state,
                     const boost::optional<double> &free_space) {
  // update properties inherited from ParkingLine
  ParkingLine::update(stamp, state);

  // update free_space
  const auto update_val = free_space ? free_space.get() : 1.0;

  free_space_cov_ = free_space_cov_ - free_space_cov_ / (1.0 + free_space_cov_) * free_space_cov_;
  free_space_ = free_space_ + free_space_cov_ * (update_val - free_space_);

  free_space_belief_ = 1.0 / (1.0 + free_space_cov_);

  ROS_DEBUG("parkingline with type end #%zu has free_space %f with belief %f",
            id_,
            free_space_,
            free_space_belief_);

  if (free_space_ < MIN_FREE_SPACE_ && free_space_belief_ > MIN_FREE_SPACE_BELIEF_) {
    belief_ = 0.0;
    covariance_ = INIT_COVARIANCE_ * Eigen::Matrix3d::Identity();
  }
}

void Endline::registerParams(common::ParameterInterface *parameters_ptr) {
  ParkingLine::registerParams(parameters_ptr);
  parameters_ptr->registerParam(MIN_FREE_SPACE);
  parameters_ptr->registerParam(MIN_FREE_SPACE_BELIEF);
}

}  // namespace perpendicular_parking
