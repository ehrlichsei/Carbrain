#ifndef PARKING_LINE_H
#define PARKING_LINE_H

#include <common/macros.h>

THIRD_PARTY_HEADERS_BEGIN
#include <ros/time.h>
#include <boost/optional.hpp>
#include <string>
THIRD_PARTY_HEADERS_END

#include "../../include/perception_types.h"
#include "common/parameter_interface.h"

namespace perpendicular_parking {
/*!
 * State[0] = x-coordinate in WorldFrame
 * State[1] = y-coordinate in WorldFrame
 * State[2] = angle in WorldFrame
 */
using State = Eigen::Vector3d;

using Line = Eigen::ParametrizedLine<double, 2>;
using Lines = std::vector<Line>;

enum Type { START = 1, END = -1, UNKNOWN = 0 };


class ParkingLine {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  ParkingLine(const ros::Time &init_stamp,
              const State &state,
              const Type &type,
              const std::size_t id,
              const ParameterInterface *const parameters_ptr);

  ParkingLine() = delete;

  void update(const ros::Time &stamp, const State &state);

  std::size_t id() const { return this->id_; }

  Type type() const { return this->type_; }

  WorldPose pose() const { return this->pose_; }

  double belief() const { return this->belief_; }

  State state() const { return this->state_; }

  bool operator==(const ParkingLine &other) const;

  static void registerParams(common::ParameterInterface *parameters_ptr);

  static const std::string NAMESPACE;

 protected:
  const std::size_t id_;
  Type type_;

  WorldPose pose_;

  State state_;

  double belief_;

  Eigen::Matrix3d covariance_;

  ros::Time last_update_time_;

  static const ParameterString<double> MAX_TIME_WITH_NO_OBSERVATION;
  static const ParameterString<double> INIT_COVARIANCE;
  static const ParameterString<double> MAX_POSITION_DELTA;
  static const ParameterString<double> MAX_ANGLE_DELTA;


  const double MAX_TIME_WITH_NO_OBSERVATION_;
  const double INIT_COVARIANCE_;
  const double MAX_POSITION_DELTA_;
  const double MAX_ANGLE_DELTA_;
};

class Startline : public ParkingLine {
 public:
  Startline(const ros::Time &init_stamp,
            const State &state,
            const std::size_t id,
            const ParameterInterface *const parameters_ptr);

  Startline() = delete;
};

class Endline : public ParkingLine {
 public:
  Endline(const ros::Time &init_stamp,
          const State &state,
          const std::size_t id,
          const common::ParameterInterface *const parameters_ptr);

  Endline() = delete;

  void update(const ros::Time &stamp, const State &state, const boost::optional<double> &free_space);

  static void registerParams(common::ParameterInterface *parameters_ptr);

  static const std::string NAMESPACE;

 private:
  static const ParameterString<double> MIN_FREE_SPACE;
  static const ParameterString<double> MIN_FREE_SPACE_BELIEF;

  const double MIN_FREE_SPACE_;
  const double MIN_FREE_SPACE_BELIEF_;

  double free_space_ = 0;
  double free_space_cov_;
  double free_space_belief_ = 0;
};
}  // namespace perpendicular_parking

#endif  // PARKING_LINE_H
