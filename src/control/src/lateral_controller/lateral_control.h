#ifndef LATERAL_CONTROL_H
#define LATERAL_CONTROL_H

#include <common/macros.h>

THIRD_PARTY_HEADERS_BEGIN
#include <geometry_msgs/PoseStamped.h>
#include <Eigen/Geometry>
THIRD_PARTY_HEADERS_END

#include "common/parameter_interface.h"
#include "common/path.h"


class LateralControl {
 public:
  struct Params {
    static const ParameterString<double> WHEELBASE;
    static const ParameterString<double> MAX_STEERING_ANGLE_LEFT;
    static const ParameterString<double> MAX_STEERING_ANGLE_RIGHT;

    static const ParameterString<double> DIST_K;
    static const ParameterString<double> EPSILON_DIVISION_BY_ZERO;
    static const ParameterString<double> LOOKAHEAD_DELTA;
    static const ParameterString<double> K_AG;
    static const ParameterString<double> K_YAW;

    static const ParameterString<double> MIN_LATERAL_CONTROL_SPEED;


    double wheelbase;
    double max_steering_angle_left;
    double max_steering_angle_right;

    double dist_k;
    double k_ag;
    double k_yaw;
    double epsilon_division_by_zero;
    double lookahead_delta;

    double min_control_speed;


    Params(const ParameterInterface* parameters);

    static void registerParams(ParameterInterface* parameters);
  } params;

  /*!
   * \brief LateralControl is the consstructor. A ros indipendent
   * functionality containing
   * class needs a pointer to a ParameterInterface (in fact a ParameterHandler)
   * to get access to parameters.
   * \param parameters the ParameterInterface
   */
  LateralControl(ParameterInterface* parameters);

  /*!
   * \brief generateSteeringAngle calculates the steering angle necessary to
   * follow the path.
   * \param path the current path.
   * \param vehicle_pose the current vehicle_pose.
   * \param measured_speed_x the measured speed in x-direction.
   * \param measured_speed_y the measured speed in y-direction.
   * \param measured_yaw_rate the measured yaw-rate.
   * \param driving_reverse wheter to drive reverse.
   * \param steering_angle_front the new front steering angle.
   * \param steering_angle_back the new back steering angle.
   * \param lookahead_distance the distance to look ahead.
   */
  void generateSteeringAngle(const common::Path<>& path,
                             const Eigen::Affine2d& vehicle_pose,
                             const double measured_speed_x,
                             const double measured_speed_y,
                             const double measured_yaw_rate,
                             bool driving_reverse,
                             double& steering_angle_front,
                             double& steering_angle_back,
                             double& lookahead_distance);

  void updateParams();

 private:
  /*!
  * \brief parameters_ptr_ is needed for parameter access
  */
  const ParameterInterface* parameters_ptr_;
};

#endif  // LATERAL_CONTROL_H
