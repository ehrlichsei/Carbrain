#ifndef HIGH_LEVEL_CONTROLLER_HELPER_H
#define HIGH_LEVEL_CONTROLLER_HELPER_H

#include <Eigen/Dense>
#include <geometry_msgs/PoseStamped.h>
#include <tf2_eigen/tf2_eigen.h>
#include "common/path.h"

#include "common/parameter_interface.h"

template <unsigned int MAX_REGRESSION_POLYNOMIAL_DEGREE, unsigned int DEFAULT_REGRESSION_POLYNOMIAL_DEGREE>
inline geometry_msgs::PoseStamped createLotfussPunkt(
    const std_msgs::Header header,
    const common::Path<MAX_REGRESSION_POLYNOMIAL_DEGREE, DEFAULT_REGRESSION_POLYNOMIAL_DEGREE> &path) {
  geometry_msgs::PoseStamped lotfusspunkt_msg;
  lotfusspunkt_msg.header = header;

  // angle between tangent and x-axis of path frame
  const double psi_t = std::atan(path.firstDerivative(0.0));
  lotfusspunkt_msg.pose = tf2::toMsg(
      Eigen::Affine3d(Eigen::Translation3d(path(0.0).x(), path(0.0).y(), 0.0) *
                      Eigen::AngleAxisd(psi_t, Eigen::Vector3d::UnitZ())));
  return lotfusspunkt_msg;
}

#endif  // HIGH_LEVEL_CONTROLLER_HELPER_H
