#pragma once
#include <common/macros.h>

THIRD_PARTY_HEADERS_BEGIN
#include <Eigen/Geometry>
THIRD_PARTY_HEADERS_END

namespace common {
namespace eigen_utils {
/*!
 * \brief toYaw returns the yaw-angle of the given rotation.
 *
 * It calculates the yaw angle around the z-Axis for the given AngleAxis.
 * The rotation represented by the AngleAxis has to be a pure rotation around
 * the z-axis or the result will not be correct.
 *
 * \param angle_axis the rotation.
 * \return the yaw-angle in interval [-pi, pi].
 */
template <typename T>
inline T toYaw(const Eigen::AngleAxis<T>& angle_axis) {
  if (angle_axis.axis()(2) < 0.0) {
    return -angle_axis.angle();
  } else {
    return angle_axis.angle();
  }
}

/*!
 * \brief toYaw returns the yaw-angle of the given rotation.
 *
 * It calculates the yaw angle around the z-Axis for the given 3D rotation
 * matrix.
 * The rotation represented by the rotation matrix has to be a pure rotation
 * around the z-axis or the result will not be correct.
 *
 * \param matrix the rotation.
 * \return yaw-angle in interval [-pi, pi]
 */
template <typename T>
inline T toYaw(const Eigen::Matrix<T, 3, 3>& matrix) {
  return toYaw(Eigen::AngleAxis<T>(matrix));
}

/*!
 * \brief toYaw returns the yaw-angle of the given rotation.
 *
 * It calculates the yaw angle around the z-Axis for the given quaternion.
 * The rotation represented by the quaternion has to be a pure rotation around
 * the z-axis or the result will not be correct
 *
 * \param quaternion the rotation.
 * \return angle in interval [-pi, pi].
 */
template <typename T, int O>
inline T toYaw(const Eigen::Quaternion<T, O>& quaternion) {
  return toYaw(Eigen::AngleAxis<T>(quaternion));
}

/*!
 * \brief toYaw calculates the yaw angle  of the given rotation.
 * \param rotation_matrix the rotation.
 * \return the yaw-angle in interval [-pi, pi].
 */
template <typename T>
inline T toYaw(const Eigen::Matrix<T, 2, 2>& rotation_matrix) {
  // formula was taken from Eigen::Rotation2Dd::fromRotationMatrix()
  return std::atan2(rotation_matrix.coeff(1, 0), rotation_matrix.coeff(0, 0));
}

/*!
 * \brief toRad converts a given angle from degrees to radians.
 * \param angle the angle in degrees.
 * \return the angle in radians.
 */
template <typename T>
inline T toRad(T angle) {
  return angle * T(M_PI) / T(180);
}

/*!
 * \brief toDegree convert a given angle from radians to degree.
 * \param angle the angle in radians.
 * \return the angle in degree.
 */
template <typename T>
inline T toDegree(T angle) {
  return angle * T(180) / T(M_PI);
}

}  // namespace eigen_utils;
using namespace eigen_utils;
}  // namespace common;
