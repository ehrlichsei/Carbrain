#ifndef PATHUTILS_H
#define PATHUTILS_H
#include <common/macros.h>

THIRD_PARTY_HEADERS_BEGIN
#include <Eigen/Eigen>
#include <nav_msgs/Path.h>
THIRD_PARTY_HEADERS_END

#include "common/types.h"

/**
 * @brief functionality for path points used in path preprocessing
 */
class PathUtils {
 public:
  PathUtils() = delete;
  static common::EigenAlignedVector<Eigen::Vector3d> normalShift(
      const common::EigenAlignedVector<Eigen::Vector3d> &points, const double dist);

  /**
   * @brief smooth reduces the points of a path by reducing a fixed number of
   * points to a new point
   * @pre the input vector must contain all points in a sorted order
   * @param points the input path vector.
   * @param smooth_size the number of points that are being averaged to a new
   * point
   * @return a sorted vector with the averaged points.
   */
  static common::EigenAlignedVector<Eigen::Vector2d> smooth(
      const common::EigenAlignedVector<Eigen::Vector2d> &points, const size_t smooth_size);

  /**
   * @brief bin reduces the points of a path to obtain a fixed size of points.
   * @pre the input must be ordered.
   * @param points the input vector of pointss
   * @param num_bins the maximum number of points that are returned
   * @return a new path vector with num_bins points
   */
  static common::EigenAlignedVector<Eigen::Vector3d> bin(
      const common::EigenAlignedVector<Eigen::Vector3d> &points, const size_t num_bins);
};

#endif  // PATHUTILS_H
