#include "pathutils.h"
#include <common/macros.h>
#include <common/math.h>
#include "common/basic_statistics_eigen.h"

THIRD_PARTY_HEADERS_BEGIN
#include <ros/ros.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <boost/range/numeric.hpp>
#include <numeric>
THIRD_PARTY_HEADERS_END

common::EigenAlignedVector<Eigen::Vector3d> PathUtils::normalShift(const common::EigenAlignedVector<Eigen::Vector3d> &points,
                                                    const double dist) {
  common::EigenAlignedVector<Eigen::Vector3d> shifted_points;
  if (points.size() < 2) {
    return shifted_points;
  }

  shifted_points.reserve(points.size());

  std::vector<std::pair<size_t, Eigen::Vector3d> > normals;
  normals.reserve(points.size());
  Eigen::Vector3d average_normal = Eigen::Vector3d::Zero();
  for (size_t point_index = 1; point_index < points.size(); point_index++) {
    Eigen::Vector3d diff = points[point_index] - points[point_index - 1];
    if (diff.norm() > 0.001) {
      diff = diff.normalized();
      const Eigen::Vector3d normal(diff(1), -diff(0), 0);
      normals.emplace_back(point_index - 1, normal);
      average_normal += normal;
    }
  }

  average_normal /= normals.size();
  for (size_t shift_index = 0; shift_index < normals.size(); shift_index++) {
    const size_t point_index = normals[shift_index].first;
    const Eigen::Vector3d normal = normals[shift_index].second;
    if (normal.dot(average_normal) >= 0) {
      const Eigen::Vector3d shift = dist * normal;
      const Eigen::Vector3d shifted_point = points[point_index] + shift;
      shifted_points.push_back(shifted_point);
    } else {
      ROS_DEBUG(
          "The shift direction of path point %zu did not match the average "
          "direction.",
          shift_index);
    }
  }
  return shifted_points;
}

common::EigenAlignedVector<Eigen::Vector3d> PathUtils::bin(const common::EigenAlignedVector<Eigen::Vector3d> &points,
                                            const size_t num_bins) {
  if (points.size() <= num_bins) {
    return points;
  }

  const size_t bin_size =
      static_cast<size_t>(std::lround(points.size() / static_cast<double>(num_bins)));
  common::EigenAlignedVector<Eigen::Vector3d> binned;
  binned.reserve(num_bins);

  for (size_t i = 0; i < num_bins; i++) {
    const size_t start = std::min(i * bin_size, points.size());
    const size_t end = std::min((i + 1) * bin_size, points.size());
    if (start < end) {
      binned.push_back(common::mean(points.begin() + start, points.begin() + end));
    }
  }
  return binned;
}

common::EigenAlignedVector<Eigen::Vector2d> PathUtils::smooth(const common::EigenAlignedVector<Eigen::Vector2d> &points,
                                               const size_t smooth_size) {
  if (points.size() < 2) {
    return points;
  }

  common::EigenAlignedVector<Eigen::Vector2d> smoothed_points;
  for (size_t i = 0; i < points.size(); i += smooth_size) {
    smoothed_points.push_back(common::mean(
        points.begin() + std::max<size_t>(0, i - smooth_size),
        points.begin() + std::min(points.size() - 1, i + smooth_size)));
  }
  return smoothed_points;
}
