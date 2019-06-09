#ifndef SPLINE_HELPER_H
#define SPLINE_HELPER_H
#include <common/macros.h>

THIRD_PARTY_HEADERS_BEGIN
#include <tk_spline_catkin/spline.h>
#include <geometry_msgs/PoseStamped.h>
THIRD_PARTY_HEADERS_END

/*!
 * \brief make_spline_and_sort_poses generate spline interpolating the poses with a cubic
 * spline along the x-axis. Throws runtime_error exception if no spline can be
 * generated out of poses (more than three points with sufficient distance to
 * each other along the x-axis required).
 * \param poses
 * \return spline interpolating the poses
 */
inline tk::spline make_spline_and_sort_poses(std::vector<geometry_msgs::PoseStamped>& poses) {
  if (poses.empty()) {
    throw std::runtime_error("passed empty poses to make_spline");
  }
  std::sort(poses.begin(),
            poses.end(),
            [](const auto& x, const auto& y) {
              return x.pose.position.x < y.pose.position.x;
            });
  std::vector<double> xs;
  xs.reserve(poses.size());
  std::vector<double> ys;
  ys.reserve(poses.size());
  xs.push_back(poses.front().pose.position.x);
  ys.push_back(poses.front().pose.position.y);
  for (const auto& pose : poses) {
    if (pose.pose.position.x > xs.back() + 0.001) {
      xs.push_back(pose.pose.position.x);
      ys.push_back(pose.pose.position.y);
    }
  }

  if (xs.size() < 3) {
    throw std::runtime_error("Only " + std::to_string(xs.size()) +
                             " poses remaining after discarding points too "
                             "close to each other.\nAt least 3 poses required "
                             "to generate spline.");
  }

  return tk::spline(std::move(xs), std::move(ys));
}

/*!
 * \brief make_spline generate spline interpolating the poses with a cubic
 * spline along the x-axis. Throws runtime_error exception if no spline can be
 * generated out of poses (more than three points with sufficient distance to
 * each other along the x-axis required).
 * \param poses
 * \return spline interpolating the poses
 */
inline tk::spline make_spline(std::vector<geometry_msgs::PoseStamped> poses) {
  return make_spline_and_sort_poses(poses);
}

inline double curvature(const tk::spline& spline, double x) {
  const double dy = spline.deriv(1, x);
  const double ddy = spline.deriv(2, x);

  return ddy / std::pow(1.0 + dy * dy, 1.5);
}

#endif  // SPLINE_HELPER_H
