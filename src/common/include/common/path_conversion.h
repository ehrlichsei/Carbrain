#ifndef PATH_CONVERSION_H
#define PATH_CONVERSION_H
#include <common/macros.h>

THIRD_PARTY_HEADERS_BEGIN
#include <nav_msgs/Path.h>
#include <tf2_eigen/tf2_eigen.h>
THIRD_PARTY_HEADERS_END

namespace common {
namespace msg_helper_detail {
inline auto toMsg(const Eigen::Vector3d& p) { return tf2::toMsg(p); }

inline auto toMsg(const Eigen::Vector2d& p) {
  geometry_msgs::Point msg;
  msg.x = p.x();
  msg.y = p.y();
  return msg;
}

inline void fromMsg(const geometry_msgs::Point& msg, Eigen::Vector2d& p) {
  p.x() = msg.x;
  p.y() = msg.y;
}

inline void fromMsg(const geometry_msgs::Point& msg, Eigen::Vector3d& p) {
  tf2::fromMsg(msg, p);
}

}  // namespace msg_helper_detail;

namespace msg_helper {
/*!
 * \brief createPathMsg creates a path msg from a collection of eigen vectors.
 * \param line_data the collection of eigen vectors.
 * \param time_stamp the time stamp for the message header.
 * \param frame_id the frame id for the message header.
 * \return the created Path messag.
 */
template <typename EigenVectorVector>
nav_msgs::Path createPathMsg(const EigenVectorVector& line_data,
                             const ros::Time& time_stamp,
                             const std::string& frame_id) {
  nav_msgs::Path path_msg;
  path_msg.header.stamp = time_stamp;
  path_msg.header.frame_id = frame_id;

  path_msg.poses.reserve(line_data.size());
  for (const auto& vp : line_data) {
    geometry_msgs::PoseStamped p;
    p.header = path_msg.header;
    p.pose.position = msg_helper_detail::toMsg(vp);
    p.pose.orientation.w = 1;  // Quaternion always have to have a length of 1
    path_msg.poses.push_back(p);
  }
  return path_msg;
}

/*!
 * \brief fromMsg convert a nav_msgs::Path to a std::vector of eigen vectors.
 * \param path the nav_msgs::Path.
 * \param ps the std::vector of eigen vectors.
 */
template <class PointsVector>
void fromMsg(const nav_msgs::Path& path, PointsVector& ps) {
  ps.reserve(ps.size() + path.poses.size());
  for (const auto& p_msg : path.poses) {
    if (!std::isnan(p_msg.pose.position.x) && !std::isnan(p_msg.pose.position.y)) {
      typename PointsVector::value_type p;
      msg_helper_detail::fromMsg(p_msg.pose.position, p);
      ps.push_back(p);
    }
  }
}

}  // namespace msg_helper
using namespace msg_helper;
}  // namespace common

namespace tf2 {
// for convenience and consistency with tf2_eigen.
template <class PointsVector>
void fromMsg(const nav_msgs::Path& path, PointsVector& ps) {
  common::fromMsg(path, ps);
}

}  // namespace tf2 

#endif  // PATH_CONVERSION_H
