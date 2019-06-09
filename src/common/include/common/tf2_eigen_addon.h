#ifndef TF2_EIGEN_ADDON_H
#define TF2_EIGEN_ADDON_H
#include <common/macros.h>

THIRD_PARTY_HEADERS_BEGIN
#include <ros/common.h>
#include <tf2_eigen/tf2_eigen.h>

#include <Eigen/Geometry>
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/TransformStamped.h>
THIRD_PARTY_HEADERS_END

namespace tf2 {

inline geometry_msgs::TransformStamped eigenToTransform(const tf2::Stamped<Eigen::Affine3d>& T) {
  geometry_msgs::TransformStamped transform_msg =
      eigenToTransform(static_cast<const Eigen::Affine3d&>(T));
  transform_msg.header.stamp = T.stamp_;
  transform_msg.header.frame_id = T.frame_id_;
  return transform_msg;
}

inline geometry_msgs::Vector3 toVector3Msg(const Eigen::Vector3d& in) {
  geometry_msgs::Vector3 msg;
  msg.x = in.x();
  msg.y = in.y();
  msg.z = in.z();
  return msg;
}

inline geometry_msgs::Point32 toPoint32Msg(const Eigen::Vector3f& in) {
  geometry_msgs::Point32 msg;
  msg.x = in.x();
  msg.y = in.y();
  msg.z = in.z();
  return msg;
}

// convert to transform message
inline geometry_msgs::TransformStamped toMsg(const tf2::Stamped<Eigen::Affine3d>& in,
                                             const std::string& child_frame_id) {
  geometry_msgs::TransformStamped msg = eigenToTransform(in);
  msg.child_frame_id = child_frame_id;
  return msg;
}

}  // namespace

#endif  // TF2_EIGEN_ADDON_H
