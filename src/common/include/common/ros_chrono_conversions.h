#ifndef COMMON_ROS_CHRONO_CONVERSIONS_H
#define COMMON_ROS_CHRONO_CONVERSIONS_H

namespace common {
inline auto toBoost(const ros::Duration &d) {
  return boost::chrono::nanoseconds(d.toNSec());
}

inline auto fromBoost(const boost::chrono::nanoseconds &d) {
  return ros::Duration(0, d.count());
}

}  // namespace common

#endif // COMMON_ROS_CHRONO_CONVERSIONS_H
