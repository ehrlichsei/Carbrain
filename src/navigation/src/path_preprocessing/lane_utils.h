#ifndef LANE_UTILS_H
#define LANE_UTILS_H

#include "common/types.h"

/**
 * @brief functionality for lane marking points used in path preprocessing
 */
class LaneUtils {
 public:
  using Lane = common::EigenAlignedVector<Eigen::Vector3d>;

  /**
   * @brief encapsultes all lane markings from different lanes
   */
  struct Lanes {
    common::EigenAlignedVector<Eigen::Vector3d> left;
    common::EigenAlignedVector<Eigen::Vector3d> middle;
    common::EigenAlignedVector<Eigen::Vector3d> right;
  };
};

#endif  // LANE_UTILS_H
