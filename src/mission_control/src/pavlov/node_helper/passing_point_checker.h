#ifndef PASSING_POINT_OBSERVER_H
#define PASSING_POINT_OBSERVER_H
#include <common/macros.h>
THIRD_PARTY_HEADERS_BEGIN
#include <vector>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_ros/buffer.h>
#include <boost/range/algorithm_ext/erase.hpp>
THIRD_PARTY_HEADERS_END

template <typename ID>
using PassingCallback = std::function<void(const ID)>;

namespace passing_condition {
using PassingCondition =
    std::function<bool(const Eigen::Vector3d vehicle_in_entity, const Eigen::Vector3d entity_in_vehicle)>;
using PositionCondition = std::function<bool(const Eigen::Vector3d vehicle_in_entity)>;

inline PassingCondition vehicleCoordinatesInEntityCosy(const PositionCondition pos_condition) {
  return [pos_condition](const Eigen::Vector3d vehicle_in_entity,
                         const Eigen::Vector3d /*entity_in_vehicle*/) {
    return pos_condition(vehicle_in_entity);
  };
}

inline PassingCondition entityCoordinatesInVehicleCosy(const PositionCondition pos_condition) {
  return [pos_condition](const Eigen::Vector3d /*vehicle_in_entity*/,
                         const Eigen::Vector3d entity_in_vehicle) {
    return pos_condition(entity_in_vehicle);
  };
}

inline PassingCondition both(const PassingCondition a, const PassingCondition b) {
  return [a, b](const Eigen::Vector3d vehicle_in_entity, const Eigen::Vector3d entity_in_vehicle) {
    return a(vehicle_in_entity, entity_in_vehicle) && b(vehicle_in_entity, entity_in_vehicle);
  };
}

inline PositionCondition haveXGreaterThan(const double threshold) {
  return [threshold](const Eigen::Vector3d vehicle_in_entity) {
    return vehicle_in_entity[0] > threshold;
  };
}

inline PositionCondition haveXSmallerThan(const double threshold) {
  return [threshold](const Eigen::Vector3d vehicle_in_entity) {
    return vehicle_in_entity[0] < threshold;
  };
}

inline PositionCondition haveYGreaterThan(const double threshold) {
  return [threshold](const Eigen::Vector3d vehicle_in_entity) {
    return vehicle_in_entity[1] > threshold;
  };
}

inline PositionCondition haveYSmallerThan(const double threshold) {
  return [threshold](const Eigen::Vector3d vehicle_in_entity) {
    return vehicle_in_entity[1] < threshold;
  };
}
};


class PassingPointChecker {
 public:
  bool checkPoint(const Eigen::Affine3d& vehicle_pose,
                  const Eigen::Affine3d& pose_in_world,
                  const passing_condition::PassingCondition condition) {
    const Eigen::Affine3d world_to_entity = pose_in_world.inverse();

    const Eigen::Vector3d entity_pos_in_world = world_to_entity.inverse().translation();
    const Eigen::Vector3d vehicle_pos_in_entity =
        world_to_entity * vehicle_pose.translation();
    const Eigen::Affine3d world_to_vehicle = vehicle_pose.inverse();
    const Eigen::Vector3d entity_pos_in_vehicle = world_to_vehicle * entity_pos_in_world;
    return condition(vehicle_pos_in_entity, entity_pos_in_vehicle);
  }
};

#endif
