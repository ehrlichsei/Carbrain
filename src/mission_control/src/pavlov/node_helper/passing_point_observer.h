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

template <typename ID>
struct PassingPointObservation {
  ID id;
  Eigen::Affine3d world_to_entity;
  passing_condition::PassingCondition condition;
  PassingCallback<ID> cb;

 public:
  PassingPointObservation(const ID id,
                          const Eigen::Affine3d world_to_entity,
                          const passing_condition::PassingCondition condition,
                          const PassingCallback<ID> cb)
      : id(id), world_to_entity(world_to_entity), condition(condition), cb(cb) {}
};

template <typename ID>
class PassingPointObserver {
  std::vector<PassingPointObservation<ID>> observations;
  Eigen::Affine3d last_vehicle_pose;
  Eigen::Vector3d last_passed_point;

 public:
  void observePoint(const ID id,
                    const Eigen::Affine3d& pose_in_world,
                    const passing_condition::PassingCondition condition,
                    const PassingCallback<ID> cb) {
    boost::remove_erase_if(observations,
                           [id](const PassingPointObservation<ID>& existing) {
                             return existing.id == id;
                           });
    auto world_to_entity = pose_in_world.inverse();
    observations.push_back(PassingPointObservation<ID>(id, world_to_entity, condition, cb));
  }

  void checkPassedAndNotify(const tf2_ros::Buffer& buf) {
    try {
      auto vehicle_to_world = buf.lookupTransform("world", "vehicle", ros::Time(0));
      Eigen::Affine3d vehicle_pose_in_world;
      tf2::doTransform(Eigen::Affine3d::Identity(), vehicle_pose_in_world, vehicle_to_world);
      checkPassedAndNotify(vehicle_pose_in_world);
    } catch (const tf2::TransformException& ex) {
      ROS_WARN_THROTTLE(1, "Can NOT transform vehicle to world: %s", ex.what());
    }
  }
  void checkPassedAndNotify(const Eigen::Affine3d& vehicle_pose_in_world) {
    last_vehicle_pose = vehicle_pose_in_world;
    for (auto iter = observations.begin(); iter != observations.end();) {
      const Eigen::Affine3d& world_to_entity = iter->world_to_entity;
      const Eigen::Vector3d entity_pos_in_world =
          world_to_entity.inverse() * Eigen::Vector3d::Zero();
      bool passed = false;
      Eigen::Vector3d vehicle_pos_in_entity =
          world_to_entity * vehicle_pose_in_world.translation();
      const Eigen::Affine3d world_to_vehicle = vehicle_pose_in_world.inverse();
      Eigen::Vector3d entity_pos_in_vehicle = world_to_vehicle * entity_pos_in_world;


      if (iter->condition(vehicle_pos_in_entity, entity_pos_in_vehicle)) {
        passed = true;
      }

      if (passed) {
        last_passed_point = world_to_entity.translation();
        iter->cb(iter->id);
        iter = observations.erase(iter);
      } else {
        ++iter;
      }
    }
  }

  void reset() { observations.clear(); }
  Eigen::Affine3d getLastVehiclePose() { return last_vehicle_pose; }
  Eigen::Vector3d getLastPassedPoint() { return last_passed_point; }
};

#endif
