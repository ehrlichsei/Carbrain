#include "pavlov_node_helper.h"

PavlovNodeHelper::PavlovNodeHelper()
    : tf_listener(tf_buffer), previous_vehicle_pos(0, 0, 0) {}


void PavlovNodeHelper::stopModule() {
  reset_subscriber.shutdown();
  stopping_completed_subscriber.shutdown();
  qr_code_subscriber.shutdown();
}



bool PavlovNodeHelper::worldToVehicle(const Eigen::Vector3d& world_pos,
                                      Eigen::Vector3d& vehicle_pos) {
  try {
    auto world_to_vehicle = tf_buffer.lookupTransform("vehicle", "world", ros::Time(0));
    tf2::doTransform(world_pos, vehicle_pos, world_to_vehicle);

    return true;
  } catch (const tf2::TransformException& ex) {
    ROS_WARN_THROTTLE(1, "Can NOT transform vehicle to world: %s", ex.what());
  }

  return false;
}

bool PavlovNodeHelper::vehicleToWorld(const Eigen::Affine3d& vehicle_pose,
                                      Eigen::Affine3d& world_pose) {
  return vehicleToWorld(ros::Time(0), vehicle_pose, world_pose);
}
bool PavlovNodeHelper::vehicleToWorld(const ros::Time& stamp,
                                      const Eigen::Affine3d& vehicle_pose,
                                      Eigen::Affine3d& world_pose) {
  try {
    auto vehicle_to_world = tf_buffer.lookupTransform("world", "vehicle", stamp);
    tf2::doTransform(vehicle_pose, world_pose, vehicle_to_world);
    return true;
  } catch (const tf2::TransformException& ex) {
    ROS_WARN_THROTTLE(1, "Can NOT transform vehicle to world: %s", ex.what());
  }
  return false;
}


bool PavlovNodeHelper::pathToWorld(const ros::Time& stamp,
                                   const Eigen::Affine3d& path_pose,
                                   Eigen::Affine3d& world_pose) {
  try {
    auto path_to_world = tf_buffer.lookupTransform("world", "path", stamp);
    tf2::doTransform(path_pose, world_pose, path_to_world);
    return true;
  } catch (const tf2::TransformException& ex) {
    ROS_WARN("Can NOT transform path to world: %s", ex.what());
  }
  return false;
}
