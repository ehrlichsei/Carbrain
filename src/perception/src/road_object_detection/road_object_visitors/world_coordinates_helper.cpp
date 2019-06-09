#include "world_coordinates_helper.h"
#include <ros/console.h>

namespace road_object_detection {

void WorldCoordinatesHelper::calcWorldCoordinates(RoadObjects &road_objects) {
  RoadObjectVisitor::visit(road_objects);
}

void WorldCoordinatesHelper::visit(Unidentified &road_object) {
  transformVehicleToWorldPose(road_object.pose_in_vehicle, road_object.pose_in_world);
  transformVehicleToWorldPoints(road_object.base_hull_polygon_in_vehicle,
                                road_object.base_hull_polygon_in_world);
}

void WorldCoordinatesHelper::visit(Obstacle &road_object) {
  transformVehicleToWorldPoints(road_object.base_hull_polygon_in_vehicle,
                                road_object.base_hull_polygon_in_world);
}

void WorldCoordinatesHelper::visit(Junction &road_object) {
  transformVehicleToWorldPose(road_object.pose_in_vehicle, road_object.pose_in_world);
  transformVehicleToWorldPoints(road_object.base_hull_polygon_in_vehicle,
                                road_object.base_hull_polygon_in_world);
}

void WorldCoordinatesHelper::visit(Crosswalk &road_object) {
  transformVehicleToWorldPose(road_object.pose_in_vehicle, road_object.pose_in_world);
  transformVehicleToWorldPoints(road_object.base_hull_polygon_in_vehicle,
                                road_object.base_hull_polygon_in_world);
}

void WorldCoordinatesHelper::visit(RoadClosure &road_object) {
  transformVehicleToWorldPose(road_object.pose_in_vehicle, road_object.pose_in_world);
  transformVehicleToWorldPoints(road_object.base_hull_polygon_in_vehicle,
                                road_object.base_hull_polygon_in_world);
}

void WorldCoordinatesHelper::visit(SpeedLimitMarking &road_object) {
  transformVehicleToWorldPose(road_object.pose_in_vehicle, road_object.pose_in_world);
  transformVehicleToWorldPoints(road_object.base_hull_polygon_in_vehicle,
                                road_object.base_hull_polygon_in_world);
}

void WorldCoordinatesHelper::visit(StartLine &road_object) {
  transformVehicleToWorldPose(road_object.pose_in_vehicle, road_object.pose_in_world);
  transformVehicleToWorldPoints(road_object.base_hull_polygon_in_vehicle,
                                road_object.base_hull_polygon_in_world);
}

void WorldCoordinatesHelper::visit(Pedestrian &road_object) {
  transformVehicleToWorldPose(road_object.pose_in_vehicle, road_object.pose_in_world);
  transformVehicleToWorldPoints(road_object.base_hull_polygon_in_vehicle,
                                road_object.base_hull_polygon_in_world);
}

void WorldCoordinatesHelper::visit(NoPassingZone &road_object) {
  transformVehicleToWorldPose(road_object.pose_in_vehicle, road_object.pose_in_world);
  transformVehicleToWorldPoints(road_object.base_hull_polygon_in_vehicle,
                                road_object.base_hull_polygon_in_world);

  road_object.start_point_world = tf_helper_->getTransform() * road_object.start_point_vehicle;
  road_object.end_point_world = tf_helper_->getTransform() * road_object.end_point_vehicle;
}

void WorldCoordinatesHelper::visit(ArrowMarking &road_object) {
  transformVehicleToWorldPose(road_object.pose_in_vehicle, road_object.pose_in_world);
  transformVehicleToWorldPoints(road_object.base_hull_polygon_in_vehicle,
                                road_object.base_hull_polygon_in_world);
}

void WorldCoordinatesHelper::transformVehicleToWorldPose(const VehiclePose &vehicle_pose,
                                                         WorldPose &world_pose) const {
  world_pose = transformVehicleToWorldP(vehicle_pose);
}

void WorldCoordinatesHelper::transformVehicleToWorldPoints(const VehiclePoints &vehicle_points,
                                                           WorldPoints &world_points) const {
  world_points.resize(vehicle_points.size());
  std::transform(vehicle_points.begin(), vehicle_points.end(), world_points.begin(), transformVehicleToWorld);
}

}  // namespace road_object_detection
