#include "no_passing_zone_visitor.h"

namespace environmental_model {
void NoPassingZoneVisitor::visit(Unidentified & /*road_object*/,
                                 TrackingElement & /*tracking_element*/) {}

void NoPassingZoneVisitor::visit(Obstacle & /*road_object*/,
                                 TrackingElement & /*tracking_element*/) {}

void NoPassingZoneVisitor::visit(Junction &road_object, TrackingElement & /*tracking_element*/) {
  if (road_object.getMsg().junction_type == perception_msgs::Junction::TYPE_GIVEWAY_LEFT ||
      road_object.getMsg().junction_type == perception_msgs::Junction::TYPE_STOPLINE_LEFT) {
    return;
  }
  const Eigen::Affine3d junction_pose = road_object.getPose();
  no_passing_zones.emplace_back(junction_pose * Eigen::Vector3d(-0.3, 0, 0),
                                junction_pose * Eigen::Vector3d(1.0, 0, 0));
}

void NoPassingZoneVisitor::visit(Crosswalk &road_object, TrackingElement & /*tracking_element*/) {
  const Eigen::Affine3d crosswalk_pose = road_object.getPose();
  no_passing_zones.emplace_back(crosswalk_pose * Eigen::Vector3d(-0.6, 0, 0),
                                crosswalk_pose * Eigen::Vector3d(0.4, 0, 0));
}

void NoPassingZoneVisitor::visit(RoadClosure & /*road_object*/,
                                 TrackingElement & /*tracking_element*/) {}

void NoPassingZoneVisitor::visit(SpeedLimit & /*road_object*/,
                                 TrackingElement & /*tracking_element*/) {}

void NoPassingZoneVisitor::visit(Arrow & /*road_object*/,
                                 TrackingElement & /*tracking_element*/) {}

void NoPassingZoneVisitor::visit(StartLine & /*road_object*/,
                                 TrackingElement & /*tracking_element*/) {}

std::vector<NoPassingZone> NoPassingZoneVisitor::getNoPassingZones() const {
  return no_passing_zones;
}
}  // namespace environmental_model
