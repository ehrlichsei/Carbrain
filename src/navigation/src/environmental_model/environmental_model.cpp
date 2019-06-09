#include "environmental_model.h"
#include "common/best_score.h"
#include "road_element_visitors/road_element_visitor.h"
#include "road_element_visitors/consistency_check_visitor.h"
THIRD_PARTY_HEADERS_BEGIN
#include <boost/range/algorithm_ext/erase.hpp>
THIRD_PARTY_HEADERS_END

namespace environmental_model {

EnvironmentalModel::EnvironmentalModel(ParameterInterface* parameters)
    : parameters_ptr_(parameters),
      road_element_parameter_(std::make_shared<RoadElementParameter>(parameters_ptr_)),
      tracking_element_parameter_(std::make_shared<TrackingElementParameter>(
          parameters, road_element_parameter_)),
      no_passing_zones_parameter_(std::make_shared<NoPassingZonesParameter>(parameters)),
      no_passing_zones(no_passing_zones_parameter_),
      matching_(tracking_element_parameter_) {}

void EnvironmentalModel::predict(const double time_diff_sec,
                                 const Eigen::Affine3d& vehicle_pose) {
  for (auto& tracking_element : tracking_elements) {
    tracking_element.predict(time_diff_sec, full_corridor_, vehicle_pose);
  }
}

void EnvironmentalModel::reset() {
  tracking_elements.clear();
  const int ignore_number_messages_after_reset = 20;
  no_passing_zones.reset(ignore_number_messages_after_reset);
}

void EnvironmentalModel::updateParameter() {
  road_element_parameter_->updateParameter(parameters_ptr_);
  no_passing_zones_parameter_->updateParameter(parameters_ptr_);
  tracking_element_parameter_->updateParameter(parameters_ptr_);
}

void EnvironmentalModel::update(std::vector<std::shared_ptr<RoadElement>>& new_road_elements,
                                const perception_msgs::Unidentifieds& unidentifieds,
                                const Eigen::Affine3d& vehicle_pose) {
  ros::Time current_time = ros::Time::now();
  if (last_vehicle_positions_.empty() ||
      (current_time - last_vehicle_positions_.back().second) > ros::Duration(0.15)) {
    if (last_vehicle_positions_.size() > 20) {  // save last 3 seconds
      last_vehicle_positions_.pop_front();
    }
    last_vehicle_positions_.emplace_back(vehicle_pose.translation(), current_time);
  }
  matching_.assignNewElementsToTracks(
      new_road_elements, tracking_elements, unidentifieds, vehicle_pose);

  boost::remove_erase_if(tracking_elements,
                         [&](const auto& element) {
                           return element.shouldBeDeleted(vehicle_pose, full_corridor_);
                         });
  ConsistencyCheckVisitor::makeModelValid(
      vehicle_pose, full_corridor_, tracking_elements, last_vehicle_positions_, no_passing_zones);
}

void EnvironmentalModel::collectMsgs(navigation_msgs::TrackingElements& tracking_elements_msg) {
  for (auto& tracking_element : tracking_elements) {
    tracking_element.collectMsgs(tracking_elements_msg);
  }
}


NoPassingZones& EnvironmentalModel::getNoPassingZones() {
  return no_passing_zones;
}

void EnvironmentalModel::updateFullCorridor(const DrivingCorridor& full_corridor) {
  full_corridor_ = full_corridor;
}

void EnvironmentalModel::receivedTOFMeasurement(const TOFMeasurement& measurement) {
  if (tracking_elements.empty()) {
    return;
  }
  class ObstacleGetter : public RoadElementVisitor {
   public:
    explicit ObstacleGetter(const TOFMeasurement& measurement)
        : measurement(measurement) {}
    void visit(Obstacle& road_object, TrackingElement& /*tracking_element*/) override {
      if (road_object.canBeConnectedToTOFMeasurement(measurement)) {
        obstacles.push_back(&road_object);
      }
    }
    std::vector<Obstacle*> obstacles;
    const TOFMeasurement& measurement;
  } visitor(measurement);

  visitElements(visitor);
  if (visitor.obstacles.empty()) {
    return;
  }
  const Eigen::Vector3d point_search_obstacle =
      measurement.rayHitsObstacle() ? measurement.reflection_point
                                    : measurement.tof_pose.translation();
  Obstacle* nearest_obstacle =
      *common::min_score(visitor.obstacles,
                         [&point_search_obstacle](const Obstacle* obstacle) {
                           return (obstacle->getEstimatedPosition() -
                                   point_search_obstacle.topRows<2>()).norm();
                         });
  nearest_obstacle->newTOFMeasurement(measurement);
}

void EnvironmentalModel::receivedTOFMeasurementAhead(const TOFMeasurementAhead& measurement) {
  if (tracking_elements.empty()) {
    return;
  }
  class ObstacleGetter : public RoadElementVisitor {
   public:
    explicit ObstacleGetter(const TOFMeasurementAhead& measurement)
        : measurement(measurement) {}
    void visit(Obstacle& road_object, TrackingElement& /*tracking_element*/) override {
      if (road_object.canBeConnectedToTOFMeasurementAhead(measurement)) {
        obstacles.push_back(&road_object);
      }
    }
    std::vector<Obstacle*> obstacles;
    const TOFMeasurementAhead& measurement;
  } visitor(measurement);

  visitElements(visitor);
  if (visitor.obstacles.empty()) {
    return;
  }
  Obstacle* nearest_obstacle =
      *common::min_score(visitor.obstacles,
                         [&measurement](const Obstacle* obstacle) {
                           return (obstacle->getEstimatedPosition() -
                                   measurement.reflection_point.topRows<2>()).norm();
                         });
  nearest_obstacle->newTOFMeasurementAhead(measurement);
}


void EnvironmentalModel::deleteAllObstaclesInRoi(const perception_msgs::LookAtRegions& obstacle_look_at_rois) {
  ConsistencyCheckVisitor::deleteAllObstaclesInRoi(tracking_elements, obstacle_look_at_rois);
}

}  // namespace environmental_model
