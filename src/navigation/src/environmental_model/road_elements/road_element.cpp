#include "road_element.h"
namespace environmental_model {
common::EigenAlignedVector<Eigen::Vector3d> RoadElement::getBaseArea() const {
  return base_area;
}

common::EigenAlignedVector<Eigen::Vector3d> RoadElement::convertHullPolygon(
    const std::vector<geometry_msgs::Point>& base_hull_polygon) {
  common::EigenAlignedVector<Eigen::Vector3d> vectors(base_hull_polygon.size());
  for (size_t i = 0; i < base_hull_polygon.size(); ++i) {
    tf2::fromMsg(base_hull_polygon.at(i), vectors.at(i));
  }
  return vectors;
}

void RoadElement::setBaseArea(const std::vector<geometry_msgs::Point>& base_hull_polygon) {
  base_area = convertHullPolygon(base_hull_polygon);
}

void RoadElement::setBaseAreaIfNotEmpty(const std::vector<geometry_msgs::Point>& base_hull_polygon) {
  if (!base_hull_polygon.empty()) {
    base_area = convertHullPolygon(base_hull_polygon);
  }
}

RoadElementParameter::RoadElementParameter(common::node_base::ParameterInterface* parameters) {
  parameters->registerParam(PARAM_ROAD_CLOSURE_START_DISTANCE);
  parameters->registerParam(PARAM_ROAD_CLOSURE_END_DISTANCE);
  parameters->registerParam(PARAM_MIN_ASSUMED_ROAD_CLOSURE_LENGTH);
  parameters->registerParam(PARAM_SPEED_LIMIT_SAVED_SPEED_LIMIT_MSGS_COUNT);
  parameters->registerParam(PARAM_SPEED_LIMIT_ALPHA);
  updateParameter(parameters);
}

void RoadElementParameter::updateParameter(common::node_base::ParameterInterface* parameters) {
  road_closure_start_distance = parameters->getParam(PARAM_ROAD_CLOSURE_START_DISTANCE);
  road_closure_end_distance = parameters->getParam(PARAM_ROAD_CLOSURE_END_DISTANCE);
  min_assumed_road_closure_length =
      parameters->getParam(PARAM_MIN_ASSUMED_ROAD_CLOSURE_LENGTH);
  saved_speed_limit_msgs_count =
      parameters->getParam(PARAM_SPEED_LIMIT_SAVED_SPEED_LIMIT_MSGS_COUNT);
  alpha = parameters->getParam(PARAM_SPEED_LIMIT_ALPHA);
}

}  // namespace environmental_model
