#include "look_at_result_visitor.h"

namespace environmental_model {


void LookAtResultVisitor::visit(Unidentified &/*road_object*/, TrackingElement &/*tracking_element*/) {
}

void LookAtResultVisitor::visit(Obstacle &/*road_object*/, TrackingElement &/*tracking_element*/) {}

void LookAtResultVisitor::visit(Junction &road_object, TrackingElement &tracking_element) {
  const double best_score = getBestScoreWithId(obstacles_, tracking_element.getId());
  road_object.setObstacleWaitingScore(
      calcNewScore(road_object.getObstacleWaitingScore(), best_score));
}

void LookAtResultVisitor::visit(Crosswalk &road_object, TrackingElement &tracking_element) {
  const double best_score = getBestScoreWithId(pedestrians_, tracking_element.getId());
  road_object.setPedestrianWaitingScore(
      calcNewScore(road_object.getPedestrianWaitingScore(), best_score));
}

void LookAtResultVisitor::visit(RoadClosure &road_object, TrackingElement &tracking_element) {
  const double best_score = getBestScoreWithId(obstacles_, tracking_element.getId());
  road_object.setObstacleWaitingScore(
      calcNewScore(road_object.getObstacleWaitingScore(), best_score));
}

void LookAtResultVisitor::visit(SpeedLimit &/*road_object*/, TrackingElement &/*tracking_element*/) {}

void LookAtResultVisitor::visit(Arrow &/*road_object*/, TrackingElement &/*tracking_element*/) {}

void LookAtResultVisitor::visit(StartLine &/*road_object*/, TrackingElement &/*tracking_element*/) {}

double LookAtResultVisitor::calcNewScore(const double old_score, const double current_msg_score) {
  if (old_score < current_msg_score) {
    return current_msg_score;
  }
  const double decrease_factor = 0.1;
  return current_msg_score * decrease_factor + old_score * (1 - decrease_factor);
}

void LookAtResultVisitor::setObstacles(const perception_msgs::Obstacles &obstacles) {
  obstacles_ = obstacles;
}


void LookAtResultVisitor::setPedestrians(const perception_msgs::Pedestrians &pedestrians) {
  pedestrians_ = pedestrians;
}

} // namespace environmental_model
