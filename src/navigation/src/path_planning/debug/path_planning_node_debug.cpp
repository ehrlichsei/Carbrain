#include "path_planning_node_debug.h"

#include "least_squares_planner_debug.h"

PathPlanningNodeDebug::PathPlanningNodeDebug(ros::NodeHandle& node_handle)
    : PathPlanningNode(node_handle),
      ceres_summary(new navigation_msgs::CeresSummary()) {
  path_planner_ = std::make_unique<LeastSquaresPlannerDebug>(parameter_handler_, ceres_summary);
}

void PathPlanningNodeDebug::carCorridorCallback(const navigation_msgs::DrivingCorridor::ConstPtr& car_corridor_msg) {
  PathPlanningNode::carCorridorCallback(car_corridor_msg);
  ceres_summary_publisher.publish(*ceres_summary);
}

void PathPlanningNodeDebug::startModule() {
  PathPlanningNode::startModule();
  ceres_summary_publisher =
      node_handle_.advertise<navigation_msgs::CeresSummary>("ceres_summary", 1);
}

void PathPlanningNodeDebug::stopModule() {
  PathPlanningNode::stopModule();
  ceres_summary_publisher.shutdown();
}
