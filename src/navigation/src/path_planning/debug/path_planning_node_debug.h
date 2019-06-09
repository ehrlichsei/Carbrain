#ifndef PATH_PLANNING_NODE_DEBUG_H
#define PATH_PLANNING_NODE_DEBUG_H
#include <common/macros.h>

THIRD_PARTY_HEADERS_BEGIN
#include <ros/node_handle.h>
THIRD_PARTY_HEADERS_END

#include <navigation_msgs/CeresSummary.h>

#include "../path_planning_node.h"

/**
 * class for generating debug output for PathPlanningNodeDebug and classes used
 * by this class.
*/
class PathPlanningNodeDebug : public PathPlanningNode {
 public:
  PathPlanningNodeDebug(ros::NodeHandle& node_handle);

 protected:
  virtual void carCorridorCallback(const navigation_msgs::DrivingCorridor::ConstPtr& car_corridor_msg) override;
  virtual void startModule() override;
  virtual void stopModule() override;

 private:
  std::shared_ptr<navigation_msgs::CeresSummary> ceres_summary;
  ros::Publisher ceres_summary_publisher;
};

#endif  // PATH_PLANNING_NODE_DEBUG_H
