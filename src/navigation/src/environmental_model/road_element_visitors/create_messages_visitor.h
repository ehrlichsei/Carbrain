#ifndef CREATE_MESSAGES_VISITOR_H
#define CREATE_MESSAGES_VISITOR_H

#include "road_element_visitor.h"
#include <common/macros.h>
THIRD_PARTY_HEADERS_BEGIN
#include "navigation_msgs/Junctions.h"
#include "navigation_msgs/Junction.h"
#include "perception_msgs/Obstacle.h"
#include "navigation_msgs/RoadClosures.h"
#include "navigation_msgs/RoadClosure.h"
#include "navigation_msgs/Crosswalks.h"
#include "navigation_msgs/Crosswalk.h"
THIRD_PARTY_HEADERS_END

namespace environmental_model {
class CreateMessagesVisitor : public RoadElementVisitor {
 public:
  CreateMessagesVisitor(const ros::Time &time_stamp);

  // RoadElementVisitor interface
 public:
  void visit(Unidentified &road_object, TrackingElement &tracking_element) override;
  void visit(Obstacle &road_object, TrackingElement &tracking_element) override;
  void visit(Junction &road_object, TrackingElement &tracking_element) override;
  void visit(Crosswalk &road_object, TrackingElement &tracking_element) override;
  void visit(RoadClosure &road_object, TrackingElement &tracking_element) override;
  void visit(SpeedLimit &road_object, TrackingElement &tracking_element) override;
  void visit(Arrow &road_object, TrackingElement &tracking_element) override;
  void visit(StartLine &road_object, TrackingElement &tracking_element) override;

  navigation_msgs::Crosswalks crosswalks;
  navigation_msgs::Junctions junctions;
  navigation_msgs::RoadClosures road_closures;
  perception_msgs::ArrowMarkings arrow_markings;
  perception_msgs::Obstacles obstacles;
  navigation_msgs::Obstacles navigation_obstacles;
  perception_msgs::StartLines start_lines;
  perception_msgs::PerpendicularParkingSpots parking_spots;
  perception_msgs::SpeedLimitMarkings speed_limits;
  perception_msgs::Unidentifieds unidentifieds;

 private:
  navigation_msgs::Obstacle toNavigationObstacle(const perception_msgs::Obstacle &perc_obstacle);
  bool isInAnyObstacleRoi(const geometry_msgs::Point &position, const double roi_extension);
};

}  // namespace environmental_model

#endif  // CREATE_MESSAGES_VISITOR_H
