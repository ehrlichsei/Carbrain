#ifndef MSG_COLLECTION_H
#define MSG_COLLECTION_H

#include <common/macros.h>

THIRD_PARTY_HEADERS_BEGIN
#include <perception_msgs/Obstacles.h>
#include <perception_msgs/Crosswalks.h>
#include <perception_msgs/Junctions.h>
#include <perception_msgs/RoadClosures.h>
#include <perception_msgs/SpeedLimitMarkings.h>
#include <perception_msgs/PerpendicularParkingSpots.h>
#include <perception_msgs/Unidentifieds.h>
#include <perception_msgs/StartLines.h>
#include <perception_msgs/ParkingLotFound.h>
#include <perception_msgs/ArrowMarkings.h>
THIRD_PARTY_HEADERS_END

namespace environmental_model {

struct MsgCollection {
  perception_msgs::Crosswalks crosswalks;
  perception_msgs::Junctions junctions;
  perception_msgs::RoadClosures road_closures;
  perception_msgs::ArrowMarkings arrow_markings;
  perception_msgs::Obstacles obstacles;
  perception_msgs::StartLines start_lines;
  perception_msgs::PerpendicularParkingSpots parking_spots;
  perception_msgs::SpeedLimitMarkings speed_limits;
  perception_msgs::Unidentifieds unidentifieds;

  MsgCollection(const ros::Time time_stamp) {
    crosswalks.header.stamp = time_stamp;
    junctions.header.stamp = time_stamp;
    road_closures.header.stamp = time_stamp;
    arrow_markings.header.stamp = time_stamp;
    obstacles.header.stamp = time_stamp;
    start_lines.header.stamp = time_stamp;
    parking_spots.header.stamp = time_stamp;
    speed_limits.header.stamp = time_stamp;
    unidentifieds.header.stamp = time_stamp;
    crosswalks.header.frame_id = "world";
    junctions.header.frame_id = "world";
    road_closures.header.frame_id = "world";
    arrow_markings.header.frame_id = "world";
    obstacles.header.frame_id = "world";
    start_lines.header.frame_id = "world";
    parking_spots.header.frame_id = "world";
    speed_limits.header.frame_id = "world";
    unidentifieds.header.frame_id = "world";
  }
};

} // namespace environmental_model

#endif
