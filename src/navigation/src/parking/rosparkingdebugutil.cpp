#include "rosparkingdebugutil.h"
#include <common/macros.h>

THIRD_PARTY_HEADERS_BEGIN
#include <visualization_msgs/Marker.h>
#include <nav_msgs/Path.h>
THIRD_PARTY_HEADERS_END

void RosParkingDebugUtil::publishSupportCircles(const std::vector<Circle>& circles) {
  for (const Circle& circle : circles) {
    visualization_msgs::Marker circle_marker;
    circle_marker.type = visualization_msgs::Marker::CYLINDER;
    circle_marker.scale.x = circle.radius;
    circle_marker.scale.y = circle.radius;
    circle_marker.scale.z = 0.01f;
    circle_marker.pose.position.x = circle.x;
    circle_marker.pose.position.y = circle.y;
    circle_marker.pose.position.z = 0;
    circle_marker.color.a = 1.f;
    circle_marker.color.r = 1.f;
    circle_marker.header.frame_id = "world";
    this->circle_publisher.publish(circle_marker);
  }
}
