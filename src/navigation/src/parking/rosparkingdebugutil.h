#ifndef ROSPARKINGDEBUGUTIL
#define ROSPARKINGDEBUGUTIL
#include <common/macros.h>

THIRD_PARTY_HEADERS_BEGIN
#include <ros/ros.h>
#include <vector>
#include <visualization_msgs/Marker.h>
THIRD_PARTY_HEADERS_END

#include "circle.h"

class RosParkingDebugUtil {
 public:

  RosParkingDebugUtil(ros::NodeHandle &nh) {
    circle_publisher = nh.advertise<visualization_msgs::Marker>("debug/support_circles", 1);
  }

  void publishSupportCircles(const std::vector<Circle>&);

 private:
  ros::Publisher circle_publisher;
};

#endif // ROSPARKINGDEBUGUTIL
