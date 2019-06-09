#ifndef COLLISION_DETECTION_NODE_H
#define COLLISION_DETECTION_NODE_H
#include <common/macros.h>

#include "common/node_base.h"
#include "navigation/driving_corridor.h"
#include "navigation/no_passing_zone.h"



#include "collision_detection.h"
#include "safety_margin.h"

THIRD_PARTY_HEADERS_BEGIN
#include "navigation_msgs/Obstacles.h"
#include "navigation_msgs/DrivingCorridor.h"
#include "navigation_msgs/NoPassingZones.h"
#include "navigation_msgs/Junctions.h"
#include "navigation_msgs/Crosswalks.h"
#include "perception_msgs/ArrowMarkings.h"
#include "perception_msgs/Junction.h"
THIRD_PARTY_HEADERS_END

static ParameterString<int> MAX_ALLOWED_PROCESSING_TIME(
    "max_allowed_processing_time");

/*!
 * \brief detects obstacles that collide with lane points
 */
class CollisionDetectionNode : public NodeBase {
 public:
  /*!
   * \brief collision_detectionNode the constructor.
   * \param node_handle the NodeHandle to be used.
   */
  CollisionDetectionNode(ros::NodeHandle& node_handle);
  /*!
   * \brief returns the name of the node. It is needed in main and onInit
   * (nodelet) method.
   * \return the name of the node
   */
  static const std::string getName();

 private:
  // NodeBase interface
  /*!
   * \brief startModule is called, if the node shall be turned active. In here
   * the subrscribers an publishers are started.
   */
  void startModule() override;
  /*!
   * \brief stopModule is called, if the node shall be turned inactive. In this
   * function subscribers and publishers are shutdown.
   */
  void stopModule() override;

  void fullCorridorCallback(const navigation_msgs::DrivingCorridor::ConstPtr& full_corridor_msg);

  void trackedObstaclesCallback(const navigation_msgs::Obstacles::ConstPtr& obstacles_msg);

  void trackedRoadClosuresCallback(const navigation_msgs::RoadClosures::ConstPtr& roadClosures_msg);

  void junctionsCallback(const navigation_msgs::Junctions::ConstPtr& junctions_msg);

  void noPassingZonesCallback(const navigation_msgs::NoPassingZones::ConstPtr& no_passing_zones_msg);

  void crosswalksCallback(const navigation_msgs::Crosswalks::ConstPtr& crosswalks_msg);

  void arrowMarkingsCallback(const perception_msgs::ArrowMarkings::ConstPtr& arrow_markings_msg);

  navigation_msgs::Obstacles::ConstPtr obstacles_msg_;
  navigation_msgs::RoadClosures::ConstPtr road_closures_msg_;
  navigation_msgs::Crosswalks::ConstPtr crosswalks_msg_;
  perception_msgs::ArrowMarkings::ConstPtr arrow_markings_msg_;

  std::unique_ptr<Eigen::Vector3d> stopline_position_;

  ros::Subscriber full_corridor_subsriber_;

  ros::Subscriber tracked_obstacles_subsriber_;

  ros::Subscriber tracked_road_closures_subsriber_;

  ros::Subscriber stop_line_subscriber_;

  ros::Subscriber no_passing_zone_subscriber_;

  ros::Subscriber crosswalk_subscriber_;

  ros::Subscriber arrow_marking_subscriber_;

  ros::Publisher safe_corridor_publisher_;

  ros::Publisher car_corridor_publisher_;

  CollisionDetection collision_detection_;
  SafetyMargin safety_margin_;

  ros::ServiceServer set_drive_past_service;

  ros::ServiceServer set_respect_no_passing_zones_service;
};

#endif  // COLLISION_DETECTION_NODE_H
