#ifndef ENVIRONMENTAL_MODEL_NODE_H
#define ENVIRONMENTAL_MODEL_NODE_H

#include "common/node_base.h"
THIRD_PARTY_HEADERS_BEGIN
#include <nav_msgs/Path.h>
#include <navigation_msgs/DrivingCorridor.h>
#include <navigation_msgs/Obstacles.h>
#include <perception_msgs/Obstacles.h>
#include <perception_msgs/ArrowMarkings.h>
#include <perception_msgs/Crosswalks.h>
#include <perception_msgs/Junctions.h>
#include <navigation_msgs/Junctions.h>
#include <perception_msgs/ParkingLotFound.h>
#include <perception_msgs/PerpendicularParkingSpots.h>
#include <perception_msgs/RoadClosures.h>
#include <navigation_msgs/RoadClosures.h>
#include <perception_msgs/SpeedLimitMarkings.h>
#include <perception_msgs/StartLines.h>
#include <perception_msgs/Unidentifieds.h>
#include <navigation_msgs/TrackingElement.h>
#include <navigation_msgs/TrackingElements.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <actionlib/client/simple_action_client.h>
#include <perception_msgs/LookForObstaclesAction.h>
#include <perception_msgs/LookForPedestriansAction.h>
#include <perception_msgs/RoadObjects.h>
#include <sensor_msgs/Range.h>
#include <std_srvs/Empty.h>
THIRD_PARTY_HEADERS_END

#include "environmental_model.h"
#include "msg_collection.h"
#include "navigation/no_passing_zone.h"
#include "navigation/driving_corridor.h"

namespace environmental_model {

/*!
 * \brief Description
 */
class EnvironmentalModelNode : public NodeBase {
 public:
  /*!
   * \brief EnvironmentalModelNode the constructor.
   * \param node_handle the NodeHandle to be used.
   */
  EnvironmentalModelNode(ros::NodeHandle& node_handle);
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

  template <class T, class MSG>
  static void insertElementsFromMsg(std::vector<std::shared_ptr<RoadElement>>& road_elements,
                                    const MSG& msgs) {
    for (const auto& sub_message : msgs.sub_messages) {
      if (sub_message.certainty < 0.1) {
        continue;
      }
      road_elements.push_back(std::make_shared<T>(T(sub_message)));
      road_elements.back()->setBaseArea(sub_message.base_hull_polygon);
    }
  }

  void handleReset(const std_msgs::Empty::ConstPtr&);
  bool resetService(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res);
  void handleFullCorridor(const navigation_msgs::DrivingCorridor::ConstPtr& full_corridor_msg);
  void handleNoPassingZones(const perception_msgs::NoPassingZones& no_passing_zones,
                            const Eigen::Affine3d& vehicle_pose);

  void handleRoadWatcherMsg(const perception_msgs::RoadObjects::ConstPtr& road_objects_msg);
  void handleTOFSensor(const sensor_msgs::Range::ConstPtr& ir);
  void handleTOFAheadMsg(const sensor_msgs::Range::ConstPtr& ir_ahead);
  bool frameToWorld(const Eigen::Affine3d& frame_pose,
                    Eigen::Affine3d& world_pose,
                    const std::string& frame_id);

  void obstacleLookAtResult(const actionlib::SimpleClientGoalState& state,
                            const perception_msgs::LookForObstaclesResult::ConstPtr& result);
  void obstacleLookAtActive();
  void obstacleLookAtFeedback(const perception_msgs::LookForObstaclesFeedback::ConstPtr& feedback);

  void pedestrianLookAtResult(const actionlib::SimpleClientGoalState& state,
                              const perception_msgs::LookForPedestriansResult::ConstPtr& result);
  void pedestrianLookAtActive();
  void pedestrianLookAtFeedback(const perception_msgs::LookForPedestriansFeedback::ConstPtr& feedback);

  void publishRoadElements(const ros::Time& stamp);



  /*!
   * \brief environmental_model contains the ROS-indipendent implementation of
   * this node.
   */
  tf2_ros::TransformListener tf_listener;
  tf2_ros::Buffer tf_buffer;
  EnvironmentalModel environmental_model_;
  ros::Subscriber reset_subscriber;
  ros::Subscriber full_corridor_subscriber;
  ros::Subscriber road_objects_subscriber;

  ros::Publisher crosswalks_publisher;
  ros::Publisher junctions_publisher;
  ros::Publisher road_closures_publisher;
  ros::Publisher arrow_markings_publisher;
  ros::Publisher speed_limit_publisher;
  ros::Publisher obstacles_publisher;
  ros::Publisher start_line_publisher;
  ros::Publisher parking_spot_publisher;
  ros::Publisher unidentified_publisher;
  ros::Publisher no_passing_zone_publisher;
  ros::Publisher tracking_element_publisher;
  ros::Publisher nav_obstacles_publisher;
  actionlib::SimpleActionClient<perception_msgs::LookForObstaclesAction> obstacle_look_at_client;
  actionlib::SimpleActionClient<perception_msgs::LookForPedestriansAction> pedestrian_look_at_client;
  ros::Publisher look_at_regions_obstacle_publisher;
  ros::Publisher look_at_regions_pedestrian_publisher;
  std::shared_ptr<DrivingCorridor> driving_corridor_;
  ros::ServiceServer reset_server_;
  ros::Time time_last_road_watcher_msg_;

  ros::Subscriber ir_front_sub_;
  ros::Subscriber ir_middle_sub_;
  ros::Subscriber ir_back_sub_;

  ros::Subscriber ir_ahead_sub_;
  std::shared_ptr<LookAtVisitorParameter> look_at_visitor_parameter_;
};

}  // namespace environmental_model

#endif  // ENVIRONMENTAL_MODEL_NODE_H
