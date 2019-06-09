#ifndef PATH_PREPROCESSING_NODE_H
#define PATH_PREPROCESSING_NODE_H
#include <common/macros.h>

THIRD_PARTY_HEADERS_BEGIN
#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <geometry_msgs/PointStamped.h>

#include <nav_msgs/Path.h>
#include <navigation_msgs/DrivingCorridor.h>
#include <navigation_msgs/TurnAt.h>
#include <navigation_msgs/ParkPerpendicular.h>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <std_srvs/Empty.h>
THIRD_PARTY_HEADERS_END

#include "navigation/obstacle.h"
#include "navigation/gate.h"
#include "navigation/driving_corridor.h"

#include "common/node_base.h"

#include "path_preprocessing.h"


typedef message_filters::TimeSynchronizer<nav_msgs::Path, nav_msgs::Path, nav_msgs::Path> LaneMarkerMessageSynchronizer;

static ParameterString<int> MAX_ALLOWED_PROCESSING_TIME(
    "max_allowed_processing_time");

/*!
 * \brief The PathPreprocessingNode takes detected road lane points and
 * generates the raw target path (target path points without collision
 * avoidance).
 */
class PathPreprocessingNode : public NodeBase {
 public:
  /*!
   * \brief PathPreprocessingNode the constructor.
   * \param node_handle the NodeHandle to be used.
   */
  PathPreprocessingNode(ros::NodeHandle &node_handle);
  /*!
   * \brief returns the name of the node. It is needed in main and
   * onInit (nodelet) method.
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

  /**
   * callback for new lane points
   */
  void onNewLanePointsReceived(const nav_msgs::Path::ConstPtr &,
                               const nav_msgs::Path::ConstPtr &,
                               const nav_msgs::Path::ConstPtr &);

  /**
   * @brief clearTrackedLane clears the points of the currently tracked lane.
   * this method gets called if the autonomous mode is reentered to allow
   * reinitialization of the lane
   */
  void clearTrackedLane(const std_msgs::Empty::ConstPtr &);

  void receiveLookaheadPoint(const geometry_msgs::PointStamped &msg);

  common::EigenAlignedVector<Eigen::Vector3d> pathToWorldPoints(const nav_msgs::Path::ConstPtr &);
  void publishCorridor(const ros::Publisher &publisher,
                       const ros::Time &stamp,
                       const DrivingCorridor &corridor);
  void publishRawPath(const ros::Time &stamp,
                      const common::EigenAlignedVector<Eigen::Vector3d> &tracked_lane);
  void publishPathToWorldTransform(const Eigen::Affine3d &path_to_world_transform,
                                   const ros::Time &stamp);
  Eigen::Affine3d getVehicleToWorldTransformation(const ros::Time &transform_time) const;

  bool turnAt(navigation_msgs::TurnAt::Request &req, navigation_msgs::TurnAt::Response &res);

  bool parkPerpendicular(navigation_msgs::ParkPerpendicular::Request &req,
                         navigation_msgs::ParkPerpendicular::Response &res);
  bool reset(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);

  /*!
   * \brief contains the ROS-independent implementation of this node.
   */
  PathPreprocessing path_preprocessing_;

  tf2_ros::Buffer tf2_buffer_;
  tf2_ros::TransformListener tf2_listener_;
  tf2_ros::TransformBroadcaster tf2_broadcaster_;
  geometry_msgs::TransformStamped transform_to_world_;

  message_filters::Subscriber<nav_msgs::Path> left_line_subscriber_;
  message_filters::Subscriber<nav_msgs::Path> middle_line_subscriber_;
  message_filters::Subscriber<nav_msgs::Path> right_line_subscriber_;
  ros::Subscriber autonomous_mode_state_subscriber_;
  ros::Subscriber lookahead_point_subscriber_;
  LaneMarkerMessageSynchronizer subscriber_synchronizer_;
  ros::Publisher preprocessed_path_publisher_;
  ros::Publisher full_corridor_publisher_;
  ros::ServiceServer turning_service_;
  ros::ServiceServer perpendicular_parking_service_;
  ros::ServiceServer reset_service_;
};

#endif  // PATH_PREPROCESSING_NODE_H
