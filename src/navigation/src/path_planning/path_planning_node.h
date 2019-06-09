#ifndef PATH_PLANNING_NODE_H
#define PATH_PLANNING_NODE_H
#include <common/macros.h>

THIRD_PARTY_HEADERS_BEGIN
#include <memory>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <nav_msgs/Path.h>
#include "navigation_msgs/DrivingCorridor.h"
#include <std_srvs/SetBool.h>
THIRD_PARTY_HEADERS_END

#include "common/node_base.h"

#include "path_planner.h"
#include <navigation_msgs/ReverseOutOfParkingSpot.h>

static ParameterString<int> MAX_ALLOWED_PROCESSING_TIME(
    "max_allowed_processing_time");

/*!
 * \brief The PathPlanningNode generates a safe target path polynomial from
 *        raw target path points and collision prone obstacles
 */
class PathPlanningNode : public NodeBase {
 public:
  /*!
   * \brief PathPlanningNode the constructor.
   * \param node_handle the NodeHandle to be used.
   */
  PathPlanningNode(ros::NodeHandle& node_handle);
  /*!
   * \brief returns the name of the node. It is needed in main and
   * onInit (nodelet) method.
   * \return the name of the node
   */
  static const std::string getName();

  bool reverseOutOfParkingSpot(navigation_msgs::ReverseOutOfParkingSpotRequest& req,
                               navigation_msgs::ReverseOutOfParkingSpotResponse& res);
  bool outOfStartBox(std_srvs::SetBoolRequest& req, std_srvs::SetBoolResponse& res);

 protected:
  /*!
   * \brief contains the ROS-indipendent implementation of this node.
   */
  std::shared_ptr<PathPlanner> path_planner_;

  virtual void carCorridorCallback(const navigation_msgs::DrivingCorridor::ConstPtr& car_corridor_msg);

  /*!
   * \brief startModule is called, if the node shall be turned active. In here
   * the subrscribers an publishers are started.
   */
  virtual void startModule() override;
  /*!
   * \brief stopModule is called, if the node shall be turned inactive. In this
   * function subscribers and publishers are shutdown.
   */
  virtual void stopModule() override;

 private:
  // NodeBase interface

  PathPlanner::Path generateStraightPath();

  void publishReverseParkingPath(const ros::TimerEvent& /*event*/);

  typedef boost::optional<geometry_msgs::TransformStamped> OptionalTransformation;

  OptionalTransformation lookupTransformationToPath(const ros::Time& stamp,
                                              const std::string& source_frame);

  tf2_ros::Buffer tf2_buffer;
  tf2_ros::TransformListener tf2_listener;

  ros::ServiceServer reverse_out_of_parking_spot_server_;

  ros::Subscriber car_corridor_subscriber_;
  ros::Publisher safe_target_path_publisher_;

  nav_msgs::Path reverse_out_of_parking_spot_path;
  bool reverse_out_of_parking_spot = false;
  ros::Timer publishing_reverse_path_timer_;

  ros::ServiceServer out_of_start_box_server_;
  bool straight_path_out_of_start_box = false;
};

#endif  // PATH_PLANNING_NODE_H
