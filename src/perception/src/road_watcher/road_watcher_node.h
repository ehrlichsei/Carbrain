#ifndef ROAD_WATCHER_NODE_H
#define ROAD_WATCHER_NODE_H

#include <common/macros.h>
#include <common/node_base.h>

THIRD_PARTY_HEADERS_BEGIN
#include <memory>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <nav_msgs/Path.h>
#include <sensor_msgs/Image.h>

THIRD_PARTY_HEADERS_END

#include "../road_object_detection/road_object_visitors/message_handler.h"
#include "../utils/ego_vehicle.h"
#include "../utils/tf_helper.h"
#include "classification.h"
#include "common/camera_transformation.h"
//#include "object_tracking/object_tracker.h"
#include "road_watcher.h"

/*!
 * \brief Description
 */
class RoadWatcherNode : public NodeBase {
 public:
  virtual ~RoadWatcherNode() override = default;

  /*!
   * \brief RoadWatcherNode the constructor meant to be used in the context of
   * the
   * Nodelet, because the NodeHandle has to obtained otherways in the context of
   * Nodelets.
   * \param node_handle the NodeHandle to be used.
   */
  RoadWatcherNode(const ros::NodeHandle& node_handle);
  /*!
   * \brief returns the name of the node. It is needed in main and onInit
   * (nodelet) method.
   * \return the name of the node
   */
  static const std::string getName();

 protected:
  // NodeBase interface
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

  virtual void handleImage(const sensor_msgs::ImageConstPtr& image_raw_msg,
                           const nav_msgs::PathConstPtr& right_points,
                           const nav_msgs::PathConstPtr& middle_points,
                           const nav_msgs::PathConstPtr& left_points,
                           const nav_msgs::PathConstPtr& no_passing_points);


  std::unique_ptr<common::CameraTransformation> camera_transformation_;
  std::unique_ptr<EgoVehicle> ego_vehicle_;
  std::unique_ptr<RoadWatcher> road_watcher_;
  std::unique_ptr<Classification> classification_;
  std::unique_ptr<road_object_detection::WorldCoordinatesHelper> world_coordinates_helper_;

  static const ParameterString<double> MAX_TF_LOOKUP_DURATION;

 private:
  static const tf_helper::FrameIDs FRAME_IDS;

  tf2_ros::Buffer tf2_buffer_;
  tf2_ros::TransformListener tf2_listener_;
  tf_helper::TFHelper<double> tf_helper_;

  road_object_detection::MessageHandler message_handler_;

  message_filters::Subscriber<sensor_msgs::Image> image_raw_sub_;
  message_filters::Subscriber<nav_msgs::Path> right_points_sub;
  message_filters::Subscriber<nav_msgs::Path> middle_points_sub;
  message_filters::Subscriber<nav_msgs::Path> left_points_sub;
  message_filters::Subscriber<nav_msgs::Path> no_passing_points_sub;
  message_filters::TimeSynchronizer<sensor_msgs::Image, nav_msgs::Path, nav_msgs::Path, nav_msgs::Path, nav_msgs::Path> sync;
};

#endif  // ROAD_WATCHER_NODE_H
