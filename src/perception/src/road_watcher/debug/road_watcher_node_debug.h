#ifndef ROAD_WATCHER_NODE_DEBUG_H
#define ROAD_WATCHER_NODE_DEBUG_H

#include "../../road_object_detection/debug/debug_images.h"
#include "../road_watcher_node.h"

class RoadWatcherNodeDebug : public RoadWatcherNode {
 public:
  RoadWatcherNodeDebug(ros::NodeHandle& node_handle);


  virtual void handleImage(const sensor_msgs::ImageConstPtr& image_raw_msg,
                           const nav_msgs::PathConstPtr& msg_right_points,
                           const nav_msgs::PathConstPtr& msg_middle_points,
                           const nav_msgs::PathConstPtr& msg_left_points,
                           const nav_msgs::PathConstPtr& no_passing_points) override;

 protected:
  virtual void startModule() override;
  virtual void stopModule() override;

 private:
  road_object_detection::DebugImages debug_images;
  ros::Publisher rospub_img_debug_;
  ros::Publisher rospub_birdsview_debug_;
  ros::Publisher rospub_canny_debug_;
  ros::Publisher rospub_field_of_vision_;
  ros::Publisher rospub_img_patch_;

  static const ParameterString<double> PIXEL_WIDTH;
  static const ParameterString<double> OFFSET_BEFORE_CENTER;
  static const ParameterString<double> OFFSET_AFTER_CENTER;
  static const ParameterString<double> OFFSET_RIGHT_OF_CENTER;
  static const ParameterString<double> OFFSET_LEFT_OF_CENTER;
  static const ParameterString<int> EXPAND_X;
  static const ParameterString<int> EXPAND_Y;
};

#endif  // ROAD_WATCHER_NODE_DEBUG_H
