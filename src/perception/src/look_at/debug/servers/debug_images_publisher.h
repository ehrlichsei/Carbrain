#ifndef DEBUG_IMAGES_PUBLISHER_H
#define DEBUG_IMAGES_PUBLISHER_H

#include "../../../road_object_detection/debug/debug_images.h"
#include "../../look_at.h"
#include "common/parameter_interface.h"

namespace look_at {

class DebugImagesPublisher {
 public:
  DebugImagesPublisher(const ros::NodeHandle &node_handle, ParameterInterface *parameters);

  virtual ~DebugImagesPublisher() = default;

  void advertise(const std::string &action_name);

  void shutdown();

 protected:
  void publishDebugImages(const RegionsToClassify &rois, const ros::Time &stamp);

  road_object_detection::DebugImages debug_images;
  ros::Publisher rospub_img_debug_;
  ros::Publisher rospub_birdsview_debug_;
  ros::Publisher rospub_canny_debug_;
  ros::Publisher rospub_area_of_interest_;
  ros::Publisher rospub_img_patch_;
  ros::NodeHandle node_handle_;


 private:
  std::array<cv::Point2f, 4> getROIInWorld(const RegionToClassify &roi_in_world) const;

  const ParameterInterface *const parameters_ptr_;

  static const ParameterString<double> PIXEL_WIDTH;
  static const ParameterString<double> OFFSET_BEFORE_CENTER;
  static const ParameterString<double> OFFSET_AFTER_CENTER;
  static const ParameterString<double> OFFSET_RIGHT_OF_CENTER;
  static const ParameterString<double> OFFSET_LEFT_OF_CENTER;
  static const ParameterString<int> EXPAND_X;
  static const ParameterString<int> EXPAND_Y;
};
}  // namespace look_at

#endif  // SERVER_INTERFACE_DEBUG_H
