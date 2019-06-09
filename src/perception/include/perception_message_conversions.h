#ifndef PERCEPTION_MSG_CONVERSIONS
#define PERCEPTION_MSG_CONVERSIONS
#include <common/macros.h>

THIRD_PARTY_HEADERS_BEGIN
#include <boost/range/algorithm/transform.hpp>
#include <opencv/cv.h>
#include <sensor_msgs/RegionOfInterest.h>
#include "perception_msgs/Pixel.h"
#include "perception_msgs/Pixels.h"
#include "perception_msgs/QrCode.h"
#include "perception_msgs/RegionOfInterestStamped.h"
THIRD_PARTY_HEADERS_END

#include "perception_types.h"

namespace perception {
namespace message_conversion {

inline void fromMsg(const sensor_msgs::RegionOfInterest& roi_msg, cv::Rect& image_limits) {
  image_limits.x = roi_msg.x_offset;
  image_limits.y = roi_msg.y_offset;
  image_limits.width = roi_msg.width;
  image_limits.height = roi_msg.height;
}

inline cv::Rect fromMsg(const sensor_msgs::RegionOfInterest& roi_msg) {
  cv::Rect rect;
  fromMsg(roi_msg, rect);
  return rect;
}

inline sensor_msgs::RegionOfInterest toMsg(const cv::Rect& image_limits) {
  sensor_msgs::RegionOfInterest roi_msg;
  roi_msg.x_offset = image_limits.x;
  roi_msg.y_offset = image_limits.y;
  roi_msg.width = image_limits.width;
  roi_msg.height = image_limits.height;
  return roi_msg;
}

inline perception_msgs::RegionOfInterestStamped toMsg(const cv::Rect& image_limits,
                                                      const std::string& frame_id,
                                                      const ros::Time& stamp) {
  perception_msgs::RegionOfInterestStamped roi_msg;
  roi_msg.header.frame_id = frame_id;
  roi_msg.header.stamp = stamp;
  roi_msg.roi = toMsg(image_limits);
  return roi_msg;
}

inline perception_msgs::Pixel toMsg(const cv::Point& point) {
  perception_msgs::Pixel pixel;
  pixel.x = point.x;
  pixel.y = point.y;
  return pixel;
}

inline perception_msgs::Pixel toMsg(const ImagePoint& p) {
  perception_msgs::Pixel msg;
  msg.x = p.x();
  msg.y = p.y();
  return msg;
}


template <class Point, class Allocator>
perception_msgs::Pixels toMsg(const std::vector<Point, Allocator>& points,
                              const ros::Time& stamp = {}) {
  perception_msgs::Pixels pixels;
  pixels.header.stamp = stamp;
  pixels.pixels.reserve(points.size());

  boost::transform(points,
                   std::back_inserter(pixels.pixels),
                   [](const auto& point) { return toMsg(point); });

  return pixels;
}

}  // namespace message_conversion
}  // namespace perception

#endif  // PERCEPTION_MSG_CONVERSIONS
