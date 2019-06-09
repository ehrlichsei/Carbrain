#include "speed_limit_contour_template.h"
#include <common/macros.h>

THIRD_PARTY_HEADERS_BEGIN
#include <ros/console.h>
THIRD_PARTY_HEADERS_END

namespace road_object_detection {

namespace contour_classifier {

SpeedLimitContourTemplate::SpeedLimitContourTemplate(const cv::FileStorage &fs)
    : ContourTemplate(fs) {
  if (!fs["speed_limit"].isInt()) {
    throw std::runtime_error(
        "SpeedLimitContourTemplate is invalid. Field 'speed_limit' does not "
        "exist!");
  }

  if (!fs["is_relieved"].isInt()) {
    throw std::runtime_error(
        "SpeedLimitContourTemplate is invalid. Required field 'is_relieved' "
        "does not exist.");
  }

  speed_limit = static_cast<int>(fs["speed_limit"]);
  is_relieved = static_cast<int>(fs["is_relieved"]);

  ROS_DEBUG_STREAM("Instantiated SpeedLimitContour with limit "
                   << speed_limit << ", is_relieved: " << is_relieved);
}

RoadObjectPtr SpeedLimitContourTemplate::asRoadObject(const ros::Time &timestamp,
                                                      double score,
                                                      const ContourTrees &contour_trees) const {
  return std::make_unique<SpeedLimitMarking>(timestamp,
                                             score,
                                             contour_trees[0].vehicle_T_contour,
                                             speed_limit,
                                             getEnclosingRectangle(contour_trees),
                                             is_relieved);
}

void SpeedLimitContourTemplate::writeAdditionalFields(cv::FileStorage *fs) const {
  *fs << "speed_limit" << speed_limit;
  *fs << "is_relieved" << is_relieved;
}

int SpeedLimitContourTemplate::getTemplateId() const {
  return static_cast<int>(ContourTemplateType::SPEED_LIMIT);
}

}  // namespace contour_classifier

}  // namespace road_object_detection
