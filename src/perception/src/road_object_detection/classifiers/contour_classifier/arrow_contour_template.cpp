#include "arrow_contour_template.h"

namespace road_object_detection {

namespace contour_classifier {

ArrowContourTemplate::ArrowContourTemplate(const cv::FileStorage &fs)
    : ContourTemplate(fs) {

  if (!fs["type"].isInt()) {
    throw std::runtime_error(
        "ArrowContourTemplate is invalid! Required field 'type' does not "
        "exist!");
  }

  type_ = static_cast<ArrowMarking::Type>(static_cast<int>(fs["type"]));
}

RoadObjectPtr ArrowContourTemplate::asRoadObject(const ros::Time &timestamp,
                                                 double score,
                                                 const ContourTrees &contour_trees) const {
  return std::make_unique<ArrowMarking>(
      timestamp, score, contour_trees[0].vehicle_T_contour, getEnclosingRectangle(contour_trees), type_);
}

void ArrowContourTemplate::writeAdditionalFields(cv::FileStorage *fs) const {
  *fs << "type" << static_cast<int>(type_);
}

int ArrowContourTemplate::getTemplateId() const {
  return static_cast<int>(ContourTemplateType::ARROW);
}

}  // namespace contour_classifier

}  // namespace road_object_detection
