#include "unclassified_contour_template.h"
#include "common/assert_fail.h"

namespace road_object_detection {

namespace contour_classifier {


RoadObjectPtr UnclassifiedContourTemplate::asRoadObject(const ros::Time &/*timestamp*/,
                                                        double /*score*/,
                                                        const ContourTrees &/*contour_trees*/) const {
  assert_fail(
      "an UNCLASSIFIED contour template should not be returned as a road "
      "object!");
#if defined(NDEBUG)
  return std::nullptr_t();
#endif
}

void UnclassifiedContourTemplate::writeAdditionalFields(cv::FileStorage * /*file_storage*/) const {
}

int UnclassifiedContourTemplate::getTemplateId() const {
  return static_cast<int>(ContourTemplateType::UNCATEGORIZED);
}

}  // namespace contour_classifier


} //namespace road_object_detection

