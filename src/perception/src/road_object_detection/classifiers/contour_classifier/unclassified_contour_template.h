#ifndef UNCLASSIFIED_CONTOUR_TEMPLATE_H
#define UNCLASSIFIED_CONTOUR_TEMPLATE_H

#include "contour_template.h"

namespace road_object_detection {

namespace contour_classifier {

class UnclassifiedContourTemplate : public ContourTemplate {
 public:
  using ContourTemplate::ContourTemplate;
  UnclassifiedContourTemplate() = delete;

  // ContourTemplate interface
  RoadObjectPtr asRoadObject(const ros::Time& timestamp,
                             double score,
                             const ContourTrees& contour_trees) const override;

 protected:
  void writeAdditionalFields(cv::FileStorage* file_storage) const override;
  int getTemplateId() const override;
};

}  // namespace contour_classifier

} //namespace road_object_detection




#endif  // UNCLASSIFIED_CONTOUR_TEMPLATE_H
