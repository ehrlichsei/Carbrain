#ifndef ARROW_CONTOUR_TEMPLATE_H
#define ARROW_CONTOUR_TEMPLATE_H

#include "contour_template.h"

namespace road_object_detection {

namespace contour_classifier {

class ArrowContourTemplate : public ContourTemplate {
 public:
  explicit ArrowContourTemplate(const cv::FileStorage& fs);
  ArrowContourTemplate() = delete;

  // ContourTemplate interface
  RoadObjectPtr asRoadObject(const ros::Time& timestamp,
                             double score,
                             const ContourTrees& contour_trees) const override;

  bool operator==(const ArrowContourTemplate& rhs) const {
    return type_ == rhs.type_;
  }

 protected:
  void writeAdditionalFields(cv::FileStorage* fs) const override;
  int getTemplateId() const override;

 private:
  ArrowMarking::Type type_;
};

}  // namespace contour_classifier

} //namespace road_object_detection




#endif  // ARROW_CONTOUR_TEMPLATE_H
