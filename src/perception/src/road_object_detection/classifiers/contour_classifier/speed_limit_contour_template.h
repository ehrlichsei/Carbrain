#ifndef SPEED_LIMIT_CONTOUR_TEMPLATE_H
#define SPEED_LIMIT_CONTOUR_TEMPLATE_H

#include "contour_template.h"

namespace road_object_detection {

namespace contour_classifier {

class SpeedLimitContourTemplate : public ContourTemplate {
 public:
  explicit SpeedLimitContourTemplate(const cv::FileStorage& fs);
  SpeedLimitContourTemplate() = delete;

  // ContourTemplate interface
  RoadObjectPtr asRoadObject(const ros::Time& timestamp,
                             double score,
                             const ContourTrees& contour_trees) const override;

  bool operator==(const SpeedLimitContourTemplate& rhs) const {
    return (speed_limit == rhs.speed_limit) && (is_relieved == rhs.is_relieved);
  }

 protected:
  void writeAdditionalFields(cv::FileStorage* fs) const override;
  int getTemplateId() const override;

 private:
  int speed_limit;
  bool is_relieved;
};

}  // namespace contour_classifier

} //namespace road_object_detection



#endif  // SPEED_LIMIT_CONTOUR_TEMPLATE_H
