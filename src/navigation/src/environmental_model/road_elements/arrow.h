#ifndef ARROW_H
#define ARROW_H

#include "road_element.h"
#include "../tracking_element.h"
namespace environmental_model {
class Arrow : public RoadElement {
 public:
  Arrow(const perception_msgs::ArrowMarking& arrow_marking);
  Arrow() = default;
  void accept(RoadElementVisitor& visitor, TrackingElement& tracking_element) override;
  virtual void newMsg(const MsgCollection& msg, const Eigen::Affine3d& vehicle_pose) override;
  void collectMsgs(MsgCollection& collection) override;
  void setProbabilityFromLastMsg(ProbabilityVector& probabilities) override;
  double getCertainty() const override;
  Eigen::Affine3d getPose() const;

  static std::vector<std::shared_ptr<Arrow>> createFromMsg(
      const perception_msgs::ArrowMarkings::ConstPtr& arrow_markings);

  perception_msgs::ArrowMarking getMsg() const;

 private:
  perception_msgs::ArrowMarking msg;
};
}  // namespace environmental_model
#endif  // ARROW_H
