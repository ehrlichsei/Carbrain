#ifndef UNIDENTIFIED_H
#define UNIDENTIFIED_H

#include "road_element.h"
#include "../tracking_element.h"
namespace environmental_model {
class Unidentified : public RoadElement {
 public:
  Unidentified(const perception_msgs::Unidentified& unidentified);
  Unidentified() {}
  void accept(RoadElementVisitor& visitor, TrackingElement& tracking_element) override;
  virtual void newMsg(const MsgCollection& msg, const Eigen::Affine3d& vehicle_pose) override;
  void collectMsgs(MsgCollection& collection) override;
  void setProbabilityFromLastMsg(ProbabilityVector& probabilities) override;

  perception_msgs::Unidentified getMsg() const;
  double getCertainty() const override;

 private:
  perception_msgs::Unidentified msg;
};
}  // namespace environmental_model
#endif  // UNIDENTIFIED_H
