#ifndef START_LINE_H
#define START_LINE_H

#include "road_element.h"
#include "../tracking_element.h"
namespace environmental_model {
class StartLine : public RoadElement {
 public:
  StartLine(const perception_msgs::StartLine& start_line);
  StartLine() {}
  void accept(RoadElementVisitor& visitor, TrackingElement& tracking_element) override;
  virtual void newMsg(const MsgCollection& msg, const Eigen::Affine3d& vehicle_pose) override;
  void collectMsgs(MsgCollection& collection) override;
  void setProbabilityFromLastMsg(ProbabilityVector& probabilities) override;

  perception_msgs::StartLine getMsg() const;
  double getCertainty() const override;

  Eigen::Affine3d getPose() const;

 private:
  perception_msgs::StartLine msg;
};
}  // namespace environmental_model
#endif  // START_LINE_H
