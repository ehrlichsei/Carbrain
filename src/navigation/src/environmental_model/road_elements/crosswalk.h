#ifndef CROSSWALK_H
#define CROSSWALK_H

#include "road_element.h"
#include "../tracking_element.h"
namespace environmental_model {
class Crosswalk : public RoadElement {
 public:
  Crosswalk(const perception_msgs::Crosswalk& crosswalk);
  Crosswalk() = default;
  void accept(RoadElementVisitor& visitor, TrackingElement& tracking_element) override;
  virtual void newMsg(const MsgCollection& msg, const Eigen::Affine3d& vehicle_pose) override;
  void collectMsgs(MsgCollection& collection) override;
  void setProbabilityFromLastMsg(ProbabilityVector& probabilities) override;
  Eigen::Affine3d getPose();
  perception_msgs::Crosswalk getMsg() const;
  bool hasPedestrianWaiting() const;
  double getPedestrianWaitingScore() const;
  void setPedestrianWaitingScore(double pedestrian_waiting_score);
  double getCertainty() const override;

 private:
  perception_msgs::Crosswalk msg;
  double pedestrian_waiting_score_ = 0;
};
}  // namespace environmental_model
#endif  // CROSSWALK_H
