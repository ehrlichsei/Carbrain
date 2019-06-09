#ifndef ROAD_CLOSURE_H
#define ROAD_CLOSURE_H

#include "road_element.h"
#include "../tracking_element.h"

namespace environmental_model {

class RoadClosure : public RoadElement {
 public:
  RoadClosure(const perception_msgs::RoadClosure& road_closure);
  RoadClosure(const std::shared_ptr<RoadElementParameter>& parameter);
  void accept(RoadElementVisitor& visitor, TrackingElement& tracking_element) override;
  virtual void newMsg(const MsgCollection& msg, const Eigen::Affine3d& vehicle_pose) override;
  void collectMsgs(MsgCollection& collection) override;
  void setProbabilityFromLastMsg(ProbabilityVector& probabilities) override;

  perception_msgs::RoadClosure getMsg() const;
  Eigen::Affine3d getPose() const;
  Eigen::Vector3d getStart() const;
  Eigen::Vector3d getEnd() const;
  double getWidth() const;
  Eigen::Vector3d getCenter() const;

  bool getObstacleWaiting() const;
  double getObstacleWaitingScore() const;
  void setObstacleWaitingScore(double obstacle_waiting_score);
  double getCertainty() const override;

 private:
  perception_msgs::RoadClosure msg;
  std::shared_ptr<RoadElementParameter> parameter_;
  double obstacle_waiting_score_ = 0;
};
}  // namespace environmental_model
#endif  // ROAD_CLOSURE_H
