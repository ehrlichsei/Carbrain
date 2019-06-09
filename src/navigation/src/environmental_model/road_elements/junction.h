#ifndef JUNCTION_H
#define JUNCTION_H

#include "road_element.h"
#include "../tracking_element.h"
namespace environmental_model {
class Junction : public RoadElement {
 public:
  Junction(const perception_msgs::Junction& junction);
  Junction() = default;
  void accept(RoadElementVisitor& visitor, TrackingElement& tracking_element) override;
  virtual void newMsg(const MsgCollection& msg, const Eigen::Affine3d& vehicle_pose) override;
  void collectMsgs(MsgCollection& collection) override;
  void setProbabilityFromLastMsg(ProbabilityVector& probabilities) override;
  Eigen::Affine3d getPose() const;
  void merge(const Junction& junction, const Eigen::Affine3d& vehicle_pose);

  perception_msgs::Junction getMsg() const;
  bool typeRightSide() const;

  bool getObstacleWaiting() const;
  double getObstacleWaitingScore() const;
  void setObstacleWaitingScore(double obstacle_waiting_score);
  double getCertainty() const override;
  Eigen::Vector3d getStoppingPoint() const;

 private:
  void updateStoppingPoint(const Eigen::Vector3d& msg_position,
                           const Eigen::Affine3d& vehicle_pose);
  perception_msgs::Junction msg;
  double obstacle_waiting_score_ = 0;
  Eigen::Vector3d stopping_point = {0, 0, 0};
};
}  // namespace environmental_model
#endif  // JUNCTION_H
