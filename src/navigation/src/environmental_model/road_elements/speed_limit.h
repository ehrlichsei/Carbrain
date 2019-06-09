#ifndef SPEED_LIMIT_H
#define SPEED_LIMIT_H

#include "road_element.h"
#include <deque>
#include "../tracking_element.h"
#include <Eigen/Dense>
namespace environmental_model {
class SpeedLimit : public RoadElement {
 public:
  SpeedLimit(const perception_msgs::SpeedLimitMarking& speed_limit);
  SpeedLimit(const std::shared_ptr<RoadElementParameter>& parameter);
  void accept(RoadElementVisitor& visitor, TrackingElement& tracking_element) override;
  virtual void newMsg(const MsgCollection& msg, const Eigen::Affine3d& vehicle_pose) override;
  void collectMsgs(MsgCollection& collection) override;
  void setProbabilityFromLastMsg(ProbabilityVector& probabilities) override;

  perception_msgs::SpeedLimitMarking getMsg() const;
  double getCertainty() const override;

 private:
  perception_msgs::SpeedLimitMarking msg;
  std::shared_ptr<RoadElementParameter> parameter_;
  // std::deque<int> previous_speed_limits;
  std::deque<Eigen::Matrix<float, 10, 1>> previous_certainties_queu;
  // std::array<int, 10> previous_speed_limits_count{{0}};
};
}  // namespace environmental_model
#endif  // SPEED_LIMIT_H
