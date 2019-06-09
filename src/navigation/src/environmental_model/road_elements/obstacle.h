#ifndef ENVIRONMENTAL_MODEL_OBSTACLE_H
#define ENVIRONMENTAL_MODEL_OBSTACLE_H

#include <memory>

#include "road_element.h"
#include "../tracking_element.h"
#include "obstacle_tracker/obstacle_tracker.h"
#include "obstacle_tracker/static_obstacle_model.h"
#include "obstacle_tracker/lane_obstacle_model.h"

namespace environmental_model {
class Obstacle : public RoadElement {
 public:
  struct DynamicState {
    double prob_dynamic;
    double velocity;
  };

  Obstacle() = default;
  Obstacle(const perception_msgs::Obstacle &obstacle);
  void predict(const double time_diff_sec,
               const DrivingCorridor &driving_corridor,
               const Eigen::Affine3d &vehicle_pose) override;
  void accept(RoadElementVisitor &visitor, TrackingElement &tracking_element) override;
  virtual void newMsg(const MsgCollection &msg, const Eigen::Affine3d &vehicle_pose) override;
  void addMsgOfConflictingObstacle(const perception_msgs::Obstacle &confl_msg);
  void collectMsgs(MsgCollection &collection) override;
  void setProbabilityFromLastMsg(ProbabilityVector &probabilities) override;
  Eigen::Vector2d getEstimatedPosition() const;

  perception_msgs::Obstacle getMsg() const;

  DynamicState getDynamicState();
  bool isNew();
  bool canBeConnectedToTOFMeasurement(const TOFMeasurement &measurement);
  bool canBeConnectedToTOFMeasurementAhead(const TOFMeasurementAhead &measurement);
  void newTOFMeasurement(const TOFMeasurement &measurement);
  void newTOFMeasurementAhead(const TOFMeasurementAhead &measurement);
  bool stoppedPrediction() const;
  long getNumberUpdatesWithoutNewMeasurement() const;
  double getCertainty() const override;

 private:
  perception_msgs::Obstacle msg;
  ObstacleTracker tracker;

  void updateMessage();
  void initializeTracker(const perception_msgs::Obstacle &obstacle);
};
}  // namespace environmental_model
#endif  // ENVIRONMENTAL_MODEL_OBSTACLE_H
