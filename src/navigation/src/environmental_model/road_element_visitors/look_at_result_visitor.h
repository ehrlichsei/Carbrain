#ifndef LOOK_AT_RESULT_VISITOR_H
#define LOOK_AT_RESULT_VISITOR_H

#include "road_element_visitor.h"
#include <common/macros.h>
THIRD_PARTY_HEADERS_BEGIN
#include "perception_msgs/Obstacles.h"
#include "perception_msgs/Obstacle.h"
#include "perception_msgs/Pedestrians.h"
#include "perception_msgs/Pedestrian.h"
THIRD_PARTY_HEADERS_END
#include "common/best_score.h"

namespace environmental_model {

class LookAtResultVisitor : public RoadElementVisitor {
 public:
  LookAtResultVisitor() = default;

  // RoadElementVisitor interface
 public:
  void visit(Unidentified &road_object, TrackingElement &tracking_element) override;
  void visit(Obstacle &road_object, TrackingElement &tracking_element) override;
  void visit(Junction &road_object, TrackingElement &tracking_element) override;
  void visit(Crosswalk &road_object, TrackingElement &tracking_element) override;
  void visit(RoadClosure &road_object, TrackingElement &tracking_element) override;
  void visit(SpeedLimit &road_object, TrackingElement &tracking_element) override;
  void visit(Arrow &road_object, TrackingElement &tracking_element) override;
  void visit(StartLine &road_object, TrackingElement &tracking_element) override;

  void setPedestrians(const perception_msgs::Pedestrians &pedestrians);

  void setObstacles(const perception_msgs::Obstacles &obstacles);

 private:
  template <class T>
  double getBestScoreWithId(const T &messages, const int32_t id) {
    if (messages.sub_messages.empty()) {
      return 0.0;
    }
    const auto scoringFkt = [&id](const auto &message) {
      if (id == message.id) {
        return message.certainty;
      }
      return 0.0f;
    };
    return scoringFkt(*common::max_score(messages.sub_messages, scoringFkt));
  }
  double calcNewScore(const double old_score, const double current_msg_score);
  perception_msgs::Obstacles obstacles_;
  perception_msgs::Pedestrians pedestrians_;
};

} // namespace environmental_model

#endif  // LOOK_AT_RESULT_VISITOR_H
