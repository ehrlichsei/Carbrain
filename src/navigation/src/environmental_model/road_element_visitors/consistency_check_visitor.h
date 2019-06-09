#ifndef CONSISTENCY_CHECK_VISITOR_H
#define CONSISTENCY_CHECK_VISITOR_H

#include "road_element_visitor.h"
#include "navigation/no_passing_zone.h"
#include "common/best_score.h"
#include "common/angle_conversions.h"
#include <perception_msgs/LookAtRegions.h>

namespace environmental_model {
class ConsistencyCheckVisitor : public RoadElementVisitor {
  struct ObstacleInformation {
    ObstacleInformation(TrackingElement *tracking_element,
                        Obstacle *obstacle,
                        bool on_left_lane,
                        const Eigen::Vector3d &position,
                        const double prob_dynamic)
        : tracking_element(tracking_element),
          obstacle(obstacle),
          on_left_lane(on_left_lane),
          position(position),
          prob_dynamic(prob_dynamic) {}
    TrackingElement *tracking_element;
    Obstacle *obstacle;
    bool on_left_lane;
    Eigen::Vector3d position;
    double dist_to_left_gate_border = 0.0;
    double dist_to_right_gate_border = 0.0;
    double prob_dynamic = 0.0;
  };
  struct JunctionInformation {
    JunctionInformation(TrackingElement *tracking_element, Junction *junction)
        : tracking_element(tracking_element), junction(junction) {}
    TrackingElement *tracking_element;
    Junction *junction;
  };
  struct ArrowInformation {
    ArrowInformation(TrackingElement *tracking_element, Arrow *arrow)
        : tracking_element(tracking_element), arrow(arrow) {}
    TrackingElement *tracking_element;
    Arrow *arrow;
  };


 public:
  void visit(Obstacle &road_object, TrackingElement &tracking_element) override;
  void visit(StartLine &road_object, TrackingElement &tracking_element) override;
  void visit(Junction &road_object, TrackingElement &tracking_element) override;
  void visit(Arrow &road_object, TrackingElement &tracking_element) override;

  static void makeModelValid(const Eigen::Affine3d &vehicle_pose,
                             const DrivingCorridor &driving_corridor,
                             std::vector<TrackingElement> &tracking_elements,
                             const std::deque<std::pair<Eigen::Vector3d, ros::Time> > &last_vehicle_positions,
                             NoPassingZones &no_passing_zones);
  static void deleteAllObstaclesInRoi(std::vector<TrackingElement> &tracking_elements,
                                      const perception_msgs::LookAtRegions &obstacle_look_at_rois);

 private:
  static bool isInAnyObstacleRoi(const Eigen::Vector2d &position,
                                 const double roi_extension,
                                 const perception_msgs::LookAtRegions &obstacle_look_at_rois);
  // constructor is private because obstacle_informations_ contains pointer
  // which can get invalid
  ConsistencyCheckVisitor(const DrivingCorridor &driving_corridor);
  void increaseProbMatchingJunctions();
  void makeNoPassingZonesValid(const Eigen::Affine3d &vehicle_pose,
                               const std::deque<std::pair<Eigen::Vector3d, ros::Time> > &last_vehicle_positions,
                               NoPassingZones &no_passing_zones);
  void makeObstaclePositionsValid(std::vector<TrackingElement> &tracking_elements);
  bool conflictingObstaclesAreSameDynamicObstacle(std::vector<unsigned int> &conflicting_ids,
                                                  const ObstacleInformation &a,
                                                  const ObstacleInformation &b) const;

  double getAngleInDegreeToCorridor(const Eigen::Affine3d &pose);

  void deleteTrackingElementsNotInCorridor(std::vector<TrackingElement> &tracking_elements);
  void deleteInvalidStartLines(std::vector<TrackingElement> &tracking_elements);
  void deleteInvalidJunctions(std::vector<TrackingElement> &tracking_elements);
  std::vector<unsigned int> getIdsOfConflictingObstacles();
  std::vector<ObstacleInformation> obstacle_informations_;
  std::vector<JunctionInformation> junction_informations_;
  std::vector<ArrowInformation> arrow_informations_;
  std::set<unsigned int> start_lines_to_delete_;
  std::set<unsigned int> junctions_to_delete_;
  DrivingCorridor driving_corridor_approx_;
};
}  // namespace environmental_model

#endif  // CONSISTENCY_CHECK_VISITOR_H
