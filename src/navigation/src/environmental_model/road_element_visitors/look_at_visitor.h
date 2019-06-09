#ifndef LOOK_AT_VISITOR_H
#define LOOK_AT_VISITOR_H

#include <common/macros.h>
#include "common/parameter_handler.h"
#include "road_element_visitor.h"
#include "navigation/driving_corridor.h"
THIRD_PARTY_HEADERS_BEGIN
#include <perception_msgs/LookAtRegion.h>
#include <perception_msgs/LookAtRegions.h>
#include <perception_msgs/LookForObstaclesActionGoal.h>
#include <perception_msgs/LookForPedestriansActionGoal.h>
#include <perception_msgs/Obstacles.h>
THIRD_PARTY_HEADERS_END

namespace environmental_model {

struct LookAtVisitorParameter;

class LookAtVisitor : public RoadElementVisitor {
 public:
  struct ROI {
    ROI() = default;
    ROI(const double min_x, const double max_x, const double min_y, const double max_y);
    double min_x_ = 0;
    double max_x_ = 0;
    double min_y_ = 0;
    double max_y_ = 0;
  };
  LookAtVisitor(const std::shared_ptr<LookAtVisitorParameter> &parameter,
                const perception_msgs::Obstacles &obstacles_msg,
                const std::shared_ptr<DrivingCorridor> &driving_corridor,
                const Eigen::Affine3d &vehicle_pose);

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

  perception_msgs::LookForObstaclesGoal getObstacleActionGoal();
  perception_msgs::LookForPedestriansGoal getPedestrianActionGoal();

  perception_msgs::LookAtRegions getObstacleRois() const;
  perception_msgs::LookAtRegions getPedestriansRois() const;
  bool hasObstacleRois() const;
  bool hasPedestrianRois() const;

 private:
  Eigen::Affine3d getRoiCenterPose(const Eigen::Affine3d &road_object_pose, const ROI &roi);
  bool obstacleInRoi(const perception_msgs::Obstacle &obstacle,
                     const Eigen::Affine3d &road_object_pose,
                     const ROI &roi);
  void addObstacleRoiJunction(const Eigen::Affine3d &pose, unsigned int id, const ROI &roi);
  perception_msgs::LookAtRegion createLookAtRegion(const Eigen::Affine3d &pose,
                                                   const double width,
                                                   const double height,
                                                   unsigned int id);
  double getBestObstacleScoreInRoi(const ROI &roi, const Eigen::Affine3d &road_object_pose);
  perception_msgs::LookAtRegions obstacle_rois_;
  perception_msgs::LookAtRegions pedestrian_rois_;
  std::shared_ptr<LookAtVisitorParameter> parameter_;
  perception_msgs::Obstacles obstacles_msg_;
  std::shared_ptr<DrivingCorridor> driving_corridor_;
  Eigen::Affine3d vehicle_pose_;
};

struct LookAtVisitorParameter {
  LookAtVisitorParameter(ParameterInterface *const parameters);
  void updateParameter(ParameterInterface *const parameters);
  LookAtVisitor::ROI junction_right_area_roi_;
  LookAtVisitor::ROI junction_center_area_roi_;
  double height_roi = 0;
  double crosswalk_look_at_dist = 0;

 private:
  static const ParameterString<double> PARAM_JUNCTION_ROI_RIGHT_MIN_X;
  static const ParameterString<double> PARAM_JUNCTION_ROI_RIGHT_MIN_Y;
  static const ParameterString<double> PARAM_JUNCTION_ROI_RIGHT_MAX_X;
  static const ParameterString<double> PARAM_JUNCTION_ROI_RIGHT_MAX_Y;
  static const ParameterString<double> PARAM_JUNCTION_ROI_CENTER_MIN_X;
  static const ParameterString<double> PARAM_JUNCTION_ROI_CENTER_MIN_Y;
  static const ParameterString<double> PARAM_JUNCTION_ROI_CENTER_MAX_X;
  static const ParameterString<double> PARAM_JUNCTION_ROI_CENTER_MAX_Y;
  static const ParameterString<double> PARAM_ROAD_CLOSURE_ROI_WIDTH;
  static const ParameterString<double> PARAM_CROSSWALK_ROI_WIDTH;
  static const ParameterString<double> PARAM_CROSSWALK_ROI_HEIGHT;
  static const ParameterString<double> PARAM_CROSSWALK_LOOK_AT_DIST;
};
}  // namespace environmental_model

#endif  // LOOK_AT_VISITOR_H
