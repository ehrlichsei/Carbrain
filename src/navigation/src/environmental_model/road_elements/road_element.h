#ifndef ROAD_ELEMENT_H
#define ROAD_ELEMENT_H

#include "common/node_base.h"
THIRD_PARTY_HEADERS_BEGIN
#include "tf2_eigen/tf2_eigen.h"
#include <Eigen/Dense>
#include <nav_msgs/Path.h>
#include <navigation_msgs/Obstacles.h>
#include <perception_msgs/ArrowMarkings.h>
#include <perception_msgs/Crosswalks.h>
#include <perception_msgs/Junctions.h>
#include <perception_msgs/PerpendicularParkingSpots.h>
#include <perception_msgs/RoadClosures.h>
#include <perception_msgs/SpeedLimitMarkings.h>
#include <perception_msgs/StartLines.h>
THIRD_PARTY_HEADERS_END

#include "navigation/driving_corridor.h"
#include "common/types.h"
#include "../msg_collection.h"
namespace environmental_model {
const ParameterString<double> PARAM_ROAD_CLOSURE_START_DISTANCE =
    ParameterString<double>("road_closure/road_closure_start_distance");
const ParameterString<double> PARAM_ROAD_CLOSURE_END_DISTANCE =
    ParameterString<double>("road_closure/road_closure_end_distance");
const ParameterString<double> PARAM_MIN_ASSUMED_ROAD_CLOSURE_LENGTH =
    ParameterString<double>("road_closure/min_assumed_road_closure_length");
const ParameterString<int> PARAM_SPEED_LIMIT_SAVED_SPEED_LIMIT_MSGS_COUNT =
    ParameterString<int>("speed_limit/saved_speed_limit_msgs_count");
const ParameterString<double> PARAM_SPEED_LIMIT_ALPHA =
    ParameterString<double>("speed_limit/alpha");
struct RoadElementParameter {
  RoadElementParameter(ParameterInterface* parameters);
  void updateParameter(ParameterInterface* parameters);
  double road_closure_start_distance = 0.0;
  double road_closure_end_distance = 0.0;
  double min_assumed_road_closure_length = 0.0;
  size_t saved_speed_limit_msgs_count = 0;
  double alpha = 0.0;
};


class TrackingElement;
class RoadElementVisitor;
class RoadElement {
 public:
  using ProbabilityVector = Eigen::Matrix<double, 8, 1>;
  RoadElement() = default;
  RoadElement(const RoadElement&) = default;
  RoadElement(RoadElement&&) = default;
  RoadElement& operator=(RoadElement&& other) = default;
  virtual ~RoadElement() = default;
  virtual void collectMsgs(MsgCollection& collection) = 0;
  virtual void setProbabilityFromLastMsg(ProbabilityVector& probabilities) = 0;

  virtual void accept(RoadElementVisitor& visitor, TrackingElement& tracking_element) = 0;
  virtual double getCertainty() const = 0;

  common::EigenAlignedVector<Eigen::Vector3d> getBaseArea() const;
  void setBaseArea(const std::vector<geometry_msgs::Point>& base_hull_polygon);

  virtual void predict(const double /*time_diff_sec*/,
                       const DrivingCorridor& /*driving_corridor*/,
                       const Eigen::Affine3d& /*vehicle_pose*/) {}

  virtual void newMsg(const MsgCollection& /*msg*/, const Eigen::Affine3d& /*vehicle_pose*/) {}


 protected:
  common::EigenAlignedVector<Eigen::Vector3d> base_area;

  void setBaseAreaIfNotEmpty(const std::vector<geometry_msgs::Point>& base_hull_polygon);

 private:
  common::EigenAlignedVector<Eigen::Vector3d> convertHullPolygon(
      const std::vector<geometry_msgs::Point>& base_hull_polygon);
};
}  // namespace environmental_model
#endif  // ROAD_ELEMENT_H
