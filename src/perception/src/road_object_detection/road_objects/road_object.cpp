#include "road_object.h"
#include "../road_object_visitors/road_object_visitor.h"

THIRD_PARTY_HEADERS_BEGIN
#include <ros/console.h>
THIRD_PARTY_HEADERS_END

namespace road_object_detection {

bool RoadObject::operator<(const RoadObject &other_road_object) const {
  if (score == other_road_object.score) {
    ROS_WARN_THROTTLE(4, "equal score of two classifiers!");
  }

  return (score < other_road_object.score);
}
bool RoadObject::operator<(double other_score) const {
  return (score < other_score);
}

bool RoadObject::operator>(const RoadObject &other_road_object) const {
  return (score > other_road_object.score);
}

bool RoadObject::operator>(double other_score) const {
  return (score > other_score);
}

RoadObject::RoadObject(const ros::Time &timestamp,
                       double score,
                       VehiclePoints base_hull_polygon_in_vehicle,
                       std::string type)
    : timestamp(timestamp),
      score(score),
      base_hull_polygon_in_vehicle(std::move(base_hull_polygon_in_vehicle)),
      type(std::move(type)) {
  assert(score >= 0 && score <= 1);
}

const std::string RoadObject::getDescription() const {
  std::stringstream stream;
  stream << type << " [ID: " << id << ", Score: " << std::setprecision(5) << score << "]";
  return stream.str();
}

bool RoadObject::shouldBeTracked() const { return true; }

//--------------------------------------------------------------
Unidentified::Unidentified(const ros::Time &timestamp,
                           double score,
                           const VehiclePose &pose_in_vehicle,
                           VehiclePoints base_hull_polygon_in_vehicle)
    : RoadObject(timestamp,
                 score,
                 std::move(base_hull_polygon_in_vehicle),
                 "Unidentified"),
      pose_in_vehicle(pose_in_vehicle) {}

void Unidentified::accept(RoadObjectVisitor &visitor) { visitor.visit(*this); }

bool Unidentified::shouldBeTracked() const {
  // We dont want to track unidentified road objects
  return false;
}

//--------------------------------------------------------------
Obstacle::Obstacle(const ros::Time &timestamp,
                   double score,
                   VehiclePoints base_hull_polygon_in_vehicle,
                   const std::vector<DetectionState> &vertices_detected)
    : RoadObject(timestamp,
                 score,
                 std::move(base_hull_polygon_in_vehicle),
                 "Obstacle"),
      vertices_detection_state(vertices_detected) {}

Obstacle::Obstacle(const ros::Time &timestamp, double score, VehiclePoints base_hull_polygon_in_vehicle)
    : RoadObject(timestamp,
                 score,
                 std::move(base_hull_polygon_in_vehicle),
                 "Obstacle") {}


void Obstacle::accept(RoadObjectVisitor &visitor) { visitor.visit(*this); }

//--------------------------------------------------------------
Junction::Junction(enum JunctionType junction_type,
                   const ros::Time &timestamp,
                   double score,
                   const VehiclePose &pose_in_vehicle,
                   VehiclePoints base_hull_polygon_in_vehicle)
    : RoadObject(timestamp,
                 score,
                 std::move(base_hull_polygon_in_vehicle),
                 "Junction"),
      pose_in_vehicle(pose_in_vehicle),
      junction_type(junction_type) {}

void Junction::accept(RoadObjectVisitor &visitor) { visitor.visit(*this); }

//--------------------------------------------------------------
Crosswalk::Crosswalk(const ros::Time &timestamp,
                     double score,
                     const VehiclePose &pose_in_vehicle,

                     VehiclePoints base_hull_polygon_in_vehicle)
    : RoadObject(timestamp,
                 score,
                 std::move(base_hull_polygon_in_vehicle),
                 "Crosswalk"),
      pose_in_vehicle(pose_in_vehicle) {}

void Crosswalk::accept(RoadObjectVisitor &visitor) { visitor.visit(*this); }

//--------------------------------------------------------------
RoadClosure::RoadClosure(const ros::Time &timestamp,
                         double score,
                         const VehiclePose &pose_in_vehicle,
                         VehiclePoints hull_polygon_vehicle)
    : RoadObject(
          timestamp, score, std::move(hull_polygon_vehicle), "Road Closure"),
      pose_in_vehicle(pose_in_vehicle) {}

void RoadClosure::accept(RoadObjectVisitor &visitor) { visitor.visit(*this); }

//--------------------------------------------------------------
SpeedLimitMarking::SpeedLimitMarking(const ros::Time &timestamp,
                                     double score,
                                     const VehiclePose &pose_in_vehicle,
                                     int speed_limit,
                                     VehiclePoints base_hull_polygon_in_vehicle,
                                     bool limit_relieved)
    : RoadObject(timestamp,
                 score,
                 std::move(base_hull_polygon_in_vehicle),
                 "Speed Limit Marking"),
      pose_in_vehicle(pose_in_vehicle),
      speed_limit(speed_limit),
      limit_relieved(limit_relieved) {}

void SpeedLimitMarking::accept(RoadObjectVisitor &visitor) {
  visitor.visit(*this);
}
const std::string SpeedLimitMarking::getDescription() const {
  std::stringstream stream;
  stream << "Speed" << std::to_string(speed_limit);

  if (limit_relieved) {
    stream << "X";
  }

  stream << " [ID: " << id << ", Score: " << std::setprecision(5) << score << "]";
  return stream.str();
}

//--------------------------------------------------------------
ArrowMarking::ArrowMarking(const ros::Time &timestamp,
                           double score,
                           const VehiclePose &pose_in_vehicle,
                           VehiclePoints base_hull_polygon_in_vehicle,
                           Type arrow_type)
    : RoadObject(timestamp,
                 score,
                 std::move(base_hull_polygon_in_vehicle),
                 "Arrow Marking"),
      pose_in_vehicle(pose_in_vehicle),
      arrow_type(arrow_type) {}

void ArrowMarking::accept(RoadObjectVisitor &visitor) { visitor.visit(*this); }

const std::string ArrowMarking::getDescription() const {
  std::stringstream stream;
  stream << "Arrow";

  switch (arrow_type) {
    case (Type::TURN_LEFT):
      stream << " left";
      break;
    case (Type::TURN_RIGHT):
      stream << " right";
      break;
  }

  stream << " [ID: " << id << ", Score: " << std::setprecision(5) << score << "]";
  return stream.str();
}

//--------------------------------------------------------------
StartLine::StartLine(const ros::Time &timestamp,
                     double score,
                     const VehiclePose &pose_in_vehicle,
                     VehiclePoints base_hull_polygon_in_vehicle)
    : RoadObject(timestamp,
                 score,
                 std::move(base_hull_polygon_in_vehicle),
                 "Start Line"),
      pose_in_vehicle(pose_in_vehicle) {}

void StartLine::accept(RoadObjectVisitor &visitor) { visitor.visit(*this); }

Pedestrian::Pedestrian(const ros::Time &timestamp,
                       double score,
                       const VehiclePose &pose_in_vehicle,
                       VehiclePoints base_hull_polygon_in_vehicle)
    : RoadObject(timestamp,
                 score,
                 std::move(base_hull_polygon_in_vehicle),
                 "Pedestrian"),
      pose_in_vehicle(pose_in_vehicle) {}

void Pedestrian::accept(RoadObjectVisitor &visitor) { visitor.visit(*this); }

NoPassingZone::NoPassingZone(const ros::Time &timestamp,
                             double score,
                             const VehiclePose &pose_in_vehicle,
                             VehiclePoints base_hull_polygon_in_vehicle,
                             const VehiclePoint &start,
                             const VehiclePoint &end)
    : RoadObject(timestamp,
                 score,
                 std::move(base_hull_polygon_in_vehicle),
                 "NoPassingZone"),
      pose_in_vehicle(pose_in_vehicle),
      start_point_vehicle(start),
      end_point_vehicle(end) {}

void NoPassingZone::accept(RoadObjectVisitor &visitor) { visitor.visit(*this); }

//--------------------------------------------------------------

}  // namespace road_object_detection
