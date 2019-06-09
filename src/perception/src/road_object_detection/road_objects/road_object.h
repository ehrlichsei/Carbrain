#ifndef ROAD_OBJECT_H
#define ROAD_OBJECT_H
#include <common/macros.h>

THIRD_PARTY_HEADERS_BEGIN
#include <geometry_msgs/Pose.h>
#include <ros/time.h>
#include <string>
THIRD_PARTY_HEADERS_END

#include "features.h"

#include "../include/perception_types.h"

// Forward declaration
namespace road_object_detection {
class RoadObjectVisitor;
}  // namespace road_object_detection

namespace road_object_detection {

//! \brief Base class from which all sub RoadObject classes are derived.
class RoadObject {

 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  RoadObject(const ros::Time& timestamp,
             double score,
             VehiclePoints base_hull_polygon_in_vehicle,
             std::string type);
  virtual ~RoadObject() = default;

  RoadObject(RoadObject&&) = default;
  RoadObject(const RoadObject&) = default;
  RoadObject& operator=(RoadObject&&) = default;
  RoadObject& operator=(const RoadObject&) = default;

  //! \brief Accept visit by RoadObjectVisitor;
  //! Basically gives the visitor a pointer to itself.
  virtual void accept(RoadObjectVisitor& visitor) = 0;

  //! \brief Each RoadObject has a unique ID.
  int id = -1;
  //! \brief Timestamp of the corresponding image frame.
  ros::Time timestamp;

  //! \brief Score, i.e. probability, that this RoadObject is indeed
  //! the designated class.
  double score;

  //! \brief Compare score with other road object
  bool operator<(const RoadObject& other_road_object) const;
  //! \brief Compare score with other road object
  bool operator<(double other_score) const;
  //! \brief Compare score with other road object
  bool operator>(const RoadObject& other_road_object) const;
  //! \brief Compare score with other road object
  bool operator>(double other_score) const;

  VehiclePoints base_hull_polygon_in_vehicle;
  WorldPoints base_hull_polygon_in_world;

  /**
   * @brief The type of the road object, e.g "Obstacle", "Junction" etc.
   */
  std::string type;

  /**
   * @brief Gets a short string describing the type, id and score of the object
   * @return a string describing the type, id and score of the object
   */
  virtual const std::string getDescription() const;

  /**
   * @brief Returns wether the road object should be tracked or not
   * @return True is returned if the road object should be tracked, otherwise
   * false is returned
   */
  virtual bool shouldBeTracked() const;
};

typedef std::unique_ptr<RoadObject> RoadObjectPtr;
typedef std::vector<std::unique_ptr<RoadObject>> RoadObjects;

inline std::ostream& operator<<(std::ostream& stream, const RoadObjectPtr& o) {
  return stream << o->getDescription();
}

inline bool operator>(const RoadObjectPtr& a, const RoadObjectPtr& b) {
  return *a > *b;
}

inline bool operator<(const RoadObjectPtr& a, const RoadObjectPtr& b) {
  return *a < *b;
}

class Unidentified : public RoadObject {
 public:
  /**
   * @brief The position of the road object in vehicle coordinates
   */
  VehiclePose pose_in_vehicle;
  /**
   * @brief The pose of the road object in world coordinates
   */
  WorldPose pose_in_world;

  Unidentified(const ros::Time& timestamp,
               double score,
               const VehiclePose& pose_in_vehicle,
               VehiclePoints base_hull_polygon_in_vehicle);

  void accept(RoadObjectVisitor& visitor) override;
  bool shouldBeTracked() const override;
};

//--------------------------------------------------------------
class Obstacle : public RoadObject {
 public:
  // Specifies whether a vertex (specified in the base_hull_polygon) has
  // actually been DETECTED (used by the classidier for classification),
  // estimated after classification (ESTIMATED) or only guessed (NSECURE)
  enum DetectionState : int8_t {
    VERTEX_DETECTED = 1,
    VERTEX_ESTIMATED = 2,
    VERTEX_INSECURE = 3
  };
  std::vector<DetectionState> vertices_detection_state;

  Obstacle(const ros::Time& timestamp,
           double score,
           VehiclePoints base_hull_polygon_in_vehicle,
           const std::vector<DetectionState>& vertices_detection_state);

  // constructor used by old obstale_classifier
  Obstacle(const ros::Time& timestamp, double score, VehiclePoints base_hull_polygon_in_vehicle);

  void accept(RoadObjectVisitor& visitor) override;
};

//--------------------------------------------------------------
class Junction : public RoadObject {
 public:
  enum JunctionType {
    stopline_left = 0,
    stopline_right = 1,
    givewayline_left = 2,
    givewayline_right = 3
  };

  /**
   * @brief The position of the road object in vehicle coordinates
   */
  VehiclePose pose_in_vehicle;
  /**
   * @brief The pose of the road object in world coordinates
   */
  WorldPose pose_in_world;

  JunctionType junction_type;

  Junction(enum JunctionType,
           const ros::Time& timestamp,
           double score,
           const VehiclePose& pose_in_vehicle,
           VehiclePoints base_hull_polygon_in_vehicle);

  void accept(RoadObjectVisitor& visitor) override;
};

//--------------------------------------------------------------
class Crosswalk : public RoadObject {
 public:
  /**
   * @brief The position of the road object in vehicle coordinates
   */
  VehiclePose pose_in_vehicle;
  /**
   * @brief The pose of the road object in world coordinates
   */
  WorldPose pose_in_world;

  Crosswalk(const ros::Time& timestamp,
            double score,
            const VehiclePose& pose_in_vehicle,
            VehiclePoints base_hull_polygon_in_vehicle);

  void accept(RoadObjectVisitor& visitor) override;
};

//--------------------------------------------------------------
class RoadClosure : public RoadObject {
 public:
  /**
   * @brief The position of the road object in vehicle coordinates
   */
  VehiclePose pose_in_vehicle;
  /**
   * @brief The pose of the road object in world coordinates
   */
  WorldPose pose_in_world;

  RoadClosure(const ros::Time& timestamp,
              double score,
              const VehiclePose& pose_in_vehicle,
              VehiclePoints hull_polygon_vehicle);

  void accept(RoadObjectVisitor& visitor) override;
};

//--------------------------------------------------------------
class SpeedLimitMarking : public RoadObject {
 public:
  /**
   * @brief The position of the road object in vehicle coordinates
   */
  VehiclePose pose_in_vehicle;
  /**
   * @brief The pose of the road object in world coordinates
   */
  WorldPose pose_in_world;

  int speed_limit;
  bool limit_relieved;

  SpeedLimitMarking(const ros::Time& timestamp,
                    double score,
                    const VehiclePose& pose_in_vehicle,
                    int speed_limit,
                    VehiclePoints base_hull_polygon_in_vehicle,
                    bool limit_relieved);

  void accept(RoadObjectVisitor& visitor) override;

  // RoadObject interface
 public:
  const std::string getDescription() const override;
};

//--------------------------------------------------------------
class StartLine : public RoadObject {
 public:
  /**
   * @brief The position of the road object in vehicle coordinates
   */
  VehiclePose pose_in_vehicle;
  /**
   * @brief The pose of the road object in world coordinates
   */
  WorldPose pose_in_world;

  StartLine(const ros::Time& timestamp,
            double score,
            const VehiclePose& pose_in_vehicle,
            VehiclePoints base_hull_polygon_in_vehicle);

  void accept(RoadObjectVisitor& visitor) override;
};

class ArrowMarking : public RoadObject {
 public:
  /**
   * @brief The position of the road object in vehicle coordinates
   */
  VehiclePose pose_in_vehicle;
  /**
   * @brief The pose of the road object in world coordinates
   */
  WorldPose pose_in_world;

  enum class Type : int { TURN_LEFT = 0, TURN_RIGHT = 1 };

  Type arrow_type;

  ArrowMarking(const ros::Time& timestamp,
               double score,
               const VehiclePose& pose_in_vehicle,
               VehiclePoints base_hull_polygon_in_vehicle,
               Type arrow_type);

  // RoadObject interface
  void accept(RoadObjectVisitor& visitor) override;

  // RoadObject interface
 public:
  const std::string getDescription() const override;
};


class Pedestrian : public RoadObject {
 public:
  /**
   * @brief The position of the road object in vehicle coordinates
   */
  VehiclePose pose_in_vehicle;
  /**
   * @brief The pose of the road object in world coordinates
   */
  WorldPose pose_in_world;

  Pedestrian(const ros::Time& timestamp,
             double score,
             const VehiclePose& pose_in_vehicle,
             VehiclePoints base_hull_polygon_in_vehicle);
  virtual ~Pedestrian() override = default;

  // RoadObject interface
  void accept(RoadObjectVisitor& visitor) override;
};

class NoPassingZone : public RoadObject {
 public:
  VehiclePose pose_in_vehicle;
  /**
   * @brief The pose of the road object in world coordinates
   */
  WorldPose pose_in_world;


  VehiclePoint start_point_vehicle;
  VehiclePoint end_point_vehicle;

  WorldPoint start_point_world;
  WorldPoint end_point_world;


  NoPassingZone(const ros::Time& timestamp,
                double score,
                const VehiclePose& pose_in_vehicle,
                VehiclePoints base_hull_polygon_in_vehicle,
                const VehiclePoint& start,
                const VehiclePoint& end);


  // RoadObject interface
  void accept(RoadObjectVisitor& visitor) override;
};

}  // namespace road_object_detection

#endif  // ROAD_OBJECT_H
