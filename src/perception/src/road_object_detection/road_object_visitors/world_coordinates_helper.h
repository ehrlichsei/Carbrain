#ifndef WORLD_COORDINATES_HELPER_H
#define WORLD_COORDINATES_HELPER_H
#include <common/macros.h>

THIRD_PARTY_HEADERS_BEGIN
#include <tf2_ros/transform_listener.h>
THIRD_PARTY_HEADERS_END

#include "../../utils/tf_helper.h"
#include "road_object_visitor.h"

namespace road_object_detection {

class WorldCoordinatesHelper : public RoadObjectVisitor {
 public:
  WorldCoordinatesHelper(const tf_helper::TFHelperInterface<double> *const tf_helper)
      : tf_helper_(tf_helper) {}

  /**
   * @brief Transforms each VehiclePoint(s) member of each RoadObject in
   *road_objects from vehicle-coordinates
   *				to world-coordinates and stores the results in
   *the
   *corresponding WorldPoint(s) member.
   * @param road_objects the RoadObjects
   *the world frame
   */
  void calcWorldCoordinates(RoadObjects &road_objects);


  void visit(Unidentified &road_object) override;
  void visit(Obstacle &road_object) override;
  void visit(Junction &road_object) override;
  void visit(Crosswalk &road_object) override;
  void visit(RoadClosure &road_object) override;
  void visit(SpeedLimitMarking &road_object) override;
  void visit(ArrowMarking &road_object) override;
  void visit(StartLine &road_object) override;
  void visit(Pedestrian &road_object) override;
  void visit(NoPassingZone &road_object) override;

  void transformVehicleToWorldPoints(const VehiclePoints &vehicle_points,
                                     WorldPoints &world_points) const;
  void transformVehicleToWorldPose(const VehiclePose &vehicle_pose, WorldPose &world_pose) const;

  const Eigen::Affine3d vehicleToWorldTransform() const {
    return this->tf_helper_->getTransform();
  }

  const tf_helper::TFHelperInterface<double> *getHelper() const {
    return this->tf_helper_;
  }

 private:
  const tf_helper::TFHelperInterface<double> *const tf_helper_;

  std::function<WorldPoint(const VehiclePoint &vehicle_point)> transformVehicleToWorld =
      [this](const VehiclePoint &vehicle_point) {
        return tf_helper_->getTransform() * vehicle_point;
      };

  std::function<WorldPose(const VehiclePose &vehicle_pose)> transformVehicleToWorldP =
      [this](const VehiclePose &vehicle_pose) {
        return tf_helper_->getTransform() * vehicle_pose;
      };
};

}  // namespace road_object_detection

#endif  // WORLD_COORDINATES_HELPER_H
