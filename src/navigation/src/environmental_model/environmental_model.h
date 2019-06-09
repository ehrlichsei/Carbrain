#ifndef ENVIRONMENTAL_MODEL_H
#define ENVIRONMENTAL_MODEL_H

#include <common/macros.h>

THIRD_PARTY_HEADERS_BEGIN
#include <unordered_map>
#include <unordered_set>
#include <navigation_msgs/TrackingElements.h>
#include <perception_msgs/Unidentifieds.h>
#include <sensor_msgs/Range.h>
THIRD_PARTY_HEADERS_END

#include "common/parameter_interface.h"
#include "road_elements/road_element.h"
#include "object_tracking/matching.h"
#include "tracking_element.h"
#include "navigation/driving_corridor.h"
#include "navigation/no_passing_zone.h"
#include "msg_collection.h"
#include "road_element_visitors/road_element_visitor.h"
#include "road_element_visitors/look_at_visitor.h"

namespace environmental_model {

/*!
 * \brief Description
 */
class EnvironmentalModel {
 public:
  /*!
  * \brief EnvironmentalModel is the consstructor. A ros independent
  * functionality containing
  * class needs a pointer to a ParameterInterface (in fact a ParameterHandler)
  * to get access to parameters.
  * \param parameters the ParameterInterface
  */
  EnvironmentalModel(ParameterInterface *parameters);
  void predict(const double time_diff_sec, const Eigen::Affine3d &vehicle_pose);
  void reset();
  void updateParameter();
  void update(std::vector<std::shared_ptr<RoadElement>> &new_road_elements,
              const perception_msgs::Unidentifieds &unidentifieds,
              const Eigen::Affine3d &vehicle_pose);
  void collectMsgs(navigation_msgs::TrackingElements &tracking_elements_msg);
  NoPassingZones &getNoPassingZones();

  void updateFullCorridor(const DrivingCorridor &full_corridor);

  void receivedTOFMeasurement(const TOFMeasurement &measurement);
  void receivedTOFMeasurementAhead(const TOFMeasurementAhead &measurement);
  template <class Visitor>
  void visitElements(Visitor &visitor) {
    visitor_helper::visitElements<Visitor>(visitor, tracking_elements);
  }
  void deleteAllObstaclesInRoi(const perception_msgs::LookAtRegions &obstacle_look_at_rois);

 private:
  /*!
  * \brief parameters_ptr_ is needed for parameter access
  */
  ParameterInterface *parameters_ptr_;
  std::shared_ptr<RoadElementParameter> road_element_parameter_;
  std::shared_ptr<TrackingElementParameter> tracking_element_parameter_;
  std::shared_ptr<NoPassingZonesParameter> no_passing_zones_parameter_;
  std::vector<TrackingElement> tracking_elements;
  NoPassingZones no_passing_zones;
  DrivingCorridor full_corridor_;
  Matching matching_;
  std::deque<std::pair<Eigen::Vector3d, ros::Time>> last_vehicle_positions_;
};

}  // namespace environmental_model

#endif  // ENVIRONMENTAL_MODEL_H
