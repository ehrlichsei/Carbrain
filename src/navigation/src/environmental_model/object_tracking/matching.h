#ifndef MATCHING_H
#define MATCHING_H

#include "../tracking_element.h"
#include "../road_elements/road_element.h"
#include <perception_msgs/Unidentifieds.h>

namespace environmental_model {

class Matching {
 public:
  Matching(const std::shared_ptr<TrackingElementParameter>& tracking_elmement_parameter);
  void assignNewElementsToTracks(std::vector<std::shared_ptr<RoadElement>>& new_road_elements,
                                 std::vector<TrackingElement>& tracking_elements,
                                 const perception_msgs::Unidentifieds& unidentifieds,
                                 const Eigen::Affine3d& vehicle_pose);

 private:
  void assignBestTrackingElement(const std::shared_ptr<RoadElement>& road_element,
                                 std::vector<TrackingElement>& tracking_elements,
                                 std::vector<std::vector<std::shared_ptr<RoadElement>>>& new_elements_per_tracking_element);
  void sortElements(std::vector<std::shared_ptr<RoadElement>>& new_road_elements);
  void mergeTrackingElements(std::vector<TrackingElement>& tracking_elements,
                             const Eigen::Affine3d& vehicle_pose);
  std::vector<Eigen::Vector3d> calcPositionsUnidentifieds(const perception_msgs::Unidentifieds& unidentifieds);

  std::shared_ptr<TrackingElementParameter> tracking_elmement_parameter_;
};

}  // namespace environmental_model

#endif  // MATCHING_H
