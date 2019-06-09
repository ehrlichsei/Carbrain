#ifndef TRACKING_H
#define TRACKING_H

#include <common/macros.h>

THIRD_PARTY_HEADERS_BEGIN
#include <memory>
#include <Eigen/Dense>
#include <ros/console.h>
#include "navigation_msgs/TrackingElement.h"
#include "navigation_msgs/TrackingElements.h"
THIRD_PARTY_HEADERS_END

#include "common/parameter_interface.h"
#include "road_elements/arrow.h"
#include "road_elements/crosswalk.h"
#include "road_elements/junction.h"
#include "road_elements/obstacle.h"
#include "road_elements/road_closure.h"
#include "road_elements/road_element.h"
#include "road_elements/speed_limit.h"
#include "road_elements/start_line.h"
#include "road_elements/unidentified.h"

#include "navigation/driving_corridor.h"
#include "msg_collection.h"
#include "object_tracking/polygon_algorithm.h"

namespace environmental_model {

class Crosswalk;
class Junction;
class RoadClosure;
class SpeedLimit;
class StartLine;
class Arrow;
class Obstacle;

namespace RoadElementType {
enum Type : char {
  Crosswalk,
  Junction,
  ArrowMarking,
  Obstacle,
  RoadClosure,
  SpeedLimit,
  StartLine,
  Unidentified
};
}

struct TrackingElementParameter;
class TrackingElement {
 public:
  static constexpr short NumElements = 8;
  static double getMaxMergeDistance(const TrackingElement& e1, const TrackingElement& e2);
  using ProbabilityVector = Eigen::Matrix<double, NumElements, 1>;
  using ProbabilityMatrix = Eigen::Matrix<double, NumElements, NumElements>;

  TrackingElement(unsigned int id, const std::shared_ptr<TrackingElementParameter>& parameter);
  void predict(const double time_diff_sec,
               const DrivingCorridor& driving_corridor,
               const Eigen::Affine3d& vehicle_pose);
  void update(const std::vector<std::shared_ptr<RoadElement>>& cluster,
              const Eigen::Affine3d& vehicle_pose);
  void decay(const double min_dist_unidentified);
  void collectMsgs(navigation_msgs::TrackingElements& tracking_elements_msg);
  unsigned int getId() const;

  static void registerParameters(ParameterInterface* parameters);

  Polygon getBaseArea() const;
  void setBaseArea(const Polygon& value);
  bool shouldBeDeleted(const Eigen::Affine3d& vehicle_pose,
                       const DrivingCorridor& driving_corridor) const;
  void merge(TrackingElement& other_element, const Eigen::Affine3d& vehicle_pose);
  double maxProbNotUnidentified(ProbabilityVector p) const;
  double maxProbNotUnidentified() const;

  const std::unique_ptr<RoadElement>& getRoadElement() const;
  bool shouldPublishElement() const;
  RoadElementType::Type typeHighestProbability() const;
  void scaleHighestProb(const double factor);
  bool hasInvalidRoadElement() const;
  bool stopDecreasingProb(const Eigen::Affine3d &vehicle_pose) const;

 private:
  size_t indexMaxProbNotUnidentified(ProbabilityVector p) const;

  unsigned int id;
  std::shared_ptr<TrackingElementParameter> parameter_;
  std::unique_ptr<RoadElement> road_element;
  ProbabilityVector probabilities;
  void fuseObservation(const ProbabilityVector& observation_probabilities);
  void wonhamFilter(const ProbabilityVector& observation_probabilities);
  void changeRoadElement(size_t index_new_element);
  Polygon base_area;
  unsigned int number_of_detections_ = 0;
};

struct TrackingElementParameter {
  TrackingElementParameter(ParameterInterface* parameters,
                           const std::shared_ptr<RoadElementParameter>& road_element_parameter);
  void updateParameter(ParameterInterface* parameters);
  TrackingElement::ProbabilityMatrix measurement_matrix =
      TrackingElement::ProbabilityMatrix::Zero();
  TrackingElement::ProbabilityVector min_prob_publishing =
      TrackingElement::ProbabilityVector::Zero();
  TrackingElement::ProbabilityVector min_prob_new_tracking_ =
      TrackingElement::ProbabilityVector::Zero();
  double scale_probabilities = 0.0;
  double min_prob_decay = 0.0;
  double max_decay_distance = 0.0;
  std::shared_ptr<RoadElementParameter> getRoadElementParameter() const;

 private:
  static const ParameterString<std::vector<double>> PARAM_MEASUREMENT_MATRIX;
  static const ParameterString<std::vector<double>> PARAM_MIN_PROB_PUBLISHING;
  static const ParameterString<std::vector<double>> PARAM_MIN_PROB_NEW_TRACKING;
  static const ParameterString<double> PARAM_SCALE_PROBABILITIES;
  static const ParameterString<double> PARAM_DECAY_MIN_PROB;
  static const ParameterString<double> PARAM_DECAY_MAX_DISTANCE;
  std::shared_ptr<RoadElementParameter> road_element_parameter_;
};

}  // namespace environmental_model

#endif  // TRACKING_H
