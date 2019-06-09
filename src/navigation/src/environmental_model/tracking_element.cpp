#include "tracking_element.h"
#include "road_element_visitors/road_element_visitor.h"

namespace environmental_model {

const ParameterString<std::vector<double>> TrackingElementParameter::PARAM_MEASUREMENT_MATRIX(
    "wonham_measurement_matrix");
const ParameterString<std::vector<double>> TrackingElementParameter::PARAM_MIN_PROB_PUBLISHING(
    "min_prob_publishing");
const ParameterString<std::vector<double>> TrackingElementParameter::PARAM_MIN_PROB_NEW_TRACKING(
    "min_prob_new_tracking");
const ParameterString<double> TrackingElementParameter::PARAM_SCALE_PROBABILITIES(
    "scale_probabilities");
const ParameterString<double> TrackingElementParameter::PARAM_DECAY_MAX_DISTANCE(
    "decay_max_distance");
const ParameterString<double> TrackingElementParameter::PARAM_DECAY_MIN_PROB(
    "decay_min_prob");

double TrackingElement::getMaxMergeDistance(const TrackingElement& e1,
                                            const TrackingElement& e2) {
  if (e1.typeHighestProbability() == e2.typeHighestProbability() &&
      e2.typeHighestProbability() != RoadElementType::Type::Obstacle &&
      e2.typeHighestProbability() != RoadElementType::Type::Unidentified) {
    if (e2.typeHighestProbability() == RoadElementType::Type::Junction) {
      return 0.5;
    }
    return 0.8;
  }
  return 0;
}

TrackingElement::TrackingElement(unsigned int id,
                                 const std::shared_ptr<TrackingElementParameter>& parameter)
    : id(id),
      parameter_(parameter),
      probabilities(ProbabilityVector::Constant(1.0 / ProbabilityVector::RowsAtCompileTime)) {}

void TrackingElement::predict(const double time_diff_sec,
                              const DrivingCorridor& driving_corridor,
                              const Eigen::Affine3d& vehicle_pose) {
  if (road_element) {
    road_element->predict(time_diff_sec, driving_corridor, vehicle_pose);
    base_area = createPolygon(road_element->getBaseArea());
  }
}

bool TrackingElement::stopDecreasingProb(const Eigen::Affine3d& vehicle_pose) const {
  if (number_of_detections_ < 3 &&
      indexMaxProbNotUnidentified(probabilities) !=
          static_cast<int>(RoadElementType::Junction)) {
    return false;
  }

  Eigen::Vector3d centroid;
  const double max_decay_distance = parameter_->max_decay_distance;
  const double min_prob_decay = parameter_->min_prob_decay;
  return calcCentroid(getBaseArea(), centroid) &&
         ((vehicle_pose.inverse() * centroid).x() < max_decay_distance &&
          maxProbNotUnidentified() > min_prob_decay);
}

void TrackingElement::update(const std::vector<std::shared_ptr<RoadElement>>& cluster,
                             const Eigen::Affine3d& vehicle_pose) {
  ProbabilityVector probabilities(ProbabilityVector::Zero());
  for (auto& element : cluster) {
    element->setProbabilityFromLastMsg(probabilities);
  }
  probabilities[RoadElementType::Unidentified] = 0.1;
  if (maxProbNotUnidentified(probabilities) > 0.2) {
    number_of_detections_++;
  } else if (stopDecreasingProb(vehicle_pose)) {
    return;
  }

  fuseObservation(probabilities);
  MsgCollection collection(ros::Time::now());
  for (const auto& element : cluster) {
    element->collectMsgs(collection);
  }
  road_element->newMsg(collection, vehicle_pose);
  base_area = createPolygon(road_element->getBaseArea());
}

void TrackingElement::decay(const double min_dist_unidentified) {
  double decreasing_factor = 1;
  if (min_dist_unidentified >= 0) {
    if (min_dist_unidentified < 0.25) {
      decreasing_factor = 0.5 + 2 * min_dist_unidentified;  // < 1
    }
  }
  ProbabilityVector observation = ProbabilityVector::Constant(1 - decreasing_factor);
  observation(static_cast<int>(RoadElementType::Unidentified)) = 1.0;
  fuseObservation(observation);
}

void TrackingElement::collectMsgs(navigation_msgs::TrackingElements& tracking_elements_msg) {
  navigation_msgs::TrackingElement tracking_element_msg;
  auto hull_polygon = toEigen3d(base_area);
  if (!hull_polygon.empty()) {
    const Eigen::Affine3d pose =
        Eigen::Affine3d::Identity() * Eigen::Translation3d(hull_polygon.front());
    tracking_element_msg.pose = tf2::toMsg(pose);
    for (const auto& point : hull_polygon) {
      tracking_element_msg.base_hull_polygon.push_back(tf2::toMsg(point));
    }
    tracking_element_msg.crosswalk_probability =
        probabilities[static_cast<int>(RoadElementType::Crosswalk)];
    tracking_element_msg.road_closure_probability =
        probabilities[static_cast<int>(RoadElementType::RoadClosure)];
    tracking_element_msg.arrow_marking_probability =
        probabilities[static_cast<int>(RoadElementType::ArrowMarking)];
    tracking_element_msg.speed_limit_probability =
        probabilities[static_cast<int>(RoadElementType::SpeedLimit)];
    tracking_element_msg.junction_probability =
        probabilities[static_cast<int>(RoadElementType::Junction)];
    tracking_element_msg.obstacle_probability =
        probabilities[static_cast<int>(RoadElementType::Obstacle)];
    tracking_element_msg.start_line_probability =
        probabilities[static_cast<int>(RoadElementType::StartLine)];
    tracking_element_msg.unidentified_probability =
        probabilities[static_cast<int>(RoadElementType::Unidentified)];
    tracking_element_msg.id = id;
    tracking_elements_msg.sub_messages.push_back(tracking_element_msg);
  }
}

unsigned int TrackingElement::getId() const { return id; }


void TrackingElement::fuseObservation(const ProbabilityVector& observation_probabilities) {
  const size_t old_most_probable_type = indexMaxProbNotUnidentified(probabilities);
  wonhamFilter(observation_probabilities);
  const size_t new_most_probable_type = indexMaxProbNotUnidentified(probabilities);
  if (new_most_probable_type != old_most_probable_type || !road_element) {
    changeRoadElement(new_most_probable_type);
  }
  probabilities /= probabilities.sum();  // Normalize
}

void TrackingElement::wonhamFilter(const ProbabilityVector& observation_probabilities) {
  // const int i_observation = static_cast<char>(observation);
  // Wonham filter
  ProbabilityVector weigthed_sum_columns = ProbabilityVector::Zero();
  for (size_t i = 0; i < observation_probabilities.SizeAtCompileTime; ++i) {
    weigthed_sum_columns +=
        observation_probabilities[i] * parameter_->measurement_matrix.col(i);
  }
  probabilities = weigthed_sum_columns.cwiseProduct(probabilities);
  probabilities /= probabilities.sum();  // Normalize
}

RoadElementType::Type TrackingElement::typeHighestProbability() const {
  std::ptrdiff_t i;
  probabilities.maxCoeff(&i);
  return static_cast<RoadElementType::Type>(i);
}

void TrackingElement::scaleHighestProb(const double factor) {
  size_t index_to_increase = indexMaxProbNotUnidentified(probabilities);
  probabilities[index_to_increase] *= factor;
  probabilities /= probabilities.sum();
}
bool TrackingElement::hasInvalidRoadElement() const {
  return road_element == nullptr || road_element->getBaseArea().empty();
}

void TrackingElement::changeRoadElement(size_t index_new_element) {
  switch (index_new_element) {
    case RoadElementType::Crosswalk:
      road_element = std::make_unique<Crosswalk>();
      break;
    case RoadElementType::Junction:
      road_element = std::make_unique<Junction>();
      break;
    case RoadElementType::Obstacle:
      road_element = std::make_unique<Obstacle>();
      break;
    case RoadElementType::ArrowMarking:
      road_element = std::make_unique<Arrow>();
      break;
    case RoadElementType::RoadClosure:
      road_element =
          std::make_unique<RoadClosure>(parameter_->getRoadElementParameter());
      break;
    case RoadElementType::SpeedLimit:
      road_element = std::make_unique<SpeedLimit>(parameter_->getRoadElementParameter());
      break;
    case RoadElementType::StartLine:
      road_element = std::make_unique<StartLine>();
      break;
    case RoadElementType::Unidentified:
      road_element = std::make_unique<Unidentified>();
      break;
    default:
      ROS_WARN("not in switch case implemented Road Element type!");
  }
}

Polygon TrackingElement::getBaseArea() const { return base_area; }

void TrackingElement::setBaseArea(const Polygon& value) { base_area = value; }

bool TrackingElement::shouldBeDeleted(const Eigen::Affine3d& vehicle_pose,
                                      const DrivingCorridor& driving_corridor) const {
  if (road_element->getBaseArea().empty()) {
    return true;
  }
  if (!driving_corridor.empty()) {
    auto hull_polygon = toEigen3d(base_area);
    if (hull_polygon.empty()) {
      return true;
    }

    // element is behind last gate ...
    if ((driving_corridor.at(0).getTransformationToGateFrame() * hull_polygon[0]).y() < 0) {
      // ... and element is behind vehicle
      if (std::all_of(hull_polygon.begin(),
                      hull_polygon.end(),
                      [&vehicle_pose](const auto& hull_point) {
                        return (vehicle_pose.inverse() * hull_point).x() < 0;
                      })) {
        return true;
      }
    }
  }
  return maxProbNotUnidentified(probabilities) < probabilities[RoadElementType::Unidentified];
}

void TrackingElement::merge(TrackingElement& other_element, const Eigen::Affine3d& vehicle_pose) {
  const size_t old_most_probable_type = indexMaxProbNotUnidentified(probabilities);
  probabilities = probabilities.cwiseProduct(other_element.probabilities);
  const size_t new_most_probable_type = indexMaxProbNotUnidentified(probabilities);
  if (new_most_probable_type != old_most_probable_type || !road_element) {
    changeRoadElement(new_most_probable_type);
  }
  if (!other_element.road_element || !road_element) {
    return;
  }
  class JunctionGetter : public RoadElementVisitor {
   public:
    JunctionGetter() {}
    void visit(Junction& road_object, TrackingElement& /*tracking_element*/) override {
      junction = &road_object;
    }
    Junction* junction = nullptr;
  } visitorThis, visitorOther;
  road_element->accept(visitorThis, *this);
  other_element.road_element->accept(visitorOther, other_element);
  if (!visitorThis.junction || !visitorOther.junction) {
    return;
  }
  visitorThis.junction->merge(*visitorOther.junction, vehicle_pose);
}

bool TrackingElement::shouldPublishElement() const {
  return maxProbNotUnidentified(probabilities) >
         parameter_->min_prob_publishing[indexMaxProbNotUnidentified(probabilities)];
}

double TrackingElement::maxProbNotUnidentified(ProbabilityVector p) const {
  p[RoadElementType::Unidentified] = 0;
  return p.maxCoeff();
}

double TrackingElement::maxProbNotUnidentified() const {
  return maxProbNotUnidentified(probabilities);
}

size_t TrackingElement::indexMaxProbNotUnidentified(TrackingElement::ProbabilityVector p) const {
  p[RoadElementType::Unidentified] = 0;
  std::ptrdiff_t i;
  p.maxCoeff(&i);
  return i;
}


const std::unique_ptr<RoadElement>& TrackingElement::getRoadElement() const {
  return road_element;
}

void TrackingElementParameter::updateParameter(common::node_base::ParameterInterface* parameters) {
  const auto v_mm = parameters->getParam(PARAM_MEASUREMENT_MATRIX);
  assert(v_mm.size() ==
         TrackingElement::ProbabilityMatrix::RowsAtCompileTime *
             TrackingElement::ProbabilityMatrix::ColsAtCompileTime);
  measurement_matrix = TrackingElement::ProbabilityMatrix(v_mm.data());
  const double scale_probabilities = parameters->getParam(PARAM_SCALE_PROBABILITIES);
  measurement_matrix.unaryExpr(
      [&](const double& a) { return std::pow(a, scale_probabilities); });

  const auto v_min_prob = parameters->getParam(PARAM_MIN_PROB_PUBLISHING);
  assert(v_min_prob.size() == TrackingElement::ProbabilityVector::RowsAtCompileTime);
  min_prob_publishing = TrackingElement::ProbabilityVector(v_min_prob.data());

  const auto v_min_prob_new_tracking = parameters->getParam(PARAM_MIN_PROB_NEW_TRACKING);
  assert(v_min_prob.size() == TrackingElement::ProbabilityVector::RowsAtCompileTime);
  min_prob_new_tracking_ =
      TrackingElement::ProbabilityVector(v_min_prob_new_tracking.data());

  max_decay_distance = parameters->getParam(PARAM_DECAY_MAX_DISTANCE);
  min_prob_decay = parameters->getParam(PARAM_DECAY_MIN_PROB);
}

std::shared_ptr<RoadElementParameter> TrackingElementParameter::getRoadElementParameter() const {
  return road_element_parameter_;
}

TrackingElementParameter::TrackingElementParameter(
    common::node_base::ParameterInterface* parameters,
    const std::shared_ptr<RoadElementParameter>& road_element_parameter)
    : road_element_parameter_(road_element_parameter) {
  parameters->registerParam(PARAM_MEASUREMENT_MATRIX);
  parameters->registerParam(PARAM_MIN_PROB_PUBLISHING);
  parameters->registerParam(PARAM_MIN_PROB_NEW_TRACKING);
  parameters->registerParam(PARAM_SCALE_PROBABILITIES);
  parameters->registerParam(PARAM_DECAY_MAX_DISTANCE);
  parameters->registerParam(PARAM_DECAY_MIN_PROB);
  updateParameter(parameters);
}
}  // namespace environmental_model
