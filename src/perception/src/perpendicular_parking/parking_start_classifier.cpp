#include "parking_start_classifier.h"

THIRD_PARTY_HEADERS_BEGIN
#include <boost/range/adaptor/filtered.hpp>
#include <boost/range/adaptor/transformed.hpp>
#include "boost/range/algorithm.hpp"
THIRD_PARTY_HEADERS_END

#include "common/angle_conversions.h"
#include "common/best_score.h"
#include "common/eigen_functors.h"
#include "common/make_vector.h"
#include "common/pca_eigen.h"
#include "common/polynomialfit.h"

namespace perpendicular_parking {

using boost::adaptors::filtered;
using boost::adaptors::transformed;

const std::string ParkingStartClassifier::NAMESPACE("parking_start_classifier");

const ParameterString<double> ParkingStartClassifier::HORIZON(NAMESPACE +
                                                              "/horizon");
const ParameterString<double> ParkingStartClassifier::MAX_DISTANCE_END_OF_POINTS(
    NAMESPACE + "/max_distance_end_of_points");
const ParameterString<double> ParkingStartClassifier::MAX_ANGLE_DELTA(
    NAMESPACE + "/max_angle_delta");
const ParameterString<double> ParkingStartClassifier::SECOND_PCA_THLD(
    NAMESPACE + "/second_pca_thld");
const ParameterString<double> ParkingStartClassifier::MAX_DIST_TO_BE_CLOSE(
    NAMESPACE + "/max_dist_to_be_close");
const ParameterString<int> ParkingStartClassifier::MIN_NR_CLOSE_PTS(
    NAMESPACE + "/min_nr_close_points");
const ParameterString<double> ParkingStartClassifier::REQUIRED_CERTAINTY(
    NAMESPACE + "/required_certainty");
const ParameterString<double> ParkingStartClassifier::RANSAC_MIN_MODEL_DIST(
    NAMESPACE + "/ransac/minimal_model_distance");
const ParameterString<int> ParkingStartClassifier::RANSAC_MIN_SET_SIZE(
    NAMESPACE + "/ransac/minimal_consensus_set_size");
const ParameterString<int> ParkingStartClassifier::RANSAC_NR_EXPECTED_LINES(
    NAMESPACE + "/ransac/expected_nr_of_lines");
const ParameterString<double> ParkingStartClassifier::MINIMUM_STARTLINE_ANGLE(
    NAMESPACE + "/minimum_startline_angle");
const ParameterString<double> ParkingStartClassifier::MAXIMUM_STARTLINE_ANGLE(
    NAMESPACE + "/maximum_startline_angle");

ParkingStartClassifier::ParkingStartClassifier(
    ParameterInterface *const parameters_ptr,
    const common::CameraTransformation *const camera_transform,
    const tf_helper::TFHelperInterface<double> *const world_coordinates_helper)
    : parameters_ptr_(parameters_ptr),
      cam_transform_(camera_transform),
      world_coordinates_helper_(world_coordinates_helper),
      db_clusterer(parameters_ptr) {
  parameters_ptr->registerParam(HORIZON);
  parameters_ptr->registerParam(MAX_DISTANCE_END_OF_POINTS);
  parameters_ptr->registerParam(MAX_ANGLE_DELTA);
  parameters_ptr->registerParam(SECOND_PCA_THLD);
  parameters_ptr->registerParam(MAX_DIST_TO_BE_CLOSE);
  parameters_ptr->registerParam(MIN_NR_CLOSE_PTS);
  parameters_ptr->registerParam(REQUIRED_CERTAINTY);
  parameters_ptr->registerParam(RANSAC_MIN_MODEL_DIST);
  parameters_ptr->registerParam(RANSAC_MIN_SET_SIZE);
  parameters_ptr->registerParam(RANSAC_NR_EXPECTED_LINES);
  parameters_ptr->registerParam(MINIMUM_STARTLINE_ANGLE);
  parameters_ptr->registerParam(MAXIMUM_STARTLINE_ANGLE);
}

Type ParkingStartClassifier::classify(FeaturePointCluster &cluster,
                                      const common::DynamicPolynomial &left_lane_polynom,
                                      const ros::Time &timestamp) {
  if (cluster.feature_points_vehicle.empty()) {
    return Type::UNKNOWN;
  }

  const auto point_clusters = db_clusterer.cluster(cluster);
  for (const auto &c : point_clusters) {
    updateLines(c, left_lane_polynom, timestamp);
  }

  if (startlines_.empty()) {
    return Type::UNKNOWN;
  }

  const auto min_certainity = parameters_ptr_->getParam(REQUIRED_CERTAINTY);
  const auto start_line =
      common::max_score(startlines_, [](const auto &sl) { return sl.belief(); });
  start_line_pose = start_line->pose();

  return start_line->belief() >= min_certainity ? Type::START : Type::UNKNOWN;
}

WorldPose ParkingStartClassifier::startLinePose() const {
  return start_line_pose;
}



void ParkingStartClassifier::reset() {
  id_count = 0;
  startlines_.clear();
}

void ParkingStartClassifier::updateLines(const FeaturePointCluster &cluster,
                                         const common::DynamicPolynomial &left_lane_polynom,
                                         const ros::Time &stamp) {
  // coefficients of polynom
  const auto ll_c = left_lane_polynom.getCoefficientsList()[0];
  const auto ll_m = left_lane_polynom.getCoefficientsList()[1];

  // extraction of state (intersection of two lines for coordinates)
  const auto toVehicleState = [&ll_m, &ll_c](const auto &line) {
    const auto& line_coeffs = line.getCoefficientsList();
    const auto intersection_x = (line_coeffs[0] - ll_c) / (ll_m - line_coeffs[1]);
    return Eigen::Vector4d{intersection_x,
                           line.evaluate(intersection_x),
                           std::atan(line_coeffs[1]),
                           std::atan(line_coeffs[1]) - std::atan(ll_m)};
  };
  // check if angle is in valid range
  const auto MIN_STARTLINE_ANGLE = parameters_ptr_->getParam(MINIMUM_STARTLINE_ANGLE);
  const auto MAX_STARTLINE_ANGLE = parameters_ptr_->getParam(MAXIMUM_STARTLINE_ANGLE);

  const auto isValid = [&MIN_STARTLINE_ANGLE, &MAX_STARTLINE_ANGLE](const auto &state) {
    ROS_DEBUG("parkingline detection, difference angle is %f", state.w());
    return state.w() > MIN_STARTLINE_ANGLE && state.w() < MAX_STARTLINE_ANGLE;
  };

  const std::size_t nr_lines =
      static_cast<std::size_t>(parameters_ptr_->getParam(RANSAC_NR_EXPECTED_LINES));
  // get lines in cluster
  const auto lines = fitLinesR(nr_lines, cluster.feature_points_vehicle);

  // prepare transformation of valid states to world_frame
  const auto toState = [](const auto &vs) {
    return State{vs.template head<3>()};
  };
  const auto toStartLine = [this, &stamp](const auto &s) {
    return Startline{stamp, s, 0UL, parameters_ptr_};
  };

  const Eigen::Affine3d world_T_vehicle = world_coordinates_helper_->getTransform();

  const auto translationToWorldFrame = [&world_T_vehicle](const auto &s) {
    const Eigen::Vector2d world_translation = to2D(world_T_vehicle) * to2D(s);
    return State{world_translation.x(),
                 world_translation.y(),
                 common::toYaw(world_T_vehicle.linear() *
                               Eigen::AngleAxisd(s.z(), Eigen::Vector3d::UnitZ()))};
  };

  const auto actual_startlines = common::make_vector(
      lines | transformed(toVehicleState) | filtered(isValid) | transformed(toState) |
      transformed(translationToWorldFrame) | transformed(toStartLine));

  for (const auto &startline : actual_startlines) {
    // search for lines that have been already seen
    const auto matched_line = boost::find(startlines_, startline);
    if (matched_line == startlines_.end()) {
      startlines_.emplace_back(stamp, startline.state(), generateId(), parameters_ptr_);
    } else {
      matched_line->update(stamp, startline.state());
    }
  }
}

std::size_t ParkingStartClassifier::generateId() {
  id_count++;
  return id_count;
}

inline auto modelSupport(const Line &model, const double min_dist) {
  return [&model, min_dist](const auto &p) {
    return std::fabs(model.distance(to2D(p))) < min_dist;
  };
};

std::vector<common::DynamicPolynomial> ParkingStartClassifier::fitLinesR(
    const std::size_t nr_lines, const VehiclePoints &points) const {
  std::vector<common::DynamicPolynomial> lines;
  lines.reserve(nr_lines);
  // use observations
  auto detected_points = points;
  const std::size_t max_it = detected_points.size();
  const double min_dist = parameters_ptr_->getParam(RANSAC_MIN_MODEL_DIST);
  const std::size_t min_set_size =
      static_cast<std::size_t>(parameters_ptr_->getParam(RANSAC_MIN_SET_SIZE));
  VehiclePoints data_next_it;

  // loop in order to detect the specified number of lines
  for (std::size_t i = 0; i < nr_lines; i++) {

    std::size_t best_set_size = 0;
    boost::optional<common::DynamicPolynomial> best_model = boost::none;

    // ransac loop
    for (std::size_t it = 0; it < max_it; it++) {
      boost::random_shuffle(detected_points);
      const Line model =
          Line::Through(to2D(detected_points.front()), to2D(detected_points.back()));
      const auto divider =
          boost::partition(detected_points, modelSupport(model, min_dist));
      const VehiclePoints consensus_set(detected_points.begin(), divider);
      const auto model_size = consensus_set.size();
      if (model_size > best_set_size && model_size > min_set_size) {
        //  least squares fit for best model
        best_model = common::fitToPoints(consensus_set, 1);
        best_set_size = model_size;
        data_next_it.clear();
        data_next_it.insert(data_next_it.begin(), divider, detected_points.end());
      }
    }

    // if no model is found, break
    if (!best_model) {
      break;
    }

    // push back best line and remove points supporting the model
    lines.push_back(best_model.get());
    detected_points = data_next_it;
    if (boost::size(detected_points) < min_set_size) {
      break;
    }
  }

  return lines;
}

}  // namespace perpendicular_parking
