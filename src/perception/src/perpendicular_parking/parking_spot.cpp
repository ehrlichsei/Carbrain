#include "parking_spot.h"

THIRD_PARTY_HEADERS_BEGIN
#include <tf2_eigen/tf2_eigen.h>
#include <iterator>
#include <vector>

#include <ros/console.h>

#include <boost/algorithm/cxx11/any_of.hpp>
#include <boost/fusion/iterator/next.hpp>
#include <boost/fusion/iterator/prior.hpp>
#include <boost/range/adaptor/map.hpp>
#include <boost/range/adaptor/reversed.hpp>
#include <boost/range/adaptor/transformed.hpp>
#include <boost/range/algorithm/find.hpp>
#include <boost/range/algorithm/partition.hpp>
#include <boost/range/algorithm/random_shuffle.hpp>
#include <boost/range/algorithm_ext/push_back.hpp>
#include <boost/range/size.hpp>
THIRD_PARTY_HEADERS_END

#include <common/adaptors.h>
#include <common/basic_statistics.h>
#include <common/basic_statistics_eigen.h>
#include <common/best_score.h>
#include <common/contains.h>
#include <common/eigen_functors.h>
#include <common/eigen_utils.h>
#include <common/make_vector.h>
#include <common/pca_eigen.h>
#include "opencv_utils.h"

namespace perpendicular_parking {

// pull in some usefull range-adaptors
using boost::adaptors::map_values;
using boost::adaptors::reversed;
using boost::adaptors::transformed;
const auto points = common::members(&ObservationPoint::point);
const auto types = common::members(&ObservationPoint::type);

const std::string ParkingSpot::NAMESPACE("parking_spot");

const ParameterString<double> ParkingSpot::PARKING_SPOT_DEPTH(
    "parking_spot_depth");
const ParameterString<double> ParkingSpot::PARKING_SPOT_WIDTH(
    "parking_spot_width");

const ParameterString<int> ParkingSpot::POINTS_TO_SKIP_FRONT(
    NAMESPACE + "/points_to_skip_front");

const ParameterString<int> ParkingSpot::POINTS_TO_SKIP_BACK(
    NAMESPACE + "/points_to_skip_back");

const ParameterString<double> ParkingSpot::EPSILON_X_CLASSIFIER(
    NAMESPACE + "/epsilon_x_classifier");

const ParameterString<double> ParkingSpot::SMALLEST_OBSTACLE_AREA(
    NAMESPACE + "/smallest_obstacle_area");

const ParameterString<double> ParkingSpot::FREE_MIN_DEPTH_TH(
    NAMESPACE + "/free_min_depth_th");
const ParameterString<double> ParkingSpot::SPOT_SCAN_LINE_PADDING_X(
    "spot_scan_line_padding_x");

const ParameterString<double> ParkingSpot::RANSAC_MIN_DIST(
    NAMESPACE + "/ransac/minimal_model_distance");
const ParameterString<int> ParkingSpot::RANSAC_MIN_CLUSTER_SIZE(
    NAMESPACE + "/ransac/minimal_consensus_set_size");
const ParameterString<int> ParkingSpot::RANSAC_NR_LINES(
    NAMESPACE + "/ransac/expected_nr_of_lines");

ParkingSpot::ParkingSpot(int id,
                         const MapPoseConstPtr &left_entrance,
                         const MapPoseConstPtr &right_entrance,
                         const ParameterInterface *const parameter_interface,
                         const common::CameraTransformation *const cam_transform,
                         const std::array<cv::Point2f, 4> &fov)
    : id_(id),
      left_entrance_(left_entrance),
      right_entrance_(right_entrance),
      camera_transformation_(cam_transform),
      field_of_vision_(fov),
      PARKING_SPOT_DEPTH_(parameter_interface->getParam(PARKING_SPOT_DEPTH)),
      PARKING_SPOT_WIDTH_(parameter_interface->getParam(PARKING_SPOT_WIDTH)),
      EPSILON_X_CLASSIFIER_(parameter_interface->getParam(EPSILON_X_CLASSIFIER)),
      SMALLEST_OBSTACLE_AREA_(parameter_interface->getParam(SMALLEST_OBSTACLE_AREA)),
      SPOT_SCAN_LINE_PADDING_X_(parameter_interface->getParam(SPOT_SCAN_LINE_PADDING_X)),
      POINTS_TO_SKIP_BACK_(parameter_interface->getParam(POINTS_TO_SKIP_BACK)),
      POINTS_TO_SKIP_FRONT_(parameter_interface->getParam(POINTS_TO_SKIP_FRONT)),
      RANSAC_MIN_DIST_(parameter_interface->getParam(RANSAC_MIN_DIST)),
      RANSAC_MIN_CLUSTER_SIZE_(static_cast<std::size_t>(
          parameter_interface->getParam(RANSAC_MIN_CLUSTER_SIZE))),
      RANSAC_NR_LINES_(static_cast<std::size_t>(parameter_interface->getParam(RANSAC_NR_LINES))) {
  // no need to register parameters here, this is done in ParkingLot c'tor via
  // ParkingSpot::setParams
}

void ParkingSpot::registerParams(ParameterInterface *parameters_ptr) {
  parameters_ptr->registerParam(POINTS_TO_SKIP_FRONT);
  parameters_ptr->registerParam(POINTS_TO_SKIP_BACK);
  parameters_ptr->registerParam(EPSILON_X_CLASSIFIER);
  parameters_ptr->registerParam(SMALLEST_OBSTACLE_AREA);
  parameters_ptr->registerParam(FREE_MIN_DEPTH_TH);
  parameters_ptr->registerParam(RANSAC_MIN_DIST);
  parameters_ptr->registerParam(RANSAC_MIN_CLUSTER_SIZE);
  parameters_ptr->registerParam(RANSAC_NR_LINES);
}
bool ParkingSpot::isInFieldOfView(const cv::Point2f &p) const {
  return cv::pointPolygonTest(toInputArray(field_of_vision_), p, false) > 0;
}

OccupationState ParkingSpot::occupationState() const {
  if (recently_updated_) {
    recently_updated_ = false;
  } else {
    return latest_state_;
  }

  if (observation_points_.empty() ||
      (vehicle_T_map_ * (*right_entrance_)).translation().x() < 0.3) {
    latest_state_ = OccupationState::UNKNOWN;
    return latest_state_;
  }

  if (isSpotNoParkingArea()) {
    latest_state_ = OccupationState::X;
    return latest_state_;
  }

  const auto occupied_area = occupiedArea();

  if (occupied_area > SMALLEST_OBSTACLE_AREA_) {
    const bool is_shadowed = isShadowedByObstacle(occupied_area);
    ROS_DEBUG_COND(is_shadowed, "spot #%d is shadowed see angles!!", id_);
    latest_state_ = is_shadowed ? OccupationState::FREE : OccupationState::OCCUPIED;
    return latest_state_;
  }

  latest_state_ = OccupationState::FREE;
  return latest_state_;
}

perception_msgs::PerpendicularParkingSpot ParkingSpot::asMsg() const {
  perception_msgs::PerpendicularParkingSpot msg;

  msg.header.frame_id = "pp_map";

  msg.id.data = id_;
  msg.entrance_pose_left = tf2::toMsg(*left_entrance_);
  msg.entrance_pose_right = tf2::toMsg(*right_entrance_);

  return msg;
}

const MapPose &ParkingSpot::leftEntrance() const { return *left_entrance_; }

const MapPose &ParkingSpot::rightEntrance() const { return *right_entrance_; }

void ParkingSpot::updateObservation(const ObservationPoints &observation_points,
                                    const std::pair<double, double> &vertical_angle_limits,
                                    const MapPose &map_T_vehicle) {

  // set transformation to world
  vehicle_T_map_ = map_T_vehicle.inverse();

  if (!isInFieldOfView(toCV(
          to2D(VehiclePoint{(vehicle_T_map_ * (*right_entrance_)).translation()})))) {
    return;
  }

  recently_updated_ = true;

  // set angle field
  vertical_angle_limits_ = vertical_angle_limits;

  // update observation per scan line
  for (const auto &obs_pair : observation_points) {
    // no observation for scan line known?
    if (!common::contains(observation_points_, obs_pair.first)) {
      observation_points_.insert(obs_pair);
      continue;
    }
    const auto &old_observation = observation_points_[obs_pair.first];
    const auto &new_observation = obs_pair.second;

    // only update if new observation is deeper inside parking slot or an
    // obstacle has been detected
    if ((new_observation.point.x() > old_observation.point.x()) ||
        (new_observation.type == ObservationPoint::Type::OBSTACLE)) {
      observation_points_[obs_pair.first] = obs_pair.second;
    }
  }

  // set detected lines
  if (!observation_points_.empty()) {
    detected_lines_ = fitLinesR(3);
    ROS_DEBUG("size of detected_lines of spot #%d is %zu", id_, detected_lines_.size());
  }
}

MapPoints ParkingSpot::freeSpace() const {
  MapPoints free_space;
  free_space.reserve(observation_points_.size() + 2);

  free_space.emplace_back(left_entrance_->translation());
  boost::push_back(free_space, observation_points_ | map_values | points);
  free_space.emplace_back(right_entrance_->translation());

  return free_space;
}

const MapPoint &ParkingSpot::rightMostObstaclePoint() const {
  const auto points = observation_points_ | reversed | map_values | types;
  auto obstacle_it = boost::find(points, ObservationPoint::Type::OBSTACLE);

  return obstacle_it.base()->point;
}

bool ParkingSpot::isSpotNoParkingArea() const {

  assert(POINTS_TO_SKIP_FRONT_ > 0 &&
         "parameter POINTS_TO_SKIP_FRONT has to be greater than 0!");
  assert(POINTS_TO_SKIP_BACK_ < 0 &&
         "parameter POINTS_TO_SKIP_BACK has to be less than 0!");

  if (observation_points_.size() < POINTS_TO_SKIP_FRONT_ - POINTS_TO_SKIP_BACK_ + 2UL)
    return false;

  // skipping first points in order to classify correctly if accuracy isn't
  // very high

  const auto map_points = (observation_points_ | map_values | points)
                              .advance_begin(POINTS_TO_SKIP_FRONT_)
                              .advance_end(POINTS_TO_SKIP_BACK_);

  // determine intersection point of the two x-lines
  auto map_point = std::begin(map_points);

  for (; map_point != boost::prior(map_points.end()); map_point++) {
    const auto next = std::next(map_point);
    const auto former = boost::prior(map_point);
    if ((next->x() - map_point->x()) * (map_point->x() - former->x()) <= 0.0) {
      break;
    }
  }
  // dividing points
  const auto rising = boost::make_iterator_range(std::begin(map_points), map_point);

  const auto falling =
      boost::make_iterator_range(std::next(map_point), map_points.end());
  // not enough points
  if (boost::size(falling) < 2 || boost::size(rising) < 2) {
    return false;
  }

  // determination of x-angles
  const double slope_angle = atan(PARKING_SPOT_WIDTH_ / PARKING_SPOT_DEPTH_);

  // computation of real line angles
  const double line_angle1 =
      std::fabs(common::getAngleToPrincipalComponent(Eigen::Vector2d::UnitX(), rising));
  const double line_angle2 =
      std::fabs(common::getAngleToPrincipalComponent(Eigen::Vector2d::UnitX(), falling));

  // tolerance
  return std::fabs(line_angle1 - slope_angle) < EPSILON_X_CLASSIFIER_ &&
         std::fabs(line_angle2 - slope_angle) < EPSILON_X_CLASSIFIER_;
}

double ParkingSpot::occupiedArea() const {
  // compute free space

  const auto spot_boundary =
      MapLine::Through(to2D(Eigen::Vector3d{left_entrance_->translation()}),
                       to2D(Eigen::Vector3d{right_entrance_->translation()}));
  const auto depth = PARKING_SPOT_DEPTH_ - SPOT_SCAN_LINE_PADDING_X_;
  const auto width_per_point =
      (left_entrance_->translation() - right_entrance_->translation()).norm() /
      observation_points_.size();
  const auto occupiedArea = [&depth, &width_per_point, &spot_boundary](const auto &p) {
    return (depth - std::fabs(spot_boundary.distance(to2D(p)))) * width_per_point;
  };

  return common::sum(observation_points_ | map_values | points | transformed(occupiedArea));
}

bool ParkingSpot::isShadowedByObstacle(const double occupied_area) const {

  if (detected_lines_.empty()) {
    ROS_DEBUG("angles_computation: detected lines of spot#%d are empty", id_);
    return false;
  }

  const IntersectionLine u_axis =
      IntersectionLine::Through(ImagePointExact::Zero(), ImagePointExact::UnitX());

  std::vector<std::pair<ImagePointExact, ImagePointExact>> line_points;
  line_points.reserve(detected_lines_.size());
  for (const auto &line : detected_lines_) {
    const MapPoint start_map{
        to3D(line.projection(to2D(MapPoint{left_entrance_->translation()})))};
    const MapPoint end_map{
        to3D(line.projection(to2D(MapPoint{right_entrance_->translation()})))};
    line_points.emplace_back(
        camera_transformation_->transformGroundToImageExact(vehicle_T_map_ * start_map),
        camera_transformation_->transformGroundToImageExact(vehicle_T_map_ * end_map));
  }

  const auto toAngle = [](const auto &p) {
    return common::getAngle(
        ImagePointExact::UnitX(),
        common::ensureSameOrientation(p.first - p.second, ImagePointExact::UnitY()));
  };
  const auto between = [this](const auto &a) {
    return a >= vertical_angle_limits_.first && a <= vertical_angle_limits_.second;
  };

  const auto angles = line_points | transformed(toAngle);

  const double min_area = 0.05;

  return boost::algorithm::any_of(angles, between) &&
         occupied_area < min_area && detected_lines_.size() == 1;
}

inline auto modelSupport(const MapLine &model, const double min_dist) {
  return [&model, min_dist](const auto &p) {
    return std::fabs(model.distance(to2D(p))) < min_dist;
  };
};

MapLines ParkingSpot::fitLinesR(const std::size_t nr_lines) const {
  MapLines lines;
  lines.reserve(nr_lines);
  // use observations
  auto detected_points = common::make_eigen_vector(
      (observation_points_ | map_values | points).advance_begin(1).advance_end(-1));

  const std::size_t max_it = observation_points_.size() / 2;
  const double min_dist = 0.01;
  const std::size_t min_set_size = 5;
  MapPoints data_next_it;

  // loop in order to detect the specified number of lines
  for (std::size_t i = 0; i < nr_lines; i++) {

    std::size_t best_set_size = 0;
    boost::optional<MapLine> best_model = boost::none;

    // ransac loop
    for (std::size_t it = 0; it < max_it; it++) {
      boost::random_shuffle(detected_points);
      const MapLine model = MapLine::Through(to2D(detected_points.front()),
                                             to2D(detected_points.back()));
      const auto divider =
          boost::partition(detected_points, modelSupport(model, min_dist));
      const MapPoints consensus_set(detected_points.begin(), divider);
      const auto model_size = consensus_set.size();
      if (model_size > best_set_size && model_size > min_set_size) {
        //  least squares fit for best model
        best_model =
            boost::make_optional<MapLine>(leastSquaresFit(MapPoints(consensus_set)));
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

MapLine ParkingSpot::leastSquaresFit(const MapPoints &points) const {
  const Eigen::Vector2d direction =
      common::getPrincipalComponent(common::toMatrix2D(points)).normalized();
  const Eigen::Vector2d origin = to2D(common::mean<MapPoint>(points));

  return MapLine{origin, direction};
}

}  // namespace perpendicular_parking
