#include "perpendicular_parking.h"

THIRD_PARTY_HEADERS_BEGIN
#include <tf2_eigen/tf2_eigen.h>
#include <boost/algorithm/clamp.hpp>
#include <boost/range/algorithm.hpp>
#include <boost/range/algorithm_ext.hpp>
THIRD_PARTY_HEADERS_END
#include "../utils/foot_finder.h"
#include "../utils/step_detection.h"
#include "common/angle_conversions.h"
#include "common/basic_statistics.h"
#include "common/console_colors.h"
#include "common/eigen_adaptors.h"
#include "common/eigen_functors.h"
#include "common/eigen_utils.h"
#include "common/normal_shift.h"
#include "common/polynomial_utils.h"
#include "common/unique_erase.h"
#include "common/pca_eigen.h"
#include "vehicle_point_conversions.h"

namespace perpendicular_parking {

const std::string PerpendicularParking::NAMESPACE("perpendicular_parking");

const ParameterString<double> PerpendicularParking::LEFT_LANE_POLY_STEP(
    NAMESPACE + "/left_lane_poly_step");
const ParameterString<int> PerpendicularParking::POLYNOMIAL_DEGREE_LINE(
    NAMESPACE + "/polynomial_degree_line");
const ParameterString<int> PerpendicularParking::POLYNOMIAL_DEGREE_CURVE(
    NAMESPACE + "/polynomial_degree_curve");
const ParameterString<double> PerpendicularParking::SCAN_LINE_LENGTH(
    NAMESPACE + "/scan_line_length");
const ParameterString<double> PerpendicularParking::STEP_DETECTION_THLD(
    NAMESPACE + "/step_detection_threshold");
const ParameterString<int> PerpendicularParking::STEP_DETECTION_REF_FUNC_SIZE(
    NAMESPACE + "/step_detection_ref_func_size");
const ParameterString<double> PerpendicularParking::MIN_OUTLAYER_DIST(
    NAMESPACE + "/min_outlayer_dist");
const ParameterString<double> PerpendicularParking::MIN_CLUSTER_CENTER_DIST(
    NAMESPACE + "/min_cluster_center_dist");
const ParameterString<int> PerpendicularParking::CLUSTER_MIN_SIZE(
    NAMESPACE + "/cluster_min_size");
const ParameterString<int> PerpendicularParking::CLUSTER_SIZE(NAMESPACE +
                                                              "/cluster_size");
const ParameterString<double> PerpendicularParking::CUTTING_DISTANCE(
    NAMESPACE + "/cutting_distance");
const ParameterString<double> PerpendicularParking::REMOVAL_DISTANCE_FRONT(
    NAMESPACE + "/removal_distance_front");
const ParameterString<int> PerpendicularParking::MIN_SIZE_LANE_POINTS(
    NAMESPACE + "/min_size_lane_points");
const ParameterString<double> PerpendicularParking::MAX_TIME_WOUT_STARTPOSE(
    NAMESPACE + "/max_time_without_startpose");
const ParameterString<double> PerpendicularParking::EPSILON_POLY_POINTS(
    NAMESPACE + "/epsilon_poly_points");
const ParameterString<double> PerpendicularParking::SCAN_LINE_SHIFT(
    NAMESPACE + "/scan_line_shift");
const ParameterString<double> PerpendicularParking::ENDLINE_RESET_DISTANCE(
    NAMESPACE + "/endline_reset_distance");

PerpendicularParking::PerpendicularParking(tf_helper::TFHelperInterface<double> *const world_coordinates_helper,
                                           ParameterInterface *parameters_ptr,
                                           const common::CameraTransformation *const camera_transform)
    : img_size(0, 0),
      world_coordinates_helper_(world_coordinates_helper),
      parameters_ptr_(parameters_ptr),
      camera_transform_(camera_transform),
      activated_(false),
      parking_lot_detected_(false) {
  parking_start_classifier = std::make_unique<ParkingStartClassifier>(
      parameters_ptr, camera_transform_, world_coordinates_helper);
  parking_lot = std::make_unique<ParkingLot>(
      parameters_ptr, camera_transform_, world_coordinates_helper);
  parking_end_classifier = std::make_unique<ParkingEndClassifier>(
      parameters_ptr, camera_transform_, world_coordinates_helper);

  parameters_ptr->registerParam(LEFT_LANE_POLY_STEP);
  parameters_ptr->registerParam(POLYNOMIAL_DEGREE_LINE);
  parameters_ptr->registerParam(POLYNOMIAL_DEGREE_CURVE);
  parameters_ptr->registerParam(SCAN_LINE_LENGTH);
  parameters_ptr->registerParam(STEP_DETECTION_THLD);
  parameters_ptr->registerParam(STEP_DETECTION_REF_FUNC_SIZE);
  parameters_ptr->registerParam(MIN_OUTLAYER_DIST);
  parameters_ptr->registerParam(CLUSTER_MIN_SIZE);
  parameters_ptr->registerParam(CLUSTER_SIZE);
  parameters_ptr->registerParam(MIN_CLUSTER_CENTER_DIST);
  parameters_ptr->registerParam(CUTTING_DISTANCE);
  parameters_ptr->registerParam(REMOVAL_DISTANCE_FRONT);
  parameters_ptr->registerParam(MIN_SIZE_LANE_POINTS);
  parameters_ptr->registerParam(MAX_TIME_WOUT_STARTPOSE);
  parameters_ptr->registerParam(EPSILON_POLY_POINTS);
  parameters_ptr->registerParam(SCAN_LINE_SHIFT);
  parameters_ptr->registerParam(ENDLINE_RESET_DISTANCE);
}

ParkingSpotsConstRef PerpendicularParking::findFreeParkingSpots(
    const cv::Mat &img, const LineVehiclePoints &lanes, const ros::Time &timestamp) {
  if (!activated_) {
    return {};
  }
  world_coordinates_helper_->update(timestamp);
  ROS_DEBUG_STREAM("transform is : translation: "
                   << world_coordinates_helper_->getTransform().translation() << " \n rotation: "
                   << world_coordinates_helper_->getTransform().linear());
  const std::size_t min_lane_points_size =
      static_cast<std::size_t>(parameters_ptr_->getParam(MIN_SIZE_LANE_POINTS));
  if (lanes[LINESPEC_LEFT].size() < min_lane_points_size) {
    return {};
  }
  ROS_INFO_THROTTLE(1, "perpendicular_parking activated.");
  img_size = img.size();

  generatePolynomialPoints(allPointsToLeft(lanes));
  left_lane_polynom_ =
      common::fitToPoints(allPointsToLeft(lanes), getPolynomialDegree());

  ScanLines scan_lines = createScanLines(polynomial_points);
  ImagePoints feature_points = apply1DGradientDetector(img, scan_lines);

  FeaturePointCluster cluster_raw(*camera_transform_, feature_points);

  parking_lot_detected_ = parking_lot->startDetected();

  boost::optional<WorldPose> parking_lot_end = boost::none;
  ParkingSpotsConstRef free_spots;
  if (parking_lot_detected_) {
    //    parking_start_classifier->classify(cluster_raw, lanes[LINESPEC_LEFT],
    //    timestamp);
    parking_start_classifier->classify(cluster_raw, left_lane_polynom_, timestamp);
    ROS_INFO_ONCE("Setting parking lot start!");
    // look for parking_start_line
    const auto start_pose_world = parking_start_classifier->startLinePose();
    // set parking lot start
    parking_lot->setStart(start_pose_world);
    // update parking lot
    parking_lot->update(img, left_lane_polynom_);
    // detect parking end line
    parking_lot_end = parking_end_classifier->detectEnd(parking_lot->allParkingSpots(),
                                                        img,
                                                        timestamp,
                                                        left_lane_polynom_,
                                                        lanes,
                                                        parking_lot->mapPose());
    if (parking_lot_end) {
      // if endline has been detected, set parking end line
      parking_lot->setEnd(parking_lot_end.get());
    }
    free_spots = parking_lot->freeParkingSpots();
  } else {
    // if parking start not yet detected, search only parking start line
    if ((parking_start_classifier->classify(
             cluster_raw, left_lane_polynom_, timestamp) == Type::START)) {
      parking_lot->setStart(parking_start_classifier->startLinePose());
    }
  }
  // case: no free spots detected
  if (parking_lot_end) {
    // if parking end line has been passed and no free parking spot is present,
    // reset parking_start_classifier, parking lot and parking end classifier
    // and search for next parking lot
    const Eigen::Affine3d vehicle_T_world =
        world_coordinates_helper_->getTransform().inverse();
    const double endline_distance_for_reset =
        parameters_ptr_->getParam(ENDLINE_RESET_DISTANCE);
    const VehiclePoint endline_position =
        vehicle_T_world * parking_lot_end->translation();

    ROS_DEBUG("DISTANCE TO ENDLINE IS %f", endline_position[0]);

    if (endline_position[0] < endline_distance_for_reset) {
      parking_start_classifier->reset();
      parking_lot->reset();
      parking_end_classifier->reset();
    }

    // devalidate spots that lay beyond parking endline pose
    const auto vehicle_T_map = vehicle_T_world * parking_lot->mapPose();
    const auto isSpotBeyondEndline = [&endline_position, &vehicle_T_map](const auto &spot) {
      return endline_position.norm() <
             (vehicle_T_map * MapPoint{spot.get().rightEntrance().translation()})
                 .norm();
    };
    boost::remove_erase_if(free_spots, isSpotBeyondEndline);
  }
  return free_spots;
}

const WorldPose PerpendicularParking::mapPose() const {
  return parking_lot->mapPose();
}

ScanLines PerpendicularParking::createScanLines(VehiclePoints &left_lane) {
  if (left_lane.size() < 2) {
    return {};
  }
  const double remove_thld = parameters_ptr_->getParam(REMOVAL_DISTANCE_FRONT);
  const auto isPointInFrontOfVehicle = [&remove_thld](const auto &lp) {
    return lp[0] < remove_thld;
  };
  // remove points lying behind vehicle
  boost::remove_erase_if(left_lane, isPointInFrontOfVehicle);
  VehiclePoints start_points, end_points, dis_points;
  const double scan_line_length = parameters_ptr_->getParam(SCAN_LINE_LENGTH);
  const double scan_line_shift = parameters_ptr_->getParam(SCAN_LINE_SHIFT);
  const double discretization_params_step = parameters_ptr_->getParam(LEFT_LANE_POLY_STEP);

  if (parking_lot->startDetected()) {
    const double max_dist = parameters_ptr_->getParam(CUTTING_DISTANCE);
    const Eigen::Affine3d &vehicle_T_world =
        world_coordinates_helper_->getTransform().inverse();
    const WorldPoint start_in_world{parking_start_classifier->startLinePose().translation()};
    const VehiclePoint start_in_vehicle = vehicle_T_world * start_in_world;
    VehiclePoints cutted = left_lane;
    boost::remove_erase_if(cutted, [&start_in_vehicle, &max_dist](const auto &p) {
      return (p - start_in_vehicle).norm() > max_dist;
    });
    const common::PolynomialDegree poly_degree = getPolynomialDegree();
    if (cutted.size() <= static_cast<std::size_t>(poly_degree)) {
      return {};
    }
    const common::DynamicPolynomial left_lane_polynom_cutted =
        common::fitToPoints(cutted, poly_degree);

    const common::DiscretizationParams discretization_params = generateParams(
        cutted.front().x(), cutted.back().x(), discretization_params_step);

    //    common::discretisize(left_lane_polynom_cutted, discretization_params,
    //    &dis_points);
    common::normalShift(
        left_lane_polynom_cutted, scan_line_shift, discretization_params, &start_points);
    common::normalShift(
        left_lane_polynom_cutted, scan_line_length, discretization_params, &end_points);
  } else {
    const common::DiscretizationParams discretization_params = generateParams(
        (*left_lane.begin())[0], (*(left_lane.end() - 1))[0], discretization_params_step);

    //    common::discretisize(left_lane_polynom_, discretization_params,
    //    &dis_points);
    common::normalShift(
        left_lane_polynom_, scan_line_shift, discretization_params, &start_points);
    common::normalShift(
        left_lane_polynom_, scan_line_length, discretization_params, &end_points);
  }

  ImagePoints start_points_img, end_points_img;
  camera_transform_->transformGroundToImage(start_points, &start_points_img);
  camera_transform_->transformGroundToImage(end_points, &end_points_img);

  ScanLines resulting_lines;
  boost::transform(start_points_img,
                   end_points_img,
                   std::back_inserter(resulting_lines),
                   [](auto a, auto b) { return ScanLine(a, b); });
  return clipScanLines(resulting_lines);
}

ImagePoints PerpendicularParking::apply1DGradientDetector(const cv::Mat &img,
                                                          const ScanLines &scan_lines) {
  ImagePoints image_points;

  const float threshold =
      static_cast<float>(parameters_ptr_->getParam(STEP_DETECTION_THLD));
  const unsigned int range_length =
      static_cast<unsigned int>(parameters_ptr_->getParam(STEP_DETECTION_REF_FUNC_SIZE));

  const std::vector<float> reference_function =
      step_detection::createReferenceFunction(range_length);

  for (const ScanLine &scan_line : scan_lines) {
    auto feature_points = step_detection::detectStep(
        img, scan_line, reference_function, threshold, true);

    // only take points along scan_line with two related feature_points as valid
    // ones
    if (feature_points.size() == 2UL) {
      // take the mean of the two points as found feature point
      const ImagePoint mean = (feature_points.front() + feature_points.back()) / 2;
      image_points.push_back(mean);
    }
  }
  return image_points;
}

VehiclePoints PerpendicularParking::allPointsToLeft(const LineVehiclePoints &lanes) {
  VehiclePoints left_points = lanes[LINESPEC_LEFT];

  common::sortAlongPrincipalComponent(&left_points);

  return deleteOutlayers(left_points);
}

VehiclePoints PerpendicularParking::deleteOutlayers(const VehiclePoints &fused) {
  LanePointsClusters clustered = clusterLanePoints(fused);
  VehiclePoints out;
  for (auto &c : clustered) {
    deleteClusterOutlayers(&c);
    boost::push_back(out, c);
  }

  return out;
}

void PerpendicularParking::deleteClusterOutlayers(VehiclePoints *cluster) {
  // perform pca
  const Eigen::MatrixXd cluster_data = common::toMatrix2D(*cluster);
  const GroundPoint mean = cluster_data.colwise().mean();
  Eigen::MatrixXd components;
  Eigen::VectorXd scores;
  common::pca_svd(cluster_data, &components, &scores);
  // second component as vector
  const GroundPoint normal_component = getNormalComponent(components, mean);
  const double outlayer_dist = parameters_ptr_->getParam(MIN_OUTLAYER_DIST);
  const double center_dist = parameters_ptr_->getParam(MIN_CLUSTER_CENTER_DIST);
  boost::remove_erase_if(
      *cluster, [&mean, &normal_component, &outlayer_dist, &center_dist](const auto &c) {
        return std::fabs((to2D(c) - mean).normalized().dot(normal_component)) > outlayer_dist &&
               (to2D(c) - mean).norm() > center_dist;
      });
}

void PerpendicularParking::generatePolynomialPoints(const VehiclePoints &actual_poly_points) {
  if (actual_poly_points.size() < 2) {
    return;
  }
  if (!parking_lot->startDetected()) {
    polynomial_points = actual_poly_points;
    return;
  }
  // getting actual translation of start_line_pose in vehicle coordinates
  const WorldPoint start_point_world =
      parking_start_classifier->startLinePose().translation();
  const Eigen::Affine3d &vehicle_T_world = world_coordinates_helper_->getTransform();
  const VehiclePoint start_point_vehicle = vehicle_T_world.inverse() * start_point_world;
  // transforming actual lane_points in world frame
  WorldPoints ap_world;
  boost::push_back(ap_world, actual_poly_points | common::eigen_transformed(vehicle_T_world));
  // inserting actual_poly_points at end of polynomial_points in world frame

  if (!polynomial_points_world.empty()) {
    common::sortAlongPrincipalComponent(&ap_world);
    common::sortAlongPrincipalComponent(&polynomial_points_world);
    const double d_first_to_start = (ap_world.front() - start_point_world).norm();
    boost::remove_erase_if(polynomial_points_world,
                           [&d_first_to_start, &start_point_world](const auto &pp) {
                             return (pp - start_point_world).norm() > d_first_to_start;
                           });
  }
  common::sortAlongPrincipalComponent(&polynomial_points_world);
  const double min_point_distance = 0.05;
  common::unique_erase(polynomial_points_world, common::areClose(min_point_distance));
  boost::push_back(polynomial_points_world, ap_world);
  common::sortAlongPrincipalComponent(&polynomial_points_world);
  // back to vehicle frame
  boost::push_back(polynomial_points,
                   polynomial_points_world |
                       common::eigen_transformed(vehicle_T_world.inverse()));
  if (polynomial_points.size() < 2)
    return;
  // principle component
  const Eigen::Vector2d pc = common::ensureSameOrientation(
      common::getPrincipalComponent(polynomial_points), Eigen::Vector2d::UnitX());
  // threshold for filtering
  const double start_projected_on_pc = start_point_vehicle.head<2>().dot(pc);
  // remove all points, whose projection onto pc is smaller than those of
  // start_point, only needed for first frames, in order to ensure, that polynom
  // is only rememberred, if start is detected
  const double epsilon = parameters_ptr_->getParam(EPSILON_POLY_POINTS);
  boost::remove_erase_if(polynomial_points, [&start_projected_on_pc, &pc, &epsilon](auto pp) {
    return to2D(pp).dot(pc) < start_projected_on_pc - epsilon;
  });
  // ordering points
  common::sortAlongPrincipalComponent(&polynomial_points);
}

LanePointsClusters PerpendicularParking::clusterLanePoints(const VehiclePoints &lane_points) {
  // Points have to be ordered when function's called
  const std::size_t cluster_size =
      static_cast<std::size_t>(parameters_ptr_->getParam(CLUSTER_SIZE));
  LanePointsClusters clusters;
  clusters.reserve((lane_points.size() % cluster_size) + 1);
  VehiclePoints cluster;
  cluster.reserve(cluster_size);
  std::size_t count = 1;
  for (const auto &lane_point : lane_points) {
    if (count < cluster_size) {
      cluster.push_back(lane_point);
      count++;
    } else {
      cluster.push_back(lane_point);
      clusters.push_back(cluster);
      count = 1;
      cluster.clear();
    }
  }
  // last cluster has to contain at least cluster_min_size points
  const std::size_t cluster_min_size = parameters_ptr_->getParam(CLUSTER_MIN_SIZE);
  if (cluster.size() >= cluster_min_size) {
    clusters.push_back(cluster);
  }
  return clusters;
}

GroundPoint PerpendicularParking::getNormalComponent(const Eigen::MatrixXd &components,
                                                     const GroundPoint &mean) {
  const GroundPoint pc{components.block<2, 1>(0, 0)};
  const GroundPoint sc{components.block<2, 1>(0, 1)};
  // note: this is last frame's left_lane_polynom but the difference is not
  // large so take it
  const auto normal_vector = common::normal(left_lane_polynom_, mean.x());
  return std::fabs(pc.dot(normal_vector)) > std::fabs(sc.dot(normal_vector)) ? pc : sc;
}

const ScanLines PerpendicularParking::clipScanLines(const ScanLines &lines) const {
  // remove scanlines whose start and end are out of  image

  // trim all the start and end points of scan lines to image limits
  ScanLines out;
  for (auto line : lines) {
    if (ScanLine::clip(cv::Rect(0, 0, img_size.width, img_size.height), line))
      out.push_back(line);
  }
  return out;
}

common::DiscretizationParams PerpendicularParking::generateParams(const double x_1,
                                                                  const double x_2,
                                                                  const double step) {
  return {std::min(x_1, x_2), std::max(x_1, x_2), step};
}

common::PolynomialDegree PerpendicularParking::getPolynomialDegree() const {
  if (parking_lot->startDetected()) {
    return parameters_ptr_->getParam(POLYNOMIAL_DEGREE_LINE);
  } else {
    return parameters_ptr_->getParam(POLYNOMIAL_DEGREE_CURVE);
  }
}

void PerpendicularParking::reset() {
  parking_start_classifier->reset();
  parking_lot->reset();
  parking_end_classifier->reset();
  polynomial_points.clear();
  polynomial_points_world.clear();
  activated_ = false;
  parking_lot_detected_ = false;
}

void PerpendicularParking::setActivationStatus(const bool search) {
  if (this->activated_ && !search) {
    ROS_INFO_THROTTLE(1, "perpendicular_parking deactivated.");
    reset();
  }
  this->activated_ = search;
}
}  // namespace perpendicular_parking
