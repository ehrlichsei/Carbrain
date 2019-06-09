#include "parking_lot.h"

#include "../utils/foot_finder.h"
#include "../utils/step_detection.h"
#include "common/adaptors.h"
#include "common/angle_conversions.h"
#include "common/basic_statistics.h"
#include "common/best_score.h"
#include "common/camera_transformation.h"
#include "common/camera_transformation_parameters.h"
#include "common/console_colors.h"
#include "common/eigen_adaptors.h"
#include "common/eigen_functors.h"
#include "common/eigen_utils.h"
#include "common/make_vector.h"
#include "common/minmax_element.h"
#include "common/normal_shift.h"
#include "common/pca_eigen.h"
#include "common/polynomial_utils.h"
#include "common/polynomialfit.h"
#include "opencv_utils.h"

THIRD_PARTY_HEADERS_BEGIN
#include <ros/console.h>
#include <Eigen/Geometry>
#include <functional>

#include <boost/algorithm/cxx11/all_of.hpp>
#include <boost/range/adaptors.hpp>
#include <boost/range/algorithm.hpp>
#include <boost/range/algorithm_ext/push_back.hpp>
THIRD_PARTY_HEADERS_END

namespace ph = std::placeholders;

namespace perpendicular_parking {

using boost::adaptors::filtered;
using boost::adaptors::transformed;
using common::eigen_transformed;

const std::string ParkingLot::NAMESPACE("parking_lot");
const ParameterString<int> ParkingLot::N_SCAN_LINES_PER_SPOT(
    NAMESPACE + "/n_scan_lines_per_spot");
const ParameterString<double> ParkingLot::PARKING_SPOT_DEPTH(
    "parking_spot_depth");
const ParameterString<double> ParkingLot::PARKING_SPOT_WIDTH(
    "parking_spot_width");
const ParameterString<double> ParkingLot::MARKING_SCAN_STEP_WIDTH(
    NAMESPACE + "/marking_scan_step_width");

const ParameterString<int> ParkingLot::STEP_REFERENCE_FUNCTION_LENGTH(
    NAMESPACE + "/step_reference_function_length");
const ParameterString<double> ParkingLot::GRADIENT_DETECTION_THLD(
    NAMESPACE + "/gradient_detection_thld");

const ParameterString<double> ParkingLot::SPOT_SCAN_LINE_PADDING_X(
    "spot_scan_line_padding_x");

const ParameterString<double> ParkingLot::SPOT_SCAN_LINE_PADDING_Y(
    NAMESPACE + "/spot_scan_line_padding_y");

const ParameterString<int> ParkingLot::IMAGE_PADDING(NAMESPACE +
                                                     "/image_padding");

const ParameterString<double> ParkingLot::MARKING_WIDTH("marking_width");

const ParameterString<double> ParkingLot::MARKING_SCAN_LINE_WIDTH(
    NAMESPACE + "/marking_scan_line_width");

const ParameterString<double> ParkingLot::MARKKING_SCAN_LINE_PADDING_FRONT(
    NAMESPACE + "/marking_scan_line_padding_front");
const ParameterString<double> ParkingLot::MARKKING_SCAN_LINE_PADDING_BACK(
    NAMESPACE + "/marking_scan_line_padding_back");

const ParameterString<double> ParkingLot::SPOT_SCAN_LINE_OBSTACLE_SAFETY_MARGIN(
    NAMESPACE + "/spot_scan_line_obstacle_safety_margin");

const ParameterString<int> ParkingLot::N_MIN_MARKINGS(NAMESPACE +
                                                      "/n_min_markings");
const ParameterString<double> ParkingLot::MAX_SPAWN_DISTANCE(
    NAMESPACE + "/max_spawn_distance");
const ParameterString<double> ParkingLot::MARKING_UPDATE_SHIFT_FACTOR(
    NAMESPACE + "/marking_update_shift_factor");

const ParameterString<double> ParkingLot::START_POSE_MAX_ALLOWED_DEVIATION(
    NAMESPACE + "/start_pose_max_allowed_deviation");

const ParameterString<double> ParkingLot::MAX_MARKING_UPDATE_DISTANCE(
    NAMESPACE + "/max_marking_update_distance");

const ParameterString<double> ParkingLot::NEWTON_EPSILON(NAMESPACE +
                                                         "/newton_epsilon");

const ParameterString<int> ParkingLot::NEWTON_MAX_ITERATIONS(
    NAMESPACE + "/newton_max_iterations");

const ParameterString<int> ParkingLot::FIELD_OF_VISION_TOP(
    NAMESPACE + "/field_of_vision/top");
const ParameterString<int> ParkingLot::FIELD_OF_VISION_BOTTOM(
    NAMESPACE + "/field_of_vision/bottom");
const ParameterString<int> ParkingLot::FIELD_OF_VISION_LEFT(
    NAMESPACE + "/field_of_vision/left");
const ParameterString<int> ParkingLot::FIELD_OF_VISION_RIGHT(
    NAMESPACE + "/field_of_vision/right");

ParkingLot::ParkingLot(ParameterInterface *const parameters_ptr,
                       const common::CameraTransformation *const camera_transformation,
                       const tf_helper::TFHelperInterface<double> *const world_coordinates_helper)
    : parameters_ptr_(parameters_ptr),
      camera_transformation_(camera_transformation),
      world_coordinates_helper_(world_coordinates_helper),
      start_pose_changed_(false),
      spawn_no_new_markings_(false) {
  parameters_ptr->registerParam(PARKING_SPOT_DEPTH);
  parameters_ptr->registerParam(PARKING_SPOT_WIDTH);
  parameters_ptr->registerParam(MARKING_WIDTH);

  parameters_ptr->registerParam(STEP_REFERENCE_FUNCTION_LENGTH);
  parameters_ptr->registerParam(GRADIENT_DETECTION_THLD);

  parameters_ptr->registerParam(IMAGE_PADDING);

  parameters_ptr->registerParam(MARKING_SCAN_STEP_WIDTH);
  parameters_ptr->registerParam(MARKING_SCAN_LINE_WIDTH);
  parameters_ptr->registerParam(MARKKING_SCAN_LINE_PADDING_FRONT);
  parameters_ptr->registerParam(MARKKING_SCAN_LINE_PADDING_BACK);

  parameters_ptr->registerParam(N_SCAN_LINES_PER_SPOT);
  parameters_ptr->registerParam(SPOT_SCAN_LINE_PADDING_X);
  parameters_ptr->registerParam(SPOT_SCAN_LINE_PADDING_Y);

  parameters_ptr->registerParam(MARKING_UPDATE_SHIFT_FACTOR);
  parameters_ptr->registerParam(MAX_SPAWN_DISTANCE);
  parameters_ptr->registerParam(N_MIN_MARKINGS);
  parameters_ptr->registerParam(SPOT_SCAN_LINE_OBSTACLE_SAFETY_MARGIN);
  parameters_ptr->registerParam(MAX_MARKING_UPDATE_DISTANCE);
  parameters_ptr->registerParam(START_POSE_MAX_ALLOWED_DEVIATION);

  parameters_ptr->registerParam(NEWTON_EPSILON);
  parameters_ptr->registerParam(NEWTON_MAX_ITERATIONS);

  parameters_ptr->registerParam(FIELD_OF_VISION_TOP);
  parameters_ptr->registerParam(FIELD_OF_VISION_BOTTOM);
  parameters_ptr->registerParam(FIELD_OF_VISION_LEFT);
  parameters_ptr->registerParam(FIELD_OF_VISION_RIGHT);

  const int fov_top = parameters_ptr->getParam(FIELD_OF_VISION_TOP);
  const int fov_bottom = parameters_ptr->getParam(FIELD_OF_VISION_BOTTOM);
  const int fov_left = parameters_ptr->getParam(FIELD_OF_VISION_LEFT);
  const int fov_right = parameters_ptr->getParam(FIELD_OF_VISION_RIGHT);

  const common::EigenAlignedVector<ImagePoint> ground_in_image = {
      ImagePoint(fov_left, fov_top),
      ImagePoint(fov_right, fov_top),
      ImagePoint(fov_right, fov_bottom),
      ImagePoint(fov_left, fov_bottom)};

  common::EigenAlignedVector<VehiclePoint> fov_on_ground;
  camera_transformation_->transformImageToGround(ground_in_image, &fov_on_ground);
  field_of_vision_ = {{toCV(to2D(fov_on_ground[0])),
                       toCV(to2D(fov_on_ground[1])),
                       toCV(to2D(fov_on_ground[2])),
                       toCV(to2D(fov_on_ground[3]))}};

  ROS_DEBUG(
      "field of vision in order to check translation : "
      "((%fl,%fl),(%fl,%lf),(%fl,%fl),(%fl,%fl))",
      field_of_vision_[0].x,
      field_of_vision_[0].y,
      field_of_vision_[1].x,
      field_of_vision_[1].y,
      field_of_vision_[2].x,
      field_of_vision_[2].y,
      field_of_vision_[3].x,
      field_of_vision_[3].y);

  assert(boost::algorithm::all_of(field_of_vision_,
                                  [](const auto &a) { return a.x > 0; }) &&
         "false field of vision chosen. Check parameters!");

  // registering Parameters of ParkingSpot (in order not to have them registered
  // everytime, a new object is constructed)
  ParkingSpot::registerParams(parameters_ptr);
}

void ParkingLot::update(const cv::Mat &img, const common::DynamicPolynomial &left_lane_polynomial) {
  // don't do anything if start pose is not known
  if (!startDetected())
    return;

  // update world_T_pp_map_
  if (start_pose_changed_) {
    start_pose_changed_ = false;

    ROS_DEBUG("Start pose changed. Updating...");
    const Eigen::Affine3d vehicle_T_world =
        world_coordinates_helper_->getTransform().inverse();

    const Eigen::Affine3d vehicle_T_start = vehicle_T_world * start_.pose;
    const Eigen::Vector2d vehicle_P_start = vehicle_T_start.translation().head<2>();

    // or intersect?
    const Eigen::Vector2d lf =
        utils::findLotfusspunkt(left_lane_polynomial, vehicle_P_start);
    const double alpha_lf = common::normalAngle(left_lane_polynomial, lf.x());
    ROS_DEBUG("lot_normal_angle is %f", alpha_lf);

    // initialize in vehicle frame
    world_T_pp_map_.translation().head<2>() = vehicle_P_start;
    world_T_pp_map_.linear() =
        Eigen::AngleAxisd(alpha_lf, Eigen::Vector3d::UnitZ()).toRotationMatrix();  // perpendicular to polynomial
    // now tranform to world coordinate frame
    const Eigen::Affine3d world_T_vehicle = world_coordinates_helper_->getTransform();
    world_T_pp_map_ = world_T_vehicle * world_T_pp_map_;
  }

  // spawn first marking based on start line info
  if (markings_.empty()) {
    const Eigen::Affine3d vehicle_T_world =
        world_coordinates_helper_->getTransform().inverse();
    const Eigen::Affine3d vehicle_T_start = vehicle_T_world * start_.pose;
    const Eigen::Vector2d vehicle_P_start = vehicle_T_start.translation().head<2>();

    const double MAX_DISTANCE = parameters_ptr_->getParam(MAX_SPAWN_DISTANCE);
    if (vehicle_P_start.x() < MAX_DISTANCE) {
      // or intersect?
      const double x_lf = utils::findLotfusspunktX(left_lane_polynomial, vehicle_P_start);

      const Eigen::Matrix3d vehicle_R_start =
          vehicle_T_world.linear() * start_.pose.linear();
      //      const Eigen::Matrix3d vehicle_R_start =
      //          world_coordinates_helper_->getTransform().linear().inverse() *
      //          start_.pose.linear();
      const double yaw_start_in_vehicle = common::toYaw(vehicle_R_start);
      // rotation in vehicle frame
      const double alpha_lf = common::tangentAngle(left_lane_polynomial, x_lf);

      const double d = parameters_ptr_->getParam(PARKING_SPOT_DEPTH);
      const double l = d / tan(yaw_start_in_vehicle - alpha_lf);

      markings_.push_back(std::make_shared<MapPose>(Eigen::Translation3d(0, -l, 0)));
    }
  }

  addNewMarkings();

  // create parking slots between markings
  if (markings_.size() > 1) {
    for (std::size_t i = parking_spots_.size(); i < markings_.size() - 1; i++) {
      parking_spots_.emplace_back(
          i, markings_[i], markings_[i + 1], parameters_ptr_, camera_transformation_, field_of_vision_);
    }
  }
  updateMarkings(img, left_lane_polynomial);

  scanParkingSlots(img, left_lane_polynomial);
}

ParkingSpotsConstRef ParkingLot::freeParkingSpots() {
  ParkingSpotsConstRef free_spots;
  std::copy_if(parking_spots_.begin(),
               parking_spots_.end(),
               std::back_inserter(free_spots),
               [](auto &parking_spot) -> bool {
                 return parking_spot.occupationState() == OccupationState::FREE;
               });
  return free_spots;
}

ParkingSpotsConstRef ParkingLot::allParkingSpots() const {
  return ParkingSpotsConstRef(parking_spots_.begin(), parking_spots_.end());
}

inline double toCenter(const MapPoint &feature_point, double y_shift) {
  return feature_point[1] + y_shift;
}

void ParkingLot::updateMarkings(const cv::Mat &img,
                                const common::DynamicPolynomial &left_lane_polynomial) {
  const Eigen::Affine3d vehicle_T_world =
      world_coordinates_helper_->getTransform().inverse();
  const Eigen::Affine3d vehicle_T_map = vehicle_T_world * world_T_pp_map_;
  const Eigen::Affine3d map_T_vehicle = vehicle_T_map.inverse();

  const double SHIFT_FACTOR = parameters_ptr_->getParam(MARKING_UPDATE_SHIFT_FACTOR);
  const double MAX_DISTANCE = parameters_ptr_->getParam(MAX_MARKING_UPDATE_DISTANCE);

  const cv::Size IMG_SIZE = img.size();

  for (unsigned int i = 0; i < markings_.size(); i++) {
    auto &marking = *markings_[i];

    projectMarkingToPolynomial(marking, left_lane_polynomial);

    // if marking is behind the car, don't do anything
    if ((vehicle_T_map * marking).translation().x() < 0) {
      continue;
    }

    // check if marking is too far away
    const Eigen::Vector3d vehicle_P_marking =
        vehicle_T_world * world_T_pp_map_ * marking.translation();
    const Eigen::Vector3d map_V_vehicle_to_marking = map_T_vehicle.linear() * vehicle_P_marking;

    if ((map_V_vehicle_to_marking.y() > 0.0) || (-map_V_vehicle_to_marking.y() > MAX_DISTANCE)) {
      continue;
    }

    // only use scan lines from that side being a free parking spot
    bool left_free, right_free;
    if (i == 0) {
      if (parking_spots_.empty())
        continue;

      left_free = true;
      right_free = parking_spots_[0].occupationState() == OccupationState::FREE;
    } else if (i == (markings_.size() - 1)) {
      left_free = parking_spots_[i - 1].occupationState() == OccupationState::FREE;
      right_free = true;
    } else {
      left_free = parking_spots_[i - 1].occupationState() == OccupationState::FREE;
      right_free = parking_spots_[i].occupationState() == OccupationState::FREE;
    }

    if (!left_free && !right_free)
      continue;

    auto scan_lines = createMarkingScanLines(i, IMG_SIZE);
    auto map_feature_points = scanMarking(img, scan_lines, left_free, right_free);

    if (map_feature_points.left.empty() && map_feature_points.right.empty())
      continue;

    const double MARKING_WIDTH_2 = parameters_ptr_->getParam(MARKING_WIDTH) / 2;

    std::vector<double> center_detections;
    center_detections.reserve(map_feature_points.left.size() +
                              map_feature_points.right.size());
    boost::transform(map_feature_points.left,
                     std::back_inserter(center_detections),
                     std::bind(&toCenter, ph::_1, -MARKING_WIDTH_2));
    boost::transform(map_feature_points.right,
                     std::back_inserter(center_detections),
                     std::bind(&toCenter, ph::_1, MARKING_WIDTH_2));

    const double center_detection = common::mean(center_detections);

    ROS_DEBUG("Detected marking #%d at %f, old: %f",
              i,
              center_detection,
              marking.translation()[1]);

    const Eigen::Translation3d shift(
        0, (center_detection - marking.translation().y()) * SHIFT_FACTOR, 0);

    ROS_DEBUG("Updating marking #%d by shifting y %f", i, shift.y());

    world_T_pp_map_ = world_T_pp_map_ * shift;
  }
}

void ParkingLot::addNewMarkings() {
  if (spawn_no_new_markings_ || markings_.empty())
    return;

  const double SLOT_WIDTH = parameters_ptr_->getParam(PARKING_SPOT_WIDTH);
  const int NUM_MIN_MARKINGS = parameters_ptr_->getParam(N_MIN_MARKINGS);
  assert(NUM_MIN_MARKINGS > 0 && "");
  const double MAX_DISTANCE = parameters_ptr_->getParam(MAX_SPAWN_DISTANCE);

  Eigen::Affine3d map_T_world = world_T_pp_map_.inverse();
  Eigen::Affine3d vehicle_T_world = world_coordinates_helper_->getTransform().inverse();

  Eigen::Affine3d map_T_vehicle = map_T_world * vehicle_T_world.inverse();

  while (true) {
    const auto &lastKnownMarking = *markings_.back();

    const MapPose marking_guess =
        lastKnownMarking * Eigen::Translation3d(0, -SLOT_WIDTH, 0);

    const Eigen::Vector3d vehicle_P_marking_guess =
        vehicle_T_world * world_T_pp_map_ * marking_guess.translation();

    const Eigen::Vector3d map_V_vehicle_to_marking_guess =
        map_T_vehicle.linear() * vehicle_P_marking_guess;

    if ((-map_V_vehicle_to_marking_guess.y() > MAX_DISTANCE)) {
      break;
    }

    // check if new marking is further than end
    if (end_.detected && (markings_.size() > static_cast<size_t>(NUM_MIN_MARKINGS))) {
      Eigen::Vector3d map_P_end = map_T_world * end_.pose.translation();
      if (map_P_end.y() > marking_guess.translation().y()) {  // y negative here
        spawn_no_new_markings_ = true;
        return;
      }
    }

    markings_.push_back(std::make_shared<MapPose>(marking_guess));
  }
}

std::map<int, ImagePoint> ParkingLot::scanParkingSlot(const cv::Mat &img,
                                                      const std::map<int, ScanLine> &scan_lines) {
  std::map<int, ImagePoint> image_points;

  const float threshold =
      static_cast<float>(parameters_ptr_->getParam(GRADIENT_DETECTION_THLD));
  const unsigned int range_length = static_cast<unsigned int>(
      parameters_ptr_->getParam(STEP_REFERENCE_FUNCTION_LENGTH));

  const std::vector<float> reference_function =
      step_detection::createReferenceFunction(range_length);

  for (const auto &scan_line_pair : scan_lines) {
    auto feature_points = step_detection::detectStep(
        img, scan_line_pair.second, reference_function, threshold, false);

    // assume that feature_points are sorted

    if (feature_points.empty()) {
      image_points.emplace(scan_line_pair.first, scan_line_pair.second.end);
    } else {
      image_points.emplace(scan_line_pair.first, feature_points.front());
    }
  }
  return image_points;
}

bool ParkingLot::startDetected() const { return start_.detected; }

void ParkingLot::setStart(const WorldPose &pose) {
  const double MAX_ALLOWED_DEVIATION =
      parameters_ptr_->getParam(START_POSE_MAX_ALLOWED_DEVIATION);

  // check for false detections
  if (start_.detected) {
    if ((pose.translation() - start_.pose.translation()).norm() > MAX_ALLOWED_DEVIATION) {
      ROS_WARN("False parking start detected. Skipping...");
      return;
    }
  }

  start_.detected = true;
  start_.pose = pose;

  const cv::Point2f start_P_vehicle = toCV(Eigen::Vector2d(
      (world_coordinates_helper_->getTransform().inverse() * pose).translation().head<2>()));

  start_pose_changed_ = isInFieldOfView(start_P_vehicle);

  ROS_DEBUG(
      "start_point_in_vehicle coordinates is (%f,%f), start_pose_changed is %d",
      start_P_vehicle.x,
      start_P_vehicle.y,
      start_pose_changed_);
}

void ParkingLot::setEnd(const WorldPose &pose) {
  end_.detected = true;
  end_.pose = pose;
}

const WorldPose ParkingLot::mapPose() const { return world_T_pp_map_; }

void ParkingLot::reset() {
  // reset conditional variables
  start_.detected = false;
  end_.detected = false;
  start_pose_changed_ = false;

  // clear containers
  markings_.clear();
  parking_spots_.clear();
  spawn_no_new_markings_ = false;

  world_T_pp_map_ = Eigen::Affine3d::Identity();
}

void ParkingLot::scanParkingSlots(const cv::Mat &img,
                                  const common::DynamicPolynomial &left_lane_polynomial) {
  const Eigen::Affine3d world_T_vehicle = world_coordinates_helper_->getTransform();

  Eigen::Affine3d pp_map_T_vehicle = world_T_pp_map_.inverse() * world_T_vehicle;

  const auto vertical_angle_limits = computeAngleField(left_lane_polynomial);

  //  const auto polynomial_points_map =
  //  common::make_eigen_vector(polynomial_points |
  //  eigen_transformed(pp_map_T_vehicle));

  for (auto &parking_spot : parking_spots_) {
    auto scan_lines_map = createSpotScanLines(parking_spot, img.size());
    auto step_points = scanParkingSlot(img, scan_lines_map);

    ObservationPoints observation_points;
    for (const auto &scan_line_pair : scan_lines_map) {
      const int &i = scan_line_pair.first;
      const auto &step_point = step_points[i];
      const auto &scan_line = scan_lines_map[i];

      ObservationPoint observation_point;
      if (step_point == scan_line.end) {
        observation_point.type = ObservationPoint::Type::VISIBILITY_LIMIT;
      } else {
        observation_point.type = ObservationPoint::Type::OBSTACLE;
      }
      observation_point.point =
          pp_map_T_vehicle * camera_transformation_->transformImageToGround(step_point);

      observation_points.insert({i, observation_point});
    }
    parking_spot.updateObservation(observation_points, vertical_angle_limits, pp_map_T_vehicle);
  }
}

bool ParkingLot::trimToVisibilityLimit(WorldScanLine *const scan_line,
                                       const ParkingSpot &reference_parking_spot,
                                       bool skip_first) {
  assert(scan_line);
  using Line = Eigen::Hyperplane<double, 2>;

  // keep a safety margin to obstacle
  const double OBSTACLE_SAFETY_MARGIN =
      parameters_ptr_->getParam(SPOT_SCAN_LINE_OBSTACLE_SAFETY_MARGIN);

  auto reversed_spots = boost::adaptors::reverse(parking_spots_);
  auto parking_spot_it = boost::find(reversed_spots, reference_parking_spot);

  if (parking_spot_it == reversed_spots.end())
    return true;

  // skip the first parking spot, if wanted
  if (skip_first) {
    parking_spot_it++;
  }

  // compute view limit as a line
  Eigen::Affine3d camera_T_vehicle;
  camera_T_vehicle.translation() = camera_transformation_->getTranslationVector();
  camera_T_vehicle.linear() = camera_transformation_->getRotationMatrix();

  Eigen::Vector3d world_P_camera =
      (world_coordinates_helper_->getTransform() * camera_T_vehicle.inverse()).translation();

  for (; parking_spot_it != reversed_spots.end(); parking_spot_it++) {
    if (parking_spot_it->occupationState() == OccupationState::X) {
      //      ROS_DEBUG("Skipping spot #%d because it is a spot marked with an
      //      X",
      //                parking_spot_it->id());
      continue;
    }
    if (parking_spot_it->occupationState() != OccupationState::OCCUPIED) {
      //      ROS_DEBUG(
      //          "Skipping spot #%d because is does not contain any obstacle
      //          points",
      //          parking_spot_it->id());
      continue;
    }

    auto world_P_obstacle_point =
        world_T_pp_map_ * (parking_spot_it->rightMostObstaclePoint() -
                           Eigen::Vector3d{OBSTACLE_SAFETY_MARGIN, 0, 0});

    auto reference_line =
        Line::Through(to2D(world_P_camera), to2D(world_P_obstacle_point));

    // check if there is an intersection
    auto intersection = reference_line.intersection(
        Line::Through(scan_line->start.head<2>(), scan_line->end.head<2>()));

    // scalar product which is positive if both
    // vectors are pointing into the same direction or negative otherwise
    double sp = (scan_line->start.head<2>() - intersection).transpose() *
                (scan_line->end.head<2>() - intersection);

    if (sp < 0) {
      Eigen::Vector2d line_direction = (scan_line->end - scan_line->start).head<2>();
      line_direction /= line_direction.norm();

      // trim the end a bit
      scan_line->end.head<2>() = intersection - line_direction * OBSTACLE_SAFETY_MARGIN;
    } else {
      // if there is no intersection between start and end,
      // the scan line could also be fully occluded by an obstacle
      // (-> on lefthandside of reference_line)
      Eigen::Affine3d vehicle_T_world =
          world_coordinates_helper_->getTransform().inverse();

      Eigen::Vector2d obstacle_vec =
          (vehicle_T_world * world_P_obstacle_point).head<2>();
      Eigen::Vector2d scan_line_vec = (vehicle_T_world * scan_line->start).head<2>();

      double test_angle = std::atan2(scan_line_vec(1), scan_line_vec(0));
      double obstacle_angle = std::atan2(obstacle_vec(1), obstacle_vec(0));

      // scan line is on left side of reference line?
      if (test_angle > obstacle_angle) {
        return false;
      }
    }

    // only for first occupied parking lot before reference_parking_spot
    // if this is not sufficient, remove this line
    // return;
  }

  return true;
}

std::map<int, ScanLine> ParkingLot::createSpotScanLines(const ParkingSpot &parking_spot,
                                                        const cv::Size &img_size) {
  std::map<int, ScanLine> scan_lines;

  const int N_SCAN_LINES = parameters_ptr_->getParam(N_SCAN_LINES_PER_SPOT);
  const double DEPTH = parameters_ptr_->getParam(PARKING_SPOT_DEPTH);

  const double SPOT_PADDING_X = parameters_ptr_->getParam(SPOT_SCAN_LINE_PADDING_X);
  const double SPOT_PADDING_Y = parameters_ptr_->getParam(SPOT_SCAN_LINE_PADDING_X);

  const int IMG_PADDING = parameters_ptr_->getParam(IMAGE_PADDING);

  const Eigen::Affine3d left_T_right =
      parking_spot.leftEntrance().inverse() * parking_spot.rightEntrance();

  const Eigen::Affine3d vehicle_T_world =
      world_coordinates_helper_->getTransform().inverse();

  const double L = left_T_right.translation().norm() - 2 * SPOT_PADDING_Y;

  const Eigen::Affine3d world_T_p_spot = world_T_pp_map_ * parking_spot.leftEntrance();
  for (int i = 1; i <= N_SCAN_LINES; i++) {
    const double dy = L * static_cast<double>(i) / (N_SCAN_LINES + 1) + SPOT_PADDING_Y;

    WorldScanLine line =
        world_T_p_spot *
        WorldScanLine({SPOT_PADDING_X, -dy, 0}, {DEPTH - SPOT_PADDING_X, -dy, 0});

    if (!trimToVisibilityLimit(&line, parking_spot)) {
      continue;
    }

    ScanLine scan_line =
        transformGroundToImage(camera_transformation_, vehicle_T_world * line);

    if (ScanLine::clip(
            cv::Rect(IMG_PADDING, IMG_PADDING, img_size.width - IMG_PADDING, img_size.height - IMG_PADDING),
            scan_line)) {
      scan_lines.insert({i, scan_line});
    }
  }
  return scan_lines;
}

bool isInFrontOfVehicle(const VehicleScanLine &l) {
  return l.start.x() > 0.0 && l.end.x() > 0.0;
}

ScanLines ParkingLot::createMarkingScanLines(const int marking_idx, const cv::Size &image_size) {
  if (parking_spots_.empty())
    return {};

  ScanLines scan_lines;

  const auto &marking = *markings_[marking_idx];

  const auto &vehicle_T_world = world_coordinates_helper_->getTransform().inverse();

  const Eigen::Affine3d world_T_marking = world_T_pp_map_ * marking;

  // if first marking is aimed to be updated, generate scanline that's slightly
  // longer because of possible errors in parking_lot-geometry
  const double MARKING_SCAN_LINE_WIDTH_2 =
      marking_idx == 0 ? parameters_ptr_->getParam(MARKING_SCAN_LINE_WIDTH) / 2.0 +
                             parameters_ptr_->getParam(MARKING_WIDTH)
                       : parameters_ptr_->getParam(MARKING_SCAN_LINE_WIDTH) / 2.0;

  // create scan lines into the known marking direction
  const double SPOT_DEPTH = parameters_ptr_->getParam(PARKING_SPOT_DEPTH);
  const double STEP_WIDTH = parameters_ptr_->getParam(MARKING_SCAN_STEP_WIDTH);
  const double PADDING_FRONT = parameters_ptr_->getParam(MARKKING_SCAN_LINE_PADDING_FRONT);
  const double PADDING_BACK = parameters_ptr_->getParam(MARKKING_SCAN_LINE_PADDING_BACK);

  for (double s = STEP_WIDTH + PADDING_FRONT; s < SPOT_DEPTH - STEP_WIDTH - PADDING_BACK;
       s += STEP_WIDTH) {
    // scan line points in world coordinates
    // move along marking

    WorldScanLine line =
        world_T_marking * WorldScanLine({s, -MARKING_SCAN_LINE_WIDTH_2, 0},
                                        {s, MARKING_SCAN_LINE_WIDTH_2, 0});

    if (!isInFrontOfVehicle(vehicle_T_world * line)) {
      continue;
    }

    if (marking_idx != (static_cast<int>(markings_.size()) - 1) && marking_idx >= 1) {
      if (!trimToVisibilityLimit(&line, parking_spots_[marking_idx - 1], false)) {
        break;  // because other scan lines won't be visible as well
      }
    }

    ScanLine scan_line =
        transformGroundToImage(camera_transformation_, vehicle_T_world * line);

    if (ScanLine::clip(image_size, scan_line)) {
      scan_lines.push_back(scan_line);
    }
  }

  return scan_lines;
}

std::pair<double, double> ParkingLot::computeAngleField(const common::DynamicPolynomial &left_lane_polynomial) const {

  const Eigen::Affine3d vehicle_T_pp_map =
      world_coordinates_helper_->getTransform().inverse() * world_T_pp_map_;
  // compute minimal and maximal discretization_point
  const auto returnTranslation = [](const auto &ptr) {
    return ptr->translation();
  };

  const auto onlyPositive = [](double p) { return p > 0.0; };

  const auto discretization_limits =
      common::minmax_element(markings_ | transformed(std::cref(returnTranslation)) |
                             eigen_transformed(vehicle_T_pp_map) |
                             common::x_values | filtered(std::cref(onlyPositive)));

  if (discretization_limits.first == discretization_limits.second) {
    return {0, 0};
  }

  ROS_DEBUG("discretization_limits are: %f,%f",
            *discretization_limits.first,
            *discretization_limits.second);
  const double step = 0.05;

  if (*discretization_limits.second - *discretization_limits.first <= step) {
    return std::pair<double, double>(0.0, 0.0);
  }

  // discretize
  const common::DiscretizationParams params{
      *discretization_limits.first, *discretization_limits.second, step};
  VehiclePoints polynomial_points;
  common::discretisize(left_lane_polynomial, params, &polynomial_points);

  // transformation of a pair of vertically shifted points (vehicle frame) to
  // homogeneous image coordinates
  const auto toVerticalPair = [this](const auto &p) {
    return std::pair<Eigen::Vector3d, Eigen::Vector3d>(
        common::toHomogenous(camera_transformation_->transformGroundToImageExact(p)),
        common::toHomogenous(camera_transformation_->transformVehicleToImageExact(
            VehiclePoint{p.x(), p.y(), 0.2})));
  };
  const auto toNormalPoint = [](const auto &p) {
    return std::pair<ImagePointExact, ImagePointExact>(
        ImagePointExact{p.first.x(), p.first.y()} / p.first.z(),
        ImagePointExact{p.second.x(), p.second.y()} / p.second.z());
  };
  const auto toAngles = [](const auto &p) {
    return common::getAngle(
        ImagePointExact::UnitX(),
        common::ensureSameOrientation(ImagePointExact{p.first - p.second},
                                      ImagePointExact::UnitY()));
  };

  // compute angle field
  const auto angle_limits = common::minmax_element(
      polynomial_points | transformed(std::cref(toVerticalPair)) |
      transformed(std::cref(toNormalPoint)) | transformed(std::cref(toAngles)));

  ROS_DEBUG("angles_range is : (%f,%f)", *angle_limits.first, *angle_limits.second);

  return std::pair<double, double>(*angle_limits.first, *angle_limits.second);
}

ParkingLot::MarkingFeaturePoints ParkingLot::scanMarking(const cv::Mat &img,
                                                         const ScanLines &scan_lines,
                                                         bool scan_from_left,
                                                         bool scan_from_right) const {
  const float THRESHOLD =
      static_cast<float>(parameters_ptr_->getParam(GRADIENT_DETECTION_THLD));
  const unsigned int RANGE_LENGTH = static_cast<unsigned int>(
      parameters_ptr_->getParam(STEP_REFERENCE_FUNCTION_LENGTH));

  const std::vector<float> rising_step_function =
      step_detection::createReferenceFunction(RANGE_LENGTH);

  std::vector<float> falling_step_function =
      step_detection::invertReferenceFunction(rising_step_function);

  Eigen::Affine3d map_T_world = world_T_pp_map_.inverse();
  Eigen::Affine3d map_T_vehicle = map_T_world * world_coordinates_helper_->getTransform();

  const auto detectMarking = [&img, &THRESHOLD, &map_T_vehicle, this](
                                 const ScanLine &scan_line, const auto &step_function) {
    const auto image_feature_points =
        step_detection::detectStep(img, scan_line, step_function, THRESHOLD, false);

    MapPoints map_feature_points;
    camera_transformation_->transformImageToGround(image_feature_points, &map_feature_points);
    for (auto &map_feature_point : map_feature_points) {
      map_feature_point = map_T_vehicle * map_feature_point;
    }

    return map_feature_points;
  };

  MarkingFeaturePoints feature_points;

  for (const ScanLine &sl : scan_lines) {
    if (scan_from_left) {
      boost::push_back(feature_points.left, detectMarking(sl, rising_step_function));
    }

    if (scan_from_right) {
      boost::push_back(feature_points.right, detectMarking(sl, falling_step_function));
    }
  }

  return feature_points;
}

Eigen::Vector3d ParkingLot::intersectMarkingWithPolynomial(
    const MapPose &marking, const common::DynamicPolynomial &left_lane_polynomial) {
  const auto &vehicle_T_world = world_coordinates_helper_->getTransform().inverse();
  const auto vehicle_T_map = vehicle_T_world * world_T_pp_map_;

  const Eigen::Affine3d vehicle_T_marking = vehicle_T_map * marking;
  // vector to be modified
  Eigen::Vector3d vehicle_P_marking = vehicle_T_marking.translation();

  const double m = std::atan(common::toYaw(vehicle_T_marking.rotation()));
  // c = y - m*x for a given point
  const double c = vehicle_P_marking.y() - m * vehicle_P_marking.x();

  const common::LinearPolynomiald marking_line = {c, m};

  const double EPSILON = parameters_ptr_->getParam(NEWTON_EPSILON);
  const int MAX_ITERATIONS = parameters_ptr_->getParam(NEWTON_MAX_ITERATIONS);

  const double x_lf = utils::findRoot(left_lane_polynomial - marking_line,
                                      EPSILON,
                                      MAX_ITERATIONS,
                                      vehicle_P_marking.x());
  vehicle_P_marking.head<2>() = common::point(left_lane_polynomial, x_lf);
  return vehicle_T_map.inverse() * vehicle_P_marking;
}

void ParkingLot::projectMarkingToPolynomial(MapPose &marking,
                                            const common::DynamicPolynomial &left_lane_polynomial) {
  marking.translation().x() =
      intersectMarkingWithPolynomial(marking, left_lane_polynomial).x();
}

bool ParkingLot::isInFieldOfView(const cv::Point2f &p) const {
  return cv::pointPolygonTest(toInputArray(field_of_vision_), p, false) > 0;
}

}  // namespace perpendicular_parking
