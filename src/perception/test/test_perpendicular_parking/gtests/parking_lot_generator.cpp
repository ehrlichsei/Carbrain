#include "parking_lot_generator.h"
#include "../src/perpendicular_parking/parking_lot.h"
#include "opencv_eigen_conversions.h"
#include "../../common/include/common/polynomialfit.h"

THIRD_PARTY_HEADERS_BEGIN
#include <boost/range/algorithm/transform.hpp>
#include <opencv2/opencv.hpp>
THIRD_PARTY_HEADERS_END

namespace perpendicular_parking {
namespace testing {

const ObstacleSize ParkingLotGenerator::OBSTACLE_SIZE_SMALL = {0.1, 0.1, 0.1};
const ObstacleSize ParkingLotGenerator::OBSTACLE_SIZE_MEDIUM = {0.2, 0.3, 0.18};
const ObstacleSize ParkingLotGenerator::OBSTACLE_SIZE_LARGE = {0.4, 0.46, 0.24};

ParkingLotGenerator::ParkingLotGenerator(const std::vector<GeneratedOccupationState> &occupation_states,
                                         const double parking_start_line_angle_in_radian,
                                         const double parking_slot_width_in_m,
                                         const double parking_slot_depth_in_m)
    : occupation_states_(occupation_states),
      parking_start_line_angle_in_radian_(parking_start_line_angle_in_radian),
      parking_slot_width_in_m_(parking_slot_width_in_m),
      parking_slot_depth_in_m_(parking_slot_depth_in_m) {
  number_of_parking_slots_ = occupation_states.size();
  // width in x direction of the parking start / stop section (they are
  // expected have the same size and angle)
  const auto start_stop_section_in_m =
      parking_slot_depth_in_m_ / tan(parking_start_line_angle_in_radian_);
  // praking_lot_length_in_m (including all spots and start and stop section)
  const auto praking_lot_length_in_m =
      2 * start_stop_section_in_m + number_of_parking_slots_ * parking_slot_width_in_m_;


  // generate all vertices of the different shapes needed to draw the parking
  // lot
  left_lane_points_.reserve(2);
  left_lane_points_ = {ParkingLotPoint(0.0, 0.0, 0.0),
                       ParkingLotPoint(praking_lot_length_in_m, 0.0, 0.0)};


  parking_start_line_points_.reserve(2);
  parking_start_line_points_ = {
      ParkingLotPoint(0.0, 0.0, 0.0),
      ParkingLotPoint(start_stop_section_in_m, parking_slot_depth_in_m_, 0.0)};

  parking_spots_points_.reserve(4 * number_of_parking_slots_);
  for (std::size_t i = 0; i < number_of_parking_slots_; i++) {

    parking_spots_points_.insert(
        parking_spots_points_.end(),
        {// top right corner
         ParkingLotPoint(start_stop_section_in_m + (i + 1) * parking_slot_width_in_m_, parking_slot_depth_in_m_, 0.0),
         // bottom right corner
         ParkingLotPoint(start_stop_section_in_m + (i + 1) * parking_slot_width_in_m_, 0.0, 0.0),
         // bottom left corner
         ParkingLotPoint(start_stop_section_in_m + i * parking_slot_width_in_m_, 0.0, 0.0),
         // top left corner
         ParkingLotPoint(start_stop_section_in_m + i * parking_slot_width_in_m_,
                         parking_slot_depth_in_m_,
                         0.0)});
  }

  parking_stop_line_points_.reserve(2);
  parking_stop_line_points_ = {
      ParkingLotPoint(start_stop_section_in_m + number_of_parking_slots_ * parking_slot_width_in_m_,
                      parking_slot_depth_in_m_,
                      0.0),
      ParkingLotPoint(start_stop_section_in_m + number_of_parking_slots_ * parking_slot_width_in_m_ +
                          start_stop_section_in_m,
                      0.0,
                      0.0)};

  // create ParkingStart and parkingEnd Pose in ParkingLotCoordinates
  parking_start_pose_ = Eigen::Affine3d::Identity();
  const Eigen::AngleAxisd angle_axis_start = Eigen::AngleAxisd(
      parking_start_line_angle_in_radian_, Eigen::Vector3d::UnitZ());
  parking_start_pose_.rotate(angle_axis_start);

  parking_end_pose_ = Eigen::Affine3d::Identity();
  const Eigen::AngleAxisd angle_axis_stop = Eigen::AngleAxisd(
      M_PI - parking_start_line_angle_in_radian, Eigen::Vector3d::UnitZ());
  parking_end_pose_.rotate(angle_axis_stop);

  parking_end_pose_.translation()(0) =
      start_stop_section_in_m +
      number_of_parking_slots_ * parking_slot_width_in_m_ + start_stop_section_in_m;
}

void ParkingLotGenerator::getLeftLanePoints(ParkingLotPoints &points,
                                            const std::size_t number_of_points) const {
  ParkingLotPoint v =
      (left_lane_points_[1] - left_lane_points_[0]) / (number_of_points - 1.0);
  points.reserve(number_of_points);
  for (std::size_t i = 0; i < number_of_points; i++) {
    points.emplace_back(left_lane_points_[0] + i * v);
  }
}

common::DynamicPolynomial ParkingLotGenerator::getLeftLanePolynom(
    const Eigen::Affine3d &parkinglot_to_vehicle_transform,
    const int number_of_left_lane_points) const {
  ParkingLotPoints points;
  getLeftLanePoints(points, number_of_left_lane_points);
  VehiclePoints vps;
  vps.reserve(number_of_left_lane_points);
  for (const auto &point : points) {
    VehiclePoint vp = parkinglot_to_vehicle_transform * point;
    vps.emplace_back(vp);
  }
  return common::fitToPoints(vps, POLYNOMIAL_DEGREE_LINE);
}

cv::Point ParkingLotGenerator::parkingLotPointToCvImagePoint(
    const ParkingLotPoint &parkinglot_point,
    const Eigen::Affine3d &parkinglot_to_vehicle_transform,
    const common::CameraTransformation &camera_transformation) const {
  return toCV(camera_transformation.transformVehicleToImage(
      parkinglot_to_vehicle_transform * parkinglot_point));
}

void ParkingLotGenerator::drawConcatenatedLine(cv::Mat &img,
                                               const Eigen::Affine3d &parkinglot_to_vehicle_transform,
                                               const common::CameraTransformation &camera_transformation,
                                               const ParkingLotPoints &parking_lot_points,
                                               const bool shapeClosed) const {
  cv::Point2i image_points[2];
  cv::Point2i first_point;
  const int thickness = 2;
  const unsigned N = parking_lot_points.size();
  for (unsigned i = 0; i < N; i++) {
    image_points[i % 2] = parkingLotPointToCvImagePoint(
        parking_lot_points[i], parkinglot_to_vehicle_transform, camera_transformation);
    if (i == 0) {
      first_point = image_points[0];
    } else {  // i>0
      cv::line(img, image_points[(i - 1) % 2], image_points[i % 2], 255, thickness, 16);
    }
  }
  if (shapeClosed) {
    // connect last point with first point
    cv::line(img, image_points[(N - 1) % 2], first_point, 255, thickness, 16);
  }
}

void ParkingLotGenerator::drawPolygon(cv::Mat &img,
                                      const Eigen::Affine3d &parkinglot_to_vehicle_transform,
                                      const common::CameraTransformation &camera_transformation,
                                      const ParkingLotPoints &parking_lot_points) const {
  const std::size_t N = parking_lot_points.size();
  std::vector<cv::Point2i> image_points;
  image_points.reserve(N);
  const auto parkingLotPointToCV =
      [&parkinglot_to_vehicle_transform, &camera_transformation](const auto &parking_lot_point) {
        return toCV(camera_transformation.transformVehicleToImage(
            parkinglot_to_vehicle_transform * parking_lot_point));
      };
  boost::range::transform(
      parking_lot_points, std::back_inserter(image_points), parkingLotPointToCV);
  cv::fillConvexPoly(img, image_points.data(), N, 255);
}

void ParkingLotGenerator::drawObstacle(cv::Mat &img,
                                       const Eigen::Affine3d &parkinglot_to_vehicle_transform,
                                       const common::CameraTransformation &camera_transformation,
                                       const ObstacleSize &obstacle_size,
                                       const ParkingLotPoint &obstacle_center_point) const {
  // draw top
  ParkingLotPoints plps{
      WorldPoint((obstacle_center_point(0) + obstacle_size.width_in_m / 2),
                 (obstacle_center_point(1) + obstacle_size.length_in_m / 2),
                 0.0),
      WorldPoint(obstacle_center_point(0) + obstacle_size.width_in_m / 2,
                 obstacle_center_point(1) - obstacle_size.length_in_m / 2,
                 0.0),
      WorldPoint(obstacle_center_point(0) - obstacle_size.width_in_m / 2,
                 obstacle_center_point(1) - obstacle_size.length_in_m / 2,
                 0.0),
      WorldPoint(obstacle_center_point(0) - obstacle_size.width_in_m / 2,
                 obstacle_center_point(1) + obstacle_size.length_in_m / 2,
                 0.0)};
  drawPolygon(img, parkinglot_to_vehicle_transform, camera_transformation, plps);

  // draw front side
  plps = {WorldPoint(obstacle_center_point(0) - obstacle_size.width_in_m / 2,
                     obstacle_center_point(1) + obstacle_size.length_in_m / 2,
                     0.0),
          WorldPoint(obstacle_center_point(0) + obstacle_size.width_in_m / 2,
                     obstacle_center_point(1) + obstacle_size.length_in_m / 2,
                     0.0),
          WorldPoint(obstacle_center_point(0) + obstacle_size.width_in_m / 2,
                     obstacle_center_point(1) + obstacle_size.length_in_m / 2,
                     obstacle_size.height_in_m),
          WorldPoint(obstacle_center_point(0) - obstacle_size.width_in_m / 2,
                     obstacle_center_point(1) + obstacle_size.length_in_m / 2,
                     obstacle_size.height_in_m)};
  drawPolygon(img, parkinglot_to_vehicle_transform, camera_transformation, plps);


  // draw back side
  plps = {WorldPoint(obstacle_center_point(0) - obstacle_size.width_in_m / 2,
                     obstacle_center_point(1) - obstacle_size.length_in_m / 2,
                     0.0),
          WorldPoint(obstacle_center_point(0) + obstacle_size.width_in_m / 2,
                     obstacle_center_point(1) - obstacle_size.length_in_m / 2,
                     0.0),
          WorldPoint(obstacle_center_point(0) + obstacle_size.width_in_m / 2,
                     obstacle_center_point(1) - obstacle_size.length_in_m / 2,
                     obstacle_size.height_in_m),
          WorldPoint(obstacle_center_point(0) - obstacle_size.width_in_m / 2,
                     obstacle_center_point(1) - obstacle_size.length_in_m / 2,
                     obstacle_size.height_in_m)};
  drawPolygon(img, parkinglot_to_vehicle_transform, camera_transformation, plps);

  // draw left side
  plps = {WorldPoint(obstacle_center_point(0) - obstacle_size.width_in_m / 2,
                     obstacle_center_point(1) - obstacle_size.length_in_m / 2,
                     0.0),
          WorldPoint(obstacle_center_point(0) - obstacle_size.width_in_m / 2,
                     obstacle_center_point(1) + obstacle_size.length_in_m / 2,
                     0.0),
          WorldPoint(obstacle_center_point(0) - obstacle_size.width_in_m / 2,
                     obstacle_center_point(1) + obstacle_size.length_in_m / 2,
                     obstacle_size.height_in_m),
          WorldPoint(obstacle_center_point(0) - obstacle_size.width_in_m / 2,
                     obstacle_center_point(1) - obstacle_size.length_in_m / 2,
                     obstacle_size.height_in_m)};
  drawPolygon(img, parkinglot_to_vehicle_transform, camera_transformation, plps);



  // draw right side
  plps = {WorldPoint(obstacle_center_point(0) + obstacle_size.width_in_m / 2,
                     obstacle_center_point(1) - obstacle_size.length_in_m / 2,
                     0.0),
          WorldPoint(obstacle_center_point(0) + obstacle_size.width_in_m / 2,
                     obstacle_center_point(1) + obstacle_size.length_in_m / 2,
                     0.0),
          WorldPoint(obstacle_center_point(0) + obstacle_size.width_in_m / 2,
                     obstacle_center_point(1) + obstacle_size.length_in_m / 2,
                     obstacle_size.height_in_m),
          WorldPoint(obstacle_center_point(0) + obstacle_size.width_in_m / 2,
                     obstacle_center_point(1) - obstacle_size.length_in_m / 2,
                     obstacle_size.height_in_m)};
  drawPolygon(img, parkinglot_to_vehicle_transform, camera_transformation, plps);
}

void ParkingLotGenerator::drawParkingLot(cv::Mat &img,
                                         const Eigen::Affine3d &parkinglot_to_vehicle_transform,
                                         const common::CameraTransformation &camera_transformation) const {
  img.create(650, 1280, CV_8U);  // set image size to 1280x650 pixel
  img = cv::Scalar::all(0);      // set all pixel to be black

  // draw parking lot line by line
  drawConcatenatedLine(img, parkinglot_to_vehicle_transform, camera_transformation, left_lane_points_);
  drawConcatenatedLine(img, parkinglot_to_vehicle_transform, camera_transformation, parking_start_line_points_);
  bool drawShapeClosed = (number_of_parking_slots_ <= 1) ? true : false;
  drawConcatenatedLine(
      img, parkinglot_to_vehicle_transform, camera_transformation, parking_spots_points_, drawShapeClosed);
  drawConcatenatedLine(img, parkinglot_to_vehicle_transform, camera_transformation, parking_stop_line_points_);

  // draw occupation state of each slot
  for (std::size_t i = 0; i < number_of_parking_slots_; i++) {
    if (occupation_states_[i] == GeneratedOccupationState::FREE) {
      // noting to do
      continue;
    }
    const std::size_t index = 4 * i;

    if (occupation_states_[i] == GeneratedOccupationState::X) {
      // draw no parking area
      ParkingLotPoints temp{parking_spots_points_[index], parking_spots_points_[index + 2]};
      drawConcatenatedLine(img, parkinglot_to_vehicle_transform, camera_transformation, temp);
      temp[0] = parking_spots_points_[index + 1];
      temp[1] = parking_spots_points_[index + 3];
      drawConcatenatedLine(img, parkinglot_to_vehicle_transform, camera_transformation, temp);
    }
    if (occupation_states_[i] == GeneratedOccupationState::OBSTACLE_SMALL ||
        occupation_states_[i] == GeneratedOccupationState::OBSTACLE_MEDIUM ||
        occupation_states_[i] == GeneratedOccupationState::OBSTACLE_LARGE) {
      // also draw obstacle
      ParkingLotPoint top_right_corner = parking_spots_points_[index];
      ParkingLotPoint obstacle_center_point(
          top_right_corner(0) - parking_slot_width_in_m_ / 2,
          top_right_corner(1) - parking_slot_depth_in_m_ / 2,
          0.0);
      if (occupation_states_[i] == GeneratedOccupationState::OBSTACLE_SMALL) {
        drawObstacle(img, parkinglot_to_vehicle_transform, camera_transformation, OBSTACLE_SIZE_SMALL, obstacle_center_point);
      } else if (occupation_states_[i] == GeneratedOccupationState::OBSTACLE_MEDIUM) {
        drawObstacle(img,
                     parkinglot_to_vehicle_transform,
                     camera_transformation,
                     OBSTACLE_SIZE_MEDIUM,
                     obstacle_center_point);
      } else if (occupation_states_[i] == GeneratedOccupationState::OBSTACLE_LARGE) {
        drawObstacle(img, parkinglot_to_vehicle_transform, camera_transformation, OBSTACLE_SIZE_LARGE, obstacle_center_point);
      }
    }
  }
}

void ParkingLotGenerator::debugDrawPolynom(cv::Mat &debug_img,
                                           const common::DynamicPolynomial &left_lane_polynom,
                                           const common::CameraTransformation &camera_transformation,
                                           const cv::Scalar &color) const {
  const double step = 0.1;
  const int N = 10;
  const int thikness = 6;
  double x = 0;
  for (int i = 0; i < N; i++) {
    x += step;
    VehiclePoint vp(x, left_lane_polynom(x), 0);
    cv::Point2i image_point = toCV(camera_transformation.transformVehicleToImage(vp));
    cv::circle(debug_img, image_point, thikness, color, -1);
  }
}

void ParkingLotGenerator::debugPrintAllGeneratedPoints() {
  std::cout << "ParkingStartLine" << std::endl;
  for (std::size_t i = 0; i < parking_start_line_points_.size(); i++) {
    std::cout << parking_start_line_points_[i] << std::endl;
  }
  std::cout << "ParkingStopLine" << std::endl;
  for (std::size_t i = 0; i < parking_stop_line_points_.size(); i++) {
    std::cout << parking_stop_line_points_[i] << std::endl;
  }
  std::cout << "ParkingSpots" << std::endl;
  for (std::size_t i = 0; i < parking_spots_points_.size(); i++) {
    std::cout << parking_spots_points_[i] << std::endl;
  }
}

void ParkingLotGenerator::drawPoints(const ParkingLotPoints points,
                                     cv::Mat &debug_img,
                                     const common::CameraTransformation &camera_transformation,
                                     const Eigen::Affine3d &parkinglot_to_vehicle_transform,
                                     const cv::Scalar &color) const {
  for (const auto &point : points) {
    cv::Point2i image_point = parkingLotPointToCvImagePoint(
        point, parkinglot_to_vehicle_transform, camera_transformation);
    const int thickness = 6;
    cv::circle(debug_img, image_point, thickness, color, -1);
  }
}

void ParkingLotGenerator::debugDrawGeneratedPoints(cv::Mat &debug_img,
                                                   const common::CameraTransformation &camera_transformation,
                                                   const Eigen::Affine3d &parkinglot_to_vehicle_transform,
                                                   const cv::Scalar &color) const {

  drawPoints(parking_start_line_points_, debug_img, camera_transformation, parkinglot_to_vehicle_transform, color);
  drawPoints(parking_stop_line_points_, debug_img, camera_transformation, parkinglot_to_vehicle_transform, color);
  drawPoints(parking_spots_points_, debug_img, camera_transformation, parkinglot_to_vehicle_transform, color);
}

void ParkingLotGenerator::debugDrawPose(cv::Mat &debug_img,
                                        const VehiclePose &pose,
                                        const common::CameraTransformation &camera_transformation,
                                        const cv::Scalar &color,
                                        const int drawAxis) const {

  cv::Point2i image_point0 =
      toCV(camera_transformation.transformVehicleToImage(pose.translation()));
  cv::Point2i image_pointx =
      toCV(camera_transformation.transformVehicleToImage(pose * VehiclePoint(0.1, 0, 0)));
  cv::Point2i image_pointy =
      toCV(camera_transformation.transformVehicleToImage(pose * VehiclePoint(0, 0.1, 0)));
  cv::Point2i image_pointz =
      toCV(camera_transformation.transformVehicleToImage(pose * VehiclePoint(0, 0, 0.1)));

  const double tip_length = 0.1;
  const int thickness = 2;
  if (drawAxis >= 1) {
    cv::arrowedLine(debug_img, image_point0, image_pointx, color, thickness, 16, 0, tip_length);
  }
  if (drawAxis >= 2) {
    cv::arrowedLine(debug_img, image_point0, image_pointy, color, thickness, 16, 0, tip_length);
  }
  if (drawAxis >= 3) {
    cv::arrowedLine(debug_img, image_point0, image_pointz, color, thickness, 16, 0, tip_length);
  }
}


}  // namespace testing
}  // namespace perpendicular parking
