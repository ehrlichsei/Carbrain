#include "parking_lot_debug.h"
#include <common/macros.h>

THIRD_PARTY_HEADERS_BEGIN
#include <boost/range/adaptor/filtered.hpp>
#include <boost/range/adaptor/transformed.hpp>
#include <boost/range/algorithm/transform.hpp>
#include <boost/range/algorithm_ext/erase.hpp>
#include <boost/range/algorithm_ext/push_back.hpp>
#include <ros/console.h>
THIRD_PARTY_HEADERS_END

#include <common/angle_conversions.h>
#include <common/best_score.h>
#include <common/camera_transformation.h>
#include <common/eigen_utils.h>

namespace perpendicular_parking {
ParkingLotDebug::ParkingLotDebug(ParkingLot &&parking_lot)
    : ParkingLot(std::move(parking_lot)) {}

ParkingSpotsConstRef ParkingLotDebug::freeParkingSpots() {
  return ParkingLot::freeParkingSpots();
}

ParkingSpotsConstRef ParkingLotDebug::getAllParkingSpots() const {
  return ParkingLot::allParkingSpots();
}

ScanLines ParkingLotDebug::getMarkingScanLines() {
  ScanLines scan_lines;

  if (parking_spots_.empty())
    return {};

  const cv::Size IMG_SIZE = current_img_.size();

  for (unsigned int i = 0; i < markings_.size(); ++i) {
    boost::push_back(scan_lines, createMarkingScanLines(i, IMG_SIZE));
  }
  return scan_lines;
}

ImagePoints ParkingLotDebug::getMarkingDetections() {
  ImagePoints points;

  Eigen::Affine3d vehicle_T_world = world_coordinates_helper_->getTransform().inverse();

  const Eigen::Affine3d vehicle_T_map = vehicle_T_world * world_T_pp_map_;

  auto mapToImage = [this, &vehicle_T_map](const auto &map_feature_point) {
    return camera_transformation_->transformGroundToImage(vehicle_T_map * map_feature_point);
  };

  const cv::Size IMG_SIZE = current_img_.size();

  for (unsigned int i = 0; i < markings_.size(); ++i) {
    auto scan_lines = createMarkingScanLines(i, IMG_SIZE);
    auto map_feature_points = scanMarking(current_img_, scan_lines, true, true);

    points.reserve(points.size() + map_feature_points.left.size() +
                   map_feature_points.right.size());
    boost::transform(map_feature_points.left, std::back_inserter(points), mapToImage);
    boost::transform(map_feature_points.right, std::back_inserter(points), mapToImage);
  }

  auto isInvalid = [&IMG_SIZE](const ImagePoint &image_point) {
    return image_point[0] < 0 || image_point[1] < 0 ||
           image_point[0] > IMG_SIZE.width || image_point[1] > IMG_SIZE.height;
  };

  boost::remove_erase_if(points, isInvalid);

  return points;
}

ScanLines ParkingLotDebug::getSpotScanLines() {
  ScanLines scan_lines;
  for (const auto &parking_spot : parking_spots_) {
    auto scan_line_map = createSpotScanLines(parking_spot, current_img_.size());

    scan_lines.reserve(scan_lines.size() + scan_line_map.size());
    for (const auto &scan_line_pair : scan_line_map) {
      scan_lines.push_back(scan_line_pair.second);
    }
  }
  return scan_lines;
}

ScanLines ParkingLotDebug::getSpotLines() const {
  if (parking_spots_.empty()) {
    return {};
  }
  ScanLines spot_lines;

  const Eigen::Affine3d vehicle_T_world =
      world_coordinates_helper_->getTransform().inverse();

  const Eigen::Affine3d vehicle_T_pp_map = vehicle_T_world * world_T_pp_map_;

  const cv::Rect size{0, 0, current_img_.size().width, current_img_.size().height};

  const auto spot =
      *common::max_score(parking_spots_, [](const auto &s) { return s.id(); });

  for (const auto &line : spot.detectedLines()) {
    ScanLine spot_line = transformGroundToImage(
        camera_transformation_,
        vehicle_T_pp_map *
            VehicleScanLine(to3D(line.pointAt(0.0)), to3D(line.pointAt(10.0))));
    ScanLine::clip(size, spot_line);
    spot_lines.emplace_back(spot_line);
  }

  return spot_lines;
}

ImagePoints ParkingLotDebug::getFeaturePoints() const {
  ImagePoints feature_points;

  Eigen::Affine3d vehicle_T_world = world_coordinates_helper_->getTransform().inverse();

  Eigen::Affine3d vehicle_T_map = vehicle_T_world * world_T_pp_map_;

  for (const auto &parking_spot : allParkingSpots()) {
    auto free_space = parking_spot.get().freeSpace();
    for (const auto &map_point : free_space) {
      feature_points.push_back(
          camera_transformation_->transformGroundToImage(vehicle_T_map * map_point));
    }
  }

  return feature_points;
}

void ParkingLotDebug::reset() {
  //  verticals.clear();
  ParkingLot::reset();
}

// std::vector<std::pair<double, ImagePointExact>>
// ParkingLotDebug::computeAngleField(
//    const common::polynomial::DynamicPolynomial &left_lane_polynomial) const {
//  verticals.clear();
//  const auto ret = ParkingLot::computeAngleField(left_lane_polynomial);
//  for (const auto &pair : ret) {
//    const ImagePointExact end =
//        pair.second +
//        to2D(Eigen::AngleAxisd{pair.first, Eigen::Vector3d::UnitZ()}
//                 .toRotationMatrix() *
//             (1000 * Eigen::Vector3d::UnitX()));
//    ScanLine actual{pair.second.cast<int>(), end.cast<int>()};
//    ScanLine::clip(
//        cv::Rect(0, 0, current_img_.size().width, current_img_.size().height),
//        actual);
//    verticals.push_back(actual);
//  }
//  return ret;
//}
}  // namespace perpendicular_parking
