#include "parking_end_classifier_debug.h"

THIRD_PARTY_HEADERS_BEGIN
#include <boost/range/algorithm.hpp>
#include <boost/range/algorithm_ext/push_back.hpp>
THIRD_PARTY_HEADERS_END

#include "common/eigen_utils.h"
#include "common/polynomial_utils.h"
#include "vehicle_scan_line.h"

namespace perpendicular_parking {
ParkingEndClassifierDebug::ParkingEndClassifierDebug(ParkingEndClassifier &&parking_end_classifier)
    : ParkingEndClassifier(std::move(parking_end_classifier)) {}

boost::optional<WorldPose> ParkingEndClassifierDebug::detectEnd(
    const ParkingSpotsConstRef &all_parking_spots,
    const cv::Mat &img,
    const ros::Time &timestamp,
    const common::DynamicPolynomial &left_lane_polynom,
    const LineVehiclePoints &lanes,
    const Eigen::Affine3d &map_T_world) {
  debug_scan_lines.clear();
  debug_feature_points.clear();
  debug_end_line_points.clear();
  debug_crucial_points.clear();
  debug_endline_clusters.clear();
  img_size_ = cv::Rect{0, 0, img.size().width, img.size().height};
  const boost::optional<WorldPose> end_line_pose = ParkingEndClassifier::detectEnd(
      all_parking_spots, img, timestamp, left_lane_polynom, lanes, map_T_world);
  if (end_line_pose) {
    ImagePoints end_line_img = toLinePoints(*end_line_pose);
    boost::push_back(debug_end_line_points, end_line_img);
  }
  //  if (safety_zone_footpoint_) {
  //    debug_crucial_points.push_back(cam_transform_->transformVehicleToImage(
  //        endline_zone_.boundary[0].in_vehicle));
  //    debug_crucial_points.push_back(cam_transform_->transformVehicleToImage(
  //        safety_zone_footpoint_->in_vehicle));
  //    //    debug_crucial_points.push_back(
  //    //
  //    cam_transform_->transformVehicleToImage(safety_zone_.boundary[1].in_vehicle));
  //  }
  return end_line_pose;
}

void ParkingEndClassifierDebug::reset() {
  ParkingEndClassifier::reset();
  debug_scan_lines.clear();
  debug_feature_points.clear();
  debug_end_line_points.clear();
  debug_crucial_points.clear();
  debug_endline_clusters.clear();
}

ImagePoints ParkingEndClassifierDebug::apply1DGradientDetector(const cv::Mat &img,
                                                               const ScanLines &scan_lines,
                                                               const bool all) const {
  ImagePoints ret = ParkingEndClassifier::apply1DGradientDetector(img, scan_lines, all);
  boost::push_back(debug_feature_points, ret);
  return ret;
}

ScanLines ParkingEndClassifierDebug::createScanLines(const ImagePoints &start_points,
                                                     const ImagePoints &end_points) const {
  const ScanLines ret = ParkingEndClassifier::createScanLines(start_points, end_points);

  boost::push_back(debug_scan_lines, ret);
  // boost::push_back(debug_feature_points, start_points);

  return ret;
}

ScanLines ParkingEndClassifierDebug::getParallelScanlines(const common::DynamicPolynomial &polynom,
                                                          const VehiclePoint &start_point,
                                                          const VehiclePoint &end_point) const {
  const auto ret =
      ParkingEndClassifier::getParallelScanlines(polynom, start_point, end_point);
  //  boost::push_back(debug_scan_lines, ret);
  return ret;
}

ImagePoints ParkingEndClassifierDebug::toLinePoints(const WorldPose &pose) {
  const WorldPoint start_point_world = pose.translation();
  const WorldPoint end_point_world =
      start_point_world + pose.linear() * WorldPoint{0.5, 0, 0};
  const int steps = 10;
  const Eigen::Affine3d vehicle_T_world =
      world_coordinates_helper_->getTransform().inverse();
  const VehiclePoint end_point_vehicle = vehicle_T_world * end_point_world;
  const VehiclePoint start_point_vehicle = vehicle_T_world * start_point_world;
  const VehiclePoint direction = common::ensureSameOrientation(
      end_point_vehicle - start_point_vehicle, -VehiclePoint::UnitX());
  const auto delta = direction / (steps - 1);
  VehiclePoints end_line_points_vehicle;
  end_line_points_vehicle.reserve(steps);
  for (int i = 0; i < steps; i++) {
    const VehiclePoint insert = start_point_vehicle + i * delta;
    end_line_points_vehicle.push_back(insert);
  }
  ImagePoints line_img;
  cam_transform_->transformGroundToImage(end_line_points_vehicle, &line_img);
  return line_img;
}

std::vector<common::DynamicPolynomial> ParkingEndClassifierDebug::fitLinesR(
    const std::size_t nr_lines, const VehiclePoints &points) const {
  const auto lines = ParkingEndClassifier::fitLinesR(nr_lines, points);

  for (const auto &line : lines) {

    ScanLine spot_line = transformGroundToImage(
        cam_transform_,
        VehicleScanLine(to3D(common::point(line, 0.0)), to3D(common::point(line, 5.0))));

    ScanLine::clip(img_size_, spot_line);
    debug_scan_lines.emplace_back(spot_line);
  }

  return lines;
}

}  // namespace perpendicular_parking
