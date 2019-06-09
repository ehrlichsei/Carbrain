#include "parking_start_classifier_debug.h"
#include "../../utils/tf_helper_interface.h"

THIRD_PARTY_HEADERS_BEGIN
#include <boost/range/algorithm_ext/push_back.hpp>
THIRD_PARTY_HEADERS_END

#include "common/eigen_utils.h"
#include "common/polynomial_utils.h"

namespace perpendicular_parking {
ParkingStartClassifierDebug::ParkingStartClassifierDebug(ParkingStartClassifier&& parking_start_classifier)
    : ParkingStartClassifier(std::move(parking_start_classifier)) {}

Type ParkingStartClassifierDebug::classify(FeaturePointCluster& cluster,
                                           const common::DynamicPolynomial& left_lane_polynom,
                                           const ros::Time& timestamp) {
  img_size_ = cv::Rect{0, 0, 1280, 650};
  debug_scanlines_.clear();
  return ParkingStartClassifier::classify(cluster, left_lane_polynom, timestamp);
}

WorldPose ParkingStartClassifierDebug::startLinePose() const {
  const auto ret = ParkingStartClassifier::startLinePose();
  start_pose_ = ret;
  return ret;
}

ImagePoints ParkingStartClassifierDebug::startLine() const {
  // currently no line_points published
  return {};
}

FeaturePointClusters ParkingStartClassifierDebug::clusters() const {
  return points_clusters;
}

ImagePoints ParkingStartClassifierDebug::crucialPoints() const {
  ImagePoints crucial_points;
  boost::push_back(crucial_points, toLinePoints(start_pose_));
  //  const ImagePoint end_point_image =
  //      cam_transform_->transformGroundToImage(start_pose_vis.second);
  //  crucial_points.push_back(end_point_image);
  return crucial_points;
}

std::vector<common::DynamicPolynomial> ParkingStartClassifierDebug::fitLinesR(
    const std::size_t nr_lines, const VehiclePoints& points) const {
  const auto ret = ParkingStartClassifier::fitLinesR(nr_lines, points);
  for (const auto& line : ret) {
    const VehiclePoint start{to3D(common::point(line, 0.0))};
    const VehiclePoint end{to3D(common::point(line, 5.0))};

    ScanLine spot_line{cam_transform_->transformGroundToImage(start),
                       cam_transform_->transformGroundToImage(end)};
    ScanLine::clip(img_size_, spot_line);
    debug_scanlines_.emplace_back(spot_line);
  }

  return ret;
}

ImagePoints ParkingStartClassifierDebug::toLinePoints(const WorldPose& pose) const {
  const WorldPoint start_point_world = pose.translation();
  const WorldPoint end_point_world =
      start_point_world + pose.linear() * WorldPoint{0.5, 0, 0};
  const int steps = 10;
  const Eigen::Affine3d vehicle_T_world =
      world_coordinates_helper_->getTransform().inverse();
  const VehiclePoint end_point_vehicle = vehicle_T_world * end_point_world;
  const VehiclePoint start_point_vehicle = vehicle_T_world * start_point_world;
  const auto direction = common::ensureSameOrientation(
      end_point_vehicle - start_point_vehicle, VehiclePoint::UnitX());
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
}  // namespace perpendicular_parking
