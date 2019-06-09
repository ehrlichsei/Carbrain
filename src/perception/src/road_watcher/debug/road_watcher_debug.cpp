#include "road_watcher_debug.h"
#include "distinct_colors.h"
#include "opencv_eigen_conversions.h"

RoadWatcherDebug::RoadWatcherDebug(RoadWatcher &&road_watcher, cv::Mat *debug_image_ptr)
    : RoadWatcher(std::move(road_watcher)), img_debug_(debug_image_ptr) {}

void RoadWatcherDebug::scanRoadAndFindClusters(const cv::Mat &img_gray,
                                               const LineVehiclePoints &points,
                                               std::vector<FeaturePointCluster> &clusters,
                                               common::DynamicPolynomial &middle_polynomial) const {

  RoadWatcher::scanRoadAndFindClusters(img_gray, points, clusters, middle_polynomial);
  // visualize(clusters);
}

ImagePoints RoadWatcherDebug::apply1dGradientDetector(const cv::Mat &img,
                                                      const ScanLines &scan_lines) const {
  ImagePoints feature_points = RoadWatcher::apply1dGradientDetector(img, scan_lines);
  //  visualizeFeaturePoints(feature_points);
  return feature_points;
}

ScanLines RoadWatcherDebug::createScanLineGrid(const LineVehiclePoints &points,
                                               const common::DynamicPolynomial &middle_polynomial) const {
  const ScanLines scan_lines = RoadWatcher::createScanLineGrid(points, middle_polynomial);
  visualize(scan_lines);
  return scan_lines;
}

VehiclePoints RoadWatcherDebug::allPointToMiddle(LineVehiclePoints points) const {
  const VehiclePoints middle_points = RoadWatcher::allPointToMiddle(std::move(points));
  for (const VehiclePoint &vp : middle_points) {
    const auto ip = cam_transform_->transformGroundToImage(vp);
    cv::circle(*img_debug_, toCV(ip), 5, cv::Scalar(0, 255, 0), 1);
  }
  return middle_points;
}

void RoadWatcherDebug::visualize(const ScanLines &scan_lines) const {
  for (const ScanLine &scan_line : scan_lines) {
    cv::line(
        *img_debug_, toCV(scan_line.start), toCV(scan_line.end), cv::Scalar(255, 0, 0, 10), 1);
  }
}

void RoadWatcherDebug::visualize(const ImagePoints &feature_points,
                                 const cv::Scalar &color) const {
  for (const ImagePoint &feature_point : feature_points) {
    cv::circle(*img_debug_, toCV(feature_point), 3, color, 1);
  }
}

void RoadWatcherDebug::visualize(const std::vector<FeaturePointCluster> &clusters) const {
  //  const auto colors = perception::nDistinctColors(clusters.size());
  for (const auto &cluster : clusters) {
    visualize(cluster.feature_points_img, cv::Scalar(0, 0, 255));
  }
}

void RoadWatcherDebug::drawBoundingRectAroundFeaturePoints(const common::Vector2iVector &feature_points) const {
  if (feature_points.size() < 2) {
    return;
  }
  const cv::Rect bounding_rect = cv::boundingRect(toCV(feature_points));
  cv::rectangle(*img_debug_, bounding_rect, CV_RGB(0, 255, 0), 1);
}
