#include "perpendicular_parking_node_debug.h"


#include "common/node_creation_makros.h"

#include "perpendicular_parking_debug.h"

namespace perpendicular_parking {

PerpendicularParkingNodeDebug::PerpendicularParkingNodeDebug(ros::NodeHandle &node_handle)
    : PerpendicularParkingNode(node_handle) {
  perpendicular_parking_ =
      std::make_unique<perpendicular_parking::PerpendicularParkingDebug>(
          std::move(*perpendicular_parking_));
}

void PerpendicularParkingNodeDebug::startModule() {
  PerpendicularParkingNode::startModule();
  // publishers
  parking_start_line_points_pub = node_handle_.advertise<perception_msgs::Pixels>(
      "parking_start_line_points", 1);
  parking_start_feature_points_pub = node_handle_.advertise<perception_msgs::Pixels>(
      "parking_start_feature_points", 1);
  parking_lot_feature_points_pub = node_handle_.advertise<perception_msgs::Pixels>(
      "parking_lot_feature_points", 1);
  parking_start_scan_lines_pub = node_handle_.advertise<perception_msgs::ScanLines>(
      "parking_start_scan_lines", 1);
  parking_lot_marking_scan_lines_pub = node_handle_.advertise<perception_msgs::ScanLines>(
      "parking_lot_marking_scan_lines", 1);
  parking_slot_scan_lines_pub = node_handle_.advertise<perception_msgs::ScanLines>(
      "parking_spot_scan_lines", 1);
  parking_start_clusters_pub = node_handle_.advertise<perception_msgs::PointsClusters>(
      "parking_start_clusters", 1);
  all_parking_spots_helper.advertise(node_handle_, "parking_spots");
  parking_start_crucial_points_pub = node_handle_.advertise<perception_msgs::Pixels>(
      "parking_start_crucial_points", 1);
  parking_lot_marking_detections_pub = node_handle_.advertise<perception_msgs::Pixels>(
      "parking_lot_marking_detections", 1);
  parking_end_line_points_pub =
      node_handle_.advertise<perception_msgs::Pixels>("parking_end_line_points", 1);
  parking_end_feature_points_pub = node_handle_.advertise<perception_msgs::Pixels>(
      "parking_end_feature_points", 1);
  parking_end_scan_lines_pub = node_handle_.advertise<perception_msgs::ScanLines>(
      "parking_end_scan_lines", 1);
  parking_end_clusters_pub = node_handle_.advertise<perception_msgs::PointsClusters>(
      "parking_end_clusters", 1);
  parking_end_crucial_points_pub = node_handle_.advertise<perception_msgs::Pixels>(
      "parking_end_crucial_points", 1);
  parking_spots_lines_pub =
      node_handle_.advertise<perception_msgs::ScanLines>("parking_spots_lines", 1);
  left_lane_polynom_pub =
      node_handle_.advertise<perception_msgs::Pixels>("left_lane_polynom", 1);
}

void PerpendicularParkingNodeDebug::stopModule() {
  PerpendicularParkingNode::stopModule();
  // publishers
  parking_start_line_points_pub.shutdown();
  parking_start_feature_points_pub.shutdown();
  parking_lot_feature_points_pub.shutdown();
  parking_start_scan_lines_pub.shutdown();
  all_parking_spots_helper.shutdown();
  parking_start_clusters_pub.shutdown();
  parking_lot_marking_scan_lines_pub.shutdown();
  parking_start_crucial_points_pub.shutdown();
  parking_end_line_points_pub.shutdown();
  parking_end_feature_points_pub.shutdown();
  parking_end_scan_lines_pub.shutdown();
  parking_end_crucial_points_pub.shutdown();
  parking_end_clusters_pub.shutdown();
  parking_spots_lines_pub.shutdown();
  left_lane_polynom_pub.shutdown();
}

void PerpendicularParkingNodeDebug::handleLeftLaneAndImage(
    const sensor_msgs::ImageConstPtr &image_raw_msg,
    const nav_msgs::PathConstPtr &left_lane_msg,
    const nav_msgs::PathConstPtr &middle_lane_msg,
    const nav_msgs::PathConstPtr &right_lane_msg,
    const nav_msgs::PathConstPtr &no_passing_lane_msg) {
  PerpendicularParkingNode::handleLeftLaneAndImage(
      image_raw_msg, left_lane_msg, middle_lane_msg, right_lane_msg, no_passing_lane_msg);
  const ros::Time stamp = image_raw_msg->header.stamp;
  createParkingSpotsMsg();
  all_parking_spots_helper.publishMessage(
      all_parking_spots_helper.generateAndClearMessage(stamp));

  auto perpendicular_parking_debug =
      dynamic_cast<PerpendicularParkingDebug *>(perpendicular_parking_.get());
  assert(perpendicular_parking_debug);

  const auto slp_msg =
      createPixelsMsgs(perpendicular_parking_debug->startLinePoints(), stamp);
  parking_start_line_points_pub.publish(slp_msg);
  const auto sfp_msg = createPixelsMsgs(
      perpendicular_parking_debug->parkingStartFeaturePoints(), stamp);
  parking_start_feature_points_pub.publish(sfp_msg);
  const auto pfp_msg =
      createPixelsMsgs(perpendicular_parking_debug->parkingLotFeaturePoints(), stamp);
  parking_lot_feature_points_pub.publish(pfp_msg);
  const auto ssl_msg =
      createScanLinesMsgs(perpendicular_parking_debug->parkingStartScanLines(), stamp);
  parking_start_scan_lines_pub.publish(ssl_msg);
  const auto pmsl_msg = createScanLinesMsgs(
      perpendicular_parking_debug->parkingLotMarkingScanLines(), stamp);
  parking_lot_marking_scan_lines_pub.publish(pmsl_msg);
  const auto pssl_msg =
      createScanLinesMsgs(perpendicular_parking_debug->parkingSlotsScanLines(), stamp);
  parking_slot_scan_lines_pub.publish(pssl_msg);
  const auto pscp_msg = createPixelsMsgs(
      perpendicular_parking_debug->parkingStartCrucialPoints(), stamp);
  parking_start_crucial_points_pub.publish(pscp_msg);
  const auto ps_clusters_msg = createPointsClustersMsg(
      perpendicular_parking_debug->startLinePointsClusters(), stamp);
  parking_start_clusters_pub.publish(ps_clusters_msg);

  const auto pmd_msg =
      createPixelsMsgs(perpendicular_parking_debug->markingDetections(), stamp);
  parking_lot_marking_detections_pub.publish(pmd_msg);
  // parking_end_classifier
  const auto pelp_msg =
      createPixelsMsgs(perpendicular_parking_debug->endLinesPoints(), stamp);
  parking_end_line_points_pub.publish(pelp_msg);
  const auto pefp_msg =
      createPixelsMsgs(perpendicular_parking_debug->parkingEndFeaturePoints(), stamp);
  parking_end_feature_points_pub.publish(pefp_msg);
  const auto pesl_msg =
      createScanLinesMsgs(perpendicular_parking_debug->parkingEndScanLines(), stamp);
  parking_end_scan_lines_pub.publish(pesl_msg);
  const auto pecp_msg =
      createPixelsMsgs(perpendicular_parking_debug->parkingEndCrucialPoints(), stamp);
  parking_end_crucial_points_pub.publish(pecp_msg);
  const auto pec_msg = createPointsClustersMsg(
      perpendicular_parking_debug->parkingEndClusters(), stamp);
  parking_end_clusters_pub.publish(pec_msg);
  const auto left_lane_polynom_msg =
      createPixelsMsgs(perpendicular_parking_debug->leftLanePolynom(), stamp);
  left_lane_polynom_pub.publish(left_lane_polynom_msg);
  const auto spots_lines_msg =
      createScanLinesMsgs(perpendicular_parking_debug->parkingSpotLines(), stamp);
  parking_spots_lines_pub.publish(spots_lines_msg);
}


void PerpendicularParkingNodeDebug::createParkingSpotsMsg() {
  auto perpendicular_parking_debug =
      dynamic_cast<PerpendicularParkingDebug *>(perpendicular_parking_.get());
  assert(perpendicular_parking_debug);

  for (const auto &spot : perpendicular_parking_debug->allParkingSpots()) {
    perception_msgs::PerpendicularParkingSpot msg = spot.get().asMsg();
    all_parking_spots_helper.addSubMessage(msg);
  }
}

perception_msgs::ScanLines PerpendicularParkingNodeDebug::createScanLinesMsgs(
    const ScanLines &scan_lines, const ros::Time &stamp) {
  perception_msgs::ScanLines scan_lines_msg = toScanLinesMsg(scan_lines);
  scan_lines_msg.header.stamp = stamp;
  scan_lines_msg.header.frame_id = "world";
  return scan_lines_msg;
}

perception_msgs::Pixels PerpendicularParkingNodeDebug::createPixelsMsgs(
    const ImagePoints &points, const ros::Time &stamp) {
  perception_msgs::Pixels pixels_msg = toPixelsMsg(points);
  pixels_msg.header.stamp = stamp;
  pixels_msg.header.frame_id = "world";
  return pixels_msg;
}

perception_msgs::Pixels PerpendicularParkingNodeDebug::toPixelsMsg(const ImagePoints &points) const {
  perception_msgs::Pixels pixels_msg;
  for (const ImagePoint &p : points) {
    perception_msgs::Pixel pix = toPixelMsg(p);
    pixels_msg.pixels.push_back(pix);
  }
  return pixels_msg;
}

perception_msgs::Pixel PerpendicularParkingNodeDebug::toPixelMsg(const ImagePoint &point) const {
  perception_msgs::Pixel pix;
  pix.x = point[0];
  pix.y = point[1];
  return pix;
}

perception_msgs::ScanLines PerpendicularParkingNodeDebug::toScanLinesMsg(const ScanLines &scan_lines) {
  perception_msgs::ScanLines scan_lines_msg;
  // All scanlines should be generated from right to left
  for (const auto &line : scan_lines) {
    perception_msgs::ScanLine scan_line_msg;
    scan_line_msg.right = toPixelMsg(line.start);
    scan_line_msg.left = toPixelMsg(line.end);
    scan_lines_msg.scanlines.push_back(scan_line_msg);
  }
  return scan_lines_msg;
}

perception_msgs::PointsClusters PerpendicularParkingNodeDebug::createPointsClustersMsg(
    const FeaturePointClusters &clusters, const ros::Time &stamp) {
  perception_msgs::PointsClusters clusters_msg;
  for (const auto &cluster : clusters) {
    perception_msgs::PointsCluster cluster_msg;
    cluster_msg.pixels = toPixelsMsg(cluster.feature_points_img);
    clusters_msg.clusters.push_back(cluster_msg);
  }
  clusters_msg.header.frame_id = "world";
  clusters_msg.header.stamp = stamp;
  return clusters_msg;
}

}  // namespace perpendicular_parking
