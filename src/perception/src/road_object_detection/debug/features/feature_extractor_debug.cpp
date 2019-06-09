#include "feature_extractor_debug.h"

namespace road_object_detection {

FeatureExtractorDebug::FeatureExtractorDebug(FeatureExtractor &&feature_extractor,
                                             DebugImages *debug_images)
    : FeatureExtractor(std::move(feature_extractor)), debug_images(debug_images) {}

ImagePatch FeatureExtractorDebug::calculateImagePatch(const cv::Mat &camera_image,
                                                      const FeaturePointCluster &cluster) {
  return FeatureExtractor::calculateImagePatch(camera_image, cluster);
}

ImagePatch FeatureExtractorDebug::calculateCannyPatch(const ImagePatch &image_patch) {
  ImagePatch canny = FeatureExtractor::calculateCannyPatch(image_patch);
  debug_images->addCannyPatch(cluster_id, canny.image);
  return canny;
}

ROI FeatureExtractorDebug::calculateRoi(const cv::Mat &camera_image,
                                        const ROIParameters &roi_params) const {
  const auto roi = FeatureExtractor::calculateRoi(camera_image, roi_params);
  debug_images->addImagePatch(0, roi.img_patch_.image);
  return roi;
}

ImagePointExact FeatureExtractorDebug::calculateCenter2d(const FeaturePointCluster &cluster) {
  return FeatureExtractor::calculateCenter2d(cluster);
}

VehiclePoint FeatureExtractorDebug::calculateCenter3d(const FeaturePointCluster &cluster) {
  return FeatureExtractor::calculateCenter3d(cluster);
}

VehiclePoint FeatureExtractorDebug::calculateCenterFootPoint(
    const VehiclePoint &cluster_center, const common::DynamicPolynomial &middle_lane_polynomial) {
  return FeatureExtractor::calculateCenterFootPoint(cluster_center, middle_lane_polynomial);
}

double FeatureExtractorDebug::calculateCenterLaneOrientation(
    const VehiclePoint &cluster_center_middle_lane_foot_point,
    const common::DynamicPolynomial &middle_lane_polynomial) {
  return FeatureExtractor::calculateCenterLaneOrientation(
      cluster_center_middle_lane_foot_point, middle_lane_polynomial);
}

BirdsviewPatch FeatureExtractorDebug::calculateBirdsviewPatch(const cv::Mat &camera_image,
                                                              double cluster_center_lane_orientation,
                                                              const VehiclePoint &cluster_center3d) {
  const BirdsviewPatch birdsview = FeatureExtractor::calculateBirdsviewPatch(
      camera_image, cluster_center_lane_orientation, cluster_center3d);
  debug_images->addBirdsviewPatch(cluster_id, *birdsview.getImage());
  return birdsview;
}

Features FeatureExtractorDebug::extractFeatures(const FeaturePointCluster &cluster,
                                                const cv::Mat &camera_image,
                                                const ros::Time &timestamp,
                                                const common::DynamicPolynomial &middle_lane_polynomial,
                                                const LineVehiclePoints &points,
                                                const boost::optional<ROIParameters> &roi_params) {
  cluster_id = cluster.id;
  return FeatureExtractor::extractFeatures(
      cluster, camera_image, timestamp, middle_lane_polynomial, points, roi_params);
}

}  // namespace road_object_detection
