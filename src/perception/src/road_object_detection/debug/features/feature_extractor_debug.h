#ifndef FEATURE_EXTRACTOR_DEBUG_H
#define FEATURE_EXTRACTOR_DEBUG_H

#include "../../features/feature_extractor.h"
#include "../debug_images.h"

namespace road_object_detection {

class FeatureExtractorDebug : public FeatureExtractor {
 public:
  FeatureExtractorDebug(FeatureExtractor&& feature_extractor, DebugImages* debug_images);

  virtual Features extractFeatures(const FeaturePointCluster& cluster,
                                   const cv::Mat& camera_image,
                                   const ros::Time& timestamp,
                                   const common::DynamicPolynomial& middle_lane_polynomial,
                                   const LineVehiclePoints& points,
                                   const boost::optional<ROIParameters>& roi_params) override;

 protected:
  DebugImages* debug_images;

  virtual ImagePatch calculateImagePatch(const cv::Mat& camera_image,
                                         const FeaturePointCluster& cluster) override;
  virtual ImagePatch calculateCannyPatch(const ImagePatch& image_patch) override;

  virtual ROI calculateRoi(const cv::Mat& camera_image,
                           const ROIParameters& roi_params) const override;

  /**
   * calculates image coordinates of feature cluster center
   * @param cluster cluster of feature points
   * @return center in image coordinates
   */
  virtual ImagePointExact calculateCenter2d(const FeaturePointCluster& cluster) override;

  /**
   * calculates vehicle coordinates of feature cluster center
   * @param cluster cluster of feature points
   * @return center in vehicle coordinates
   */
  virtual VehiclePoint calculateCenter3d(const FeaturePointCluster& cluster) override;

  /**
   * projects given point to given polynomial
   * @param cluster_center point to project to polynomial in vehicle coordinates
   * @param middle_lane_polynomial given polynomial in vehicle coordinates
   * @return projected point
   */
  virtual VehiclePoint calculateCenterFootPoint(const VehiclePoint& cluster_center,
                                                const common::DynamicPolynomial& middle_lane_polynomial) override;

  /**
   * returns orientation of lane at given point
   * @param cluster_center_middle_lane_foot_point point to calculate the orientation at
   * @param middle_lane_polynomial polynomial for orientation calculation
   * @return angle in x,y-plane starting at x axis
   */
  virtual double calculateCenterLaneOrientation(
      const VehiclePoint& cluster_center_middle_lane_foot_point,
      const common::DynamicPolynomial& middle_lane_polynomial) override;

  /**
   * returns birds view image around cluster center point
   * @param camera_image image given by camera
   * @param cluster_center_lane_orientation angle of lane orientation at cluster
   *        in x,y-plane starting at x axis
   * @param cluster_center3d cluster center in vehicle coordinates
   * @return birds view image around cluster center point
   */
  virtual BirdsviewPatch calculateBirdsviewPatch(const cv::Mat& camera_image,
                                                 double cluster_center_lane_orientation,
                                                 const VehiclePoint& cluster_center3d) override;

 private:
  unsigned int cluster_id = 0;
};


}  // namespace road_object_detection

#endif  // FEATURE_EXTRACTOR_DEBUG_H
