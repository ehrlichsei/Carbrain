#ifndef FEATURE_EXTRACTOR_H
#define FEATURE_EXTRACTOR_H

#include "common/camera_transformation.h"
#include "common/parameter_interface.h"
#include "common/polynomial.h"

#include "../../utils/ego_vehicle.h"
#include "../../utils/tf_helper_interface.h"
#include "features.h"

#include <ros/time.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

namespace road_object_detection {

using ROIParameters = std::pair<RegionToClassify, cv::Size>;

class FeatureExtractor {
 public:
  FeatureExtractor(ParameterInterface *parameters_ptr,
                   const common::CameraTransformation *,
                   const EgoVehicle *ego_vehicle,
                   const tf_helper::TFHelperInterface<double> *const tf_helper);
  FeatureExtractor(FeatureExtractor &&) = default;
  virtual ~FeatureExtractor() = default;

  virtual Features extractFeatures(const FeaturePointCluster &cluster,
                                   const cv::Mat &camera_image,
                                   const ros::Time &timestamp,
                                   const common::DynamicPolynomial &middle_lane_polynomial,
                                   const LineVehiclePoints &points,
                                   const boost::optional<ROIParameters> &roi_params = boost::none);

 protected:
  const ParameterInterface *parameters_ptr_;
  const common::CameraTransformation *camera_transform_;
  const EgoVehicle *ego_vehicle;
  const tf_helper::TFHelperInterface<double> *const tf_helper_;

  void registerParameters(ParameterInterface *parameters_ptr);

  /*!
   * \brief calculateImagePatch calculates the ImagePatch enclosing the region
   * of interest for the given feature point cluster with added padding
   */

  virtual ImagePatch calculateImagePatch(const cv::Mat &camera_image,
                                         const FeaturePointCluster &cluster);
  virtual ImagePatch calculateCannyPatch(const ImagePatch &image_patch);

  /**
   * calculates image coordinates of feature cluster center
   * @param cluster cluster of feature points
   * @return center in image coordinates
   */
  virtual ImagePointExact calculateCenter2d(const FeaturePointCluster &cluster);

  /**
   * calculates vehicle coordinates of feature cluster center
   * @param cluster cluster of feature points
   * @return center in vehicle coordinates
   */
  virtual VehiclePoint calculateCenter3d(const FeaturePointCluster &cluster);

  /**
   * calculates the neares point of the feature
   * @param cluster cluster of feature points
   * @return point with min x value
   */
  virtual VehiclePoint calculateNearestPoint(const FeaturePointCluster &cluster);

  /**
   * projects given point to given polynomial
   * @param cluster_center point to project to polynomial in vehicle coordinates
   * @param middle_lane_polynomial given polynomial in vehicle coordinates
   * @return projected point
   */
  virtual VehiclePoint calculateCenterFootPoint(const VehiclePoint &cluster_center,
                                                const common::DynamicPolynomial &middle_lane_polynomial);

  /**
   * returns orientation of lane at given point
   * @param cluster_center_middle_lane_foot_point point to calculate the
   * orientation at
   * @param middle_lane_polynomial polynomial for orientation calculation
   * @return angle in x,y-plane starting at x axis
   */
  virtual double calculateCenterLaneOrientation(const VehiclePoint &cluster_center_middle_lane_foot_point,
                                                const common::DynamicPolynomial &middle_lane_polynomial);

  /**
   * returns birds view image around cluster center point
   * @param camera_image image given by camera
   * @param cluster_center_lane_orientation angle of lane orientation at cluster
   *        in x,y-plane starting at x axis
   * @param cluster_center3d cluster center in vehicle coordinates
   * @return birds view image around cluster center point
   */
  virtual BirdsviewPatch calculateBirdsviewPatch(const cv::Mat &camera_image,
                                                 double cluster_center_lane_orientation,
                                                 const VehiclePoint &cluster_center3d);
  /*!
   * \brief calculateRoiPatch
   * \param camera_image the image.
   * \param roi_params the paramters of the region of interes.
   * \return the ROI.
   */
  virtual ROI calculateRoi(const cv::Mat &camera_image, const ROIParameters &roi_params) const;

  cv::Rect extractROIInImage(const cv::Mat &img, const ROIParameters &roi) const;

  VehiclePoints computeROIBoundary(const ROIParameters &roi) const;

  static const ParameterString<double> EPSILON_NEWTON_METHOD;
  static const ParameterString<int> MAX_ITERATIONS_NEWTON_METHOD;

  static const ParameterString<double> PIXEL_WIDTH;
  static const ParameterString<double> OFFSET_BEFORE_CENTER;
  static const ParameterString<double> OFFSET_AFTER_CENTER;
  static const ParameterString<double> OFFSET_RIGHT_OF_CENTER;
  static const ParameterString<double> OFFSET_LEFT_OF_CENTER;

  static const ParameterString<double> THRESHOLD1;
  static const ParameterString<double> THRESHOLD2;

  static const ParameterString<int> EXPAND_X;
  static const ParameterString<int> EXPAND_Y;

  static const ParameterString<double> LANE_WIDTH;
};

}  // namespace road_object_detection

#endif  // FEATURE_EXTRACTOR_H
