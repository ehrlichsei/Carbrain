#ifndef LOOK_AT_H
#define LOOK_AT_H

#include "common/macros.h"

#include "../road_object_detection/classifier.h"
#include "../road_object_detection/features/feature_extractor.h"
#include "../utils/dbscan_clusterer.h"
#include "../utils/ego_vehicle.h"
#include "../utils/tf_helper_interface.h"
#include "common/camera_transformation.h"
#include "common/parameter_interface.h"
#include "vehicle_scan_line.h"

THIRD_PARTY_HEADERS_BEGIN
#include <Eigen/Geometry>
#include <opencv2/core/core.hpp>
THIRD_PARTY_HEADERS_END

namespace look_at {

struct ClassificationResult {

  road_object_detection::RoadObjects detected_objects;

  std::size_t id;
};
using ClassificationResults = std::vector<ClassificationResult>;

/*!
 * \brief Node in order to observe specific regions on demand, invoked from
 * navigation via action
 */
class LookAt {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  /*!
   * \brief LookAt is the consstructor. A ros indipendent functionality
   * containing class needs a pointer to a ParameterInterface (in fact a
   * ParameterHandler) to get access to parameters. \param parameters the
   * ParameterInterface
   */
  LookAt(ParameterInterface *const parameters,
         const common::CameraTransformation *const camera_transform,
         const tf_helper::TFHelperInterface<double> *const tf_helper,
         const EgoVehicle *const ego_vehicle);

  LookAt(LookAt &&) = default;

  virtual ~LookAt() = default;

  virtual ClassificationResults classifyROI(
      const cv::Mat &img,
      const ros::Time &timestamp,
      const RegionsToClassify &regions,
      const LineVehiclePoints &lane_points,
      const std::unique_ptr<road_object_detection::Classifier> &classifier);

  const std::array<cv::Point2f, 4> &getFieldOfVision() const {
    return field_of_vision;
  }

  void setROIOffset(const cv::Size &size) { this->roi_offset_ = size; }

  static const ParameterString<int> FIELD_OF_VISION_TOP;
  static const ParameterString<int> FIELD_OF_VISION_BOTTOM;
  static const ParameterString<int> FIELD_OF_VISION_LEFT;
  static const ParameterString<int> FIELD_OF_VISION_RIGHT;
  static const ParameterString<double> SIDE_TRIM;
  static const ParameterString<double> LANE_WIDTH;
  static const ParameterString<double> AREA_OF_INTEREST_STEP_SIZE;
  static const ParameterString<double> GRADIENT_DETECTION_THLD;
  static const ParameterString<double> STEP_REFERENCE_FUNCTION_LENGTH;
  static const ParameterString<int> DEGREE_OF_POLYNOMIAL;

 protected:
  boost::optional<common::DynamicPolynomial> createMiddlePolynomialByFusion(
      const LineVehiclePoints &points) const;

  virtual VehiclePoints allPointsToMiddle(LineVehiclePoints points) const;

  virtual ImagePoints apply1dGradientDetector(const cv::Mat &img,
                                              const ScanLines &scan_lines) const;

  virtual std::vector<FeaturePointCluster> clusterFeaturePoints(const FeaturePointCluster &raw) const;

  virtual ScanLines createScanLineGrid(const RegionToClassify &roi, const double step_size) const;

  bool isInFieldOfView(const cv::RotatedRect &rrect) const;
  double fieldOfViewNearX() const;
  double fieldOfViewFarX() const;
  bool isInFrontOfCamera(const WorldPoint &p) const;
  bool isInFrontOfCamera(const VehicleScanLine &l) const;
  bool isinFrontOfCamera(const ScanLine &l) const;

  void clipVehicleScanLineFOV(VehicleScanLine &line) const;

  /*!
   * \brief parameters_ptr_ is needed for parameter access
   */
  const ParameterInterface *const parameters_ptr_;
  const common::CameraTransformation *const camera_transformation_;
  const tf_helper::TFHelperInterface<double> *const tf_helper_;
  const EgoVehicle *const ego_vehicle_;
  std::unique_ptr<road_object_detection::FeatureExtractor> feature_extractor_;
  DBScanClusterer clusterer_;

  cv::Size roi_offset_ = cv::Size(0, 0);

  std::array<cv::Point2f, 4> field_of_vision;  // vehicle coordinates
};

}  // namespace look_at

#endif  // LOOK_AT_H
