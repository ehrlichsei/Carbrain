#ifndef PERPENDICULAR_PARKING_H
#define PERPENDICULAR_PARKING_H

#include <common/macros.h>

THIRD_PARTY_HEADERS_BEGIN
#include <memory>

#include <ros/time.h>
#include <array>
#include <opencv2/core/core.hpp>
THIRD_PARTY_HEADERS_END

#include "common/polynomialfit.h"
#include "parking_lot.h"
#include "parking_start_classifier.h"
#include "parking_end_classifier.h"
#include "../utils/tf_helper_interface.h"
/*!
 * \brief Node for detection and localization of parking spots on the left part
 * of the lane, that can be used for perpendicular parking
 */
namespace perpendicular_parking {

using FeaturePointClusters = std::vector<FeaturePointCluster>;
using LanePointsClusters = std::vector<VehiclePoints>;
using GroundPoint = Eigen::Vector2d;

class PerpendicularParking {
 public:
  /*!
  * \brief PerpendicularParking is the consstructor. A ros indipendent
  * functionality containing class needs a pointer to a ParameterInterface (in
  * fact a ParameterHandler) to get access to parameters.
  * \param world_coordinates_helper the world coordinates helper.
  * \param parameters_ptr the ParameterInterface.
  * \param camera_transform the camera transformation.
  */
  PerpendicularParking(tf_helper::TFHelperInterface<double>* const world_coordinates_helper,
                       ParameterInterface* parameters_ptr,
                       const common::CameraTransformation* const camera_transform);

  virtual ~PerpendicularParking() = default;
  PerpendicularParking(PerpendicularParking&&) = default;

  /*!
   * \brief findFreeParkingSpots central interface to node_wrapper, searches for
   * free parking spots
   * \param img camera_image_raw
   * \param lanes left, right and middle lanes published from lane_detection
   * \param timestamp timestamp of camera image. This one should be used
   * everywhere, where such a timestamp is required
   * \return all free parking spots in parking_lot, containing poses for left
   * and right entrance of parking spot and index
   */
  virtual ParkingSpotsConstRef findFreeParkingSpots(const cv::Mat& img,
                                                    const LineVehiclePoints& lanes,
                                                    const ros::Time& timestamp);
  /*!
   * \brief setActivationStatus activates the functionality, when service call
   * from navigation is invoked
   * \param search true, if search for free parking spots is demanded; false,
   * otherwise
   */
  void setActivationStatus(const bool search);

  /*!
   * \brief mapPose computes the mapPose in world frame
   * \return affine transformation from map_frame (which is used inside
   * parking_lot) to world_frame
   */
  const WorldPose mapPose() const;

  /*!
   * \brief isParkingLotDetected indicates, whether a parking lot has been
   * detected. Security feature in order inform navigation about possible
   * invalid aactivation and enable a deactivation for such cases
   * \return true, if parking_lot is detected; false otherwise
   */
  bool isParkingLotDetected() const { return this->parking_lot_detected_; }

  static const std::string NAMESPACE;

 protected:
  VehiclePoints polynomial_points;
  WorldPoints polynomial_points_world;
  cv::Size img_size;
  std::unique_ptr<ParkingStartClassifier> parking_start_classifier;
  std::unique_ptr<ParkingLot> parking_lot;
  std::unique_ptr<ParkingEndClassifier> parking_end_classifier;


  /*!
   * \brief LEFT_LANE_POLY_STEP discretization step for computation of
   * left_lane_polynom
   */
  static const ParameterString<double> LEFT_LANE_POLY_STEP;
  /*!
   * \brief POLYNOMIAL_DEGREE_LINE degree of left_lane_polyom, if parking_start
   * has been detected
   */
  static const ParameterString<int> POLYNOMIAL_DEGREE_LINE;
  /*!
   * \brief POLYNOMIAL_DEGREE_CURVE degree of left_lane_polynom before detection
   * of parking_start
   */
  static const ParameterString<int> POLYNOMIAL_DEGREE_CURVE;
  /*!
   * \brief SCAN_LINE_LENGTH length of scan_lines for detection of lines on the
   * left side of left_lane
   */
  static const ParameterString<double> SCAN_LINE_LENGTH;
  /*!
   * \brief STEP_DETECTION_THLD threshold for step_detection
   */
  static const ParameterString<double> STEP_DETECTION_THLD;
  /*!
   * \brief STEP_DETECTION_REF_FUNC_SIZE length of reference_function for
   * step_detection
   */
  static const ParameterString<int> STEP_DETECTION_REF_FUNC_SIZE;
  /*!
   * \brief MIN_OUTLAYER_DIST minimal distance to cluster center perpendicular
   * to principal component of lane cluster for clustering
   */
  static const ParameterString<double> MIN_OUTLAYER_DIST;
  /*!
   * \brief MIN_CLUSTER_CENTER_DIST general minimal distance to cluster center
   */
  static const ParameterString<double> MIN_CLUSTER_CENTER_DIST;
  /*!
   * \brief CLUSTER_MIN_SIZE minimal number of points to build a cluster. Used
   * to delete last points, if there are not enough at the end
   */
  static const ParameterString<int> CLUSTER_MIN_SIZE;
  /*!
   * \brief CLUSTER_SIZE cluster size for all clusters but the last
   */
  static const ParameterString<int> CLUSTER_SIZE;
  /*!
   * \brief CUTTING_DISTANCE When parking_start_line is detected, points of left
   * lane, that are wider away than this distance aren't used for generation off
   * new scan lines
   */
  static const ParameterString<double> CUTTING_DISTANCE;
  /*!
   * \brief REMOVAL_DISTANCE_FRONT distance from origin of vehicle_frame, in
   * which first scan line is generated
   */
  static const ParameterString<double> REMOVAL_DISTANCE_FRONT;
  /*!
   * \brief MIN_SIZE_LANE_POINTS minimal size of lane points to be processed by
   * perpendicular parking
   */
  static const ParameterString<int> MIN_SIZE_LANE_POINTS;
  /*!
   * \brief MAX_TIME_WOUT_STARTPOSE if class is activated and start_line hasn't
   * been seen for longer than this time, it is deactivated
   */
  static const ParameterString<double> MAX_TIME_WOUT_STARTPOSE;
  /*!
   * \brief EPSILON_POLY_POINTS distance from start_line.translation() for
   * remembering of polynomial_points, if it's detected
   */
  static const ParameterString<double> EPSILON_POLY_POINTS;
  /*!
   * \brief SCAN_LINE_SHIFT distance from left_lane_polynom, in which scan_lines
   * start (in order to avoid step detection processing left_lane)
   */
  static const ParameterString<double> SCAN_LINE_SHIFT;

  static const ParameterString<double> ENDLINE_RESET_DISTANCE;

  /*!
   * \brief createScanLines creates the scan lines used to detect point clusters
   * left of left_lane
   * \param left_lane points, the left lane polynom consists of
   * \return ScanLines for apply1DGradientDetector
   */
  virtual ScanLines createScanLines(VehiclePoints& left_lane);
  /*!
   * \brief apply1DGradientDetector applies step_detection on along scan_lines
   * in order to find feature_points
   * \param img camera_image
   * \param scan_lines scan_lines generated from createScanLines
   * \return detected feature_points in image_frame
   */
  virtual ImagePoints apply1DGradientDetector(const cv::Mat& img, const ScanLines& scan_lines);
  /*!
   * \brief allPointsToLeft fuses left, middle and right lane to common cluster
   * on located of left_lane (disabled right now (04.01.18)
   * \param lanes left, middle and right lane in vehicle_frame
   * \return fused points in vehicle_frame
   */
  VehiclePoints allPointsToLeft(const LineVehiclePoints& lanes);
  /*!
   * \brief deleteOutlayers deletes outlayers, caused by errors in normalShift
   * \param fused fused left_lane_points
   * \return left_lane_points without outlayers
   */
  VehiclePoints deleteOutlayers(const VehiclePoints& fused);
  /*!
   * \brief deleteClusterOutlayers deletes outlayers in the clusters, the fused
   * left lane points are separated in, in deleteOutlayers
   * \param cluster cluster of left_lane_points
   */
  void deleteClusterOutlayers(VehiclePoints* cluster);
  /*!
   * \brief generatePolynomialPoints generates the points for the fit of
   * left_lane_polynom
   * \param actual_poly_points left_lane_points detected by lane_detection in
   * actual frame
   */
  void generatePolynomialPoints(const VehiclePoints& actual_poly_points);
  /*!
   * \brief clusterLanePoints clusters fused left_lane_points in order to enable
   * outlayer-deletion
   * \param lane_points fused lane_points
   * \return clusters, the left_lane is split to
   */
  LanePointsClusters clusterLanePoints(const VehiclePoints& lane_points);
  /*!
   * \brief getNormalComponent computes direction normal to principal component
   * \param components matrix, containing the principal components as
   * column-vectors
   * \param mean mean of cluster, used as cluster center
   * \return vector, pointing normal to principal component
   */
  GroundPoint getNormalComponent(const Eigen::MatrixXd& components, const GroundPoint& mean);
  /*!
   * \brief clipScanLines clips scan lines to image limits
   * \param lines raw scan_lines
   * \return clipped scan_lines
   */
  const ScanLines clipScanLines(const ScanLines& lines) const;
  /*!
   * \brief generateParams generates params used for normalShift
   * \param x_1 minimum x value
   * \param x_2 maximum x value
   * \param step discretization step
   * \return params
   */
  common::DiscretizationParams generateParams(const double x_1, const double x_2, const double step);
  /*!
   * \brief getPolynomialDegree returns polynom degree of left_lane_polynom,
   * depending of state of perpendicular_parking
   * \return polynom degree
   */
  common::PolynomialDegree getPolynomialDegree() const;
  /*!
   * \brief reset resets all members, that are updated, when
   * perpendicular_parking is deactivated
   */
  void reset();

  tf_helper::TFHelperInterface<double>* const world_coordinates_helper_;
  const ParameterInterface* const parameters_ptr_;
  const common::CameraTransformation* const camera_transform_;

  common::DynamicPolynomial left_lane_polynom_;

  bool activated_;
  bool parking_lot_detected_;
};
}
#endif  // PERPENDICULAR_PARKING_H
