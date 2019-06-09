#pragma once

#include "parking_line.h"
#include "parking_spot.h"

THIRD_PARTY_HEADERS_BEGIN

THIRD_PARTY_HEADERS_END
#include <common/polynomial.h>
#include "../utils/dbscan_clusterer.h"
#include "../utils/tf_helper_interface.h"
#include "common/discretisize.h"
#include "line_vehicle_points.h"

namespace perpendicular_parking {

using ParkingSpotsConstRef = std::vector<ParkingSpotConstRef>;

using FeaturePointClusters = std::vector<FeaturePointCluster>;

class ParkingEndClassifier {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  ParkingEndClassifier(ParameterInterface *const parameters_ptr,
                       const common::CameraTransformation *const camera_transform,
                       const tf_helper::TFHelperInterface<double> *const world_coordinates_helper);

  virtual ~ParkingEndClassifier() = default;
  ParkingEndClassifier(ParkingEndClassifier &&) = default;
  /*!
   * \brief detectEnd detects endline of parking_lot
   * \param all_parking_spots parking spots that are already initialized
   * \param img current camera image
   * \param timestamp timestamp related to current camera image
   * \param left_lane_polynom polynom fit of left lane that is taken as
   * reference for position of endline
   * \param lanes lane information, provided by lane detection
   * \param world_T_map transform from map frame to world frame
   * \return boost::none, if no endline has been detected so far. Pose in world
   * frame, consisting of position (on left lane polynom) and orientation
   * pointing in direction of line-segement
   */
  virtual boost::optional<WorldPose> detectEnd(const ParkingSpotsConstRef &all_parking_spots,
                                               const cv::Mat &img,
                                               const ros::Time &timestamp,
                                               const common::DynamicPolynomial &left_lane_polynom,
                                               const LineVehiclePoints &lanes,
                                               const MapPose &world_T_map);
  /*!
   * \brief reset resets parking_end_classifier
   */
  virtual void reset();
  /*!
   * \brief NAMESPACE in order to be able to find parameters in .yaml files of
   * perpendicular parking
   */
  static const std::string NAMESPACE;

 protected:
  /*!
   * \brief apply1DGradientDetector extract feature points along scanlines
   * \param img see description of detectEnd
   * \param scan_lines scanlines to which feature point detection is applied
   * \param all flag determining if all feature points along every scanline
   * shall be returned or just the first
   * \return detected feature points in image frame
   */
  virtual ImagePoints apply1DGradientDetector(const cv::Mat &img,
                                              const ScanLines &scan_lines,
                                              const bool all) const;
  /*!
   * \brief getEndlineFeaturePoints determines feature points beyond parking
   * spots, which are valid ones for a hypothetical endline
   * \param img see description of detectEnd
   * \param left_lane_polynom see description of detectEnd
   * \return valid feature points for an endline in image frame
   */
  virtual ImagePoints getEndlineFeaturePoints(const cv::Mat &img,
                                              const common::DynamicPolynomial &left_lane_polynom,
                                              const common::DiscretizationParams &disc_params) const;
  
  boost::optional<double> computeFreeDepth(const cv::Mat &img,
                                           const common::DynamicPolynomial &left_lane_polynom,
                                           const VehiclePoint &farest_cluster_point) const;

  /*!
   * \brief getParallelScanlines generates scanlines parallel to a line between
   * the projection of a start- and end-point onto a polynom
   * \param polynom the polynom, the points are projected onto
   * \param start_point start point in vehicle frame (unprojected)
   * \param end_point end point in vehicle frame (unprojected)
   * \return the generated scanlines
   */
  virtual ScanLines getParallelScanlines(const common::DynamicPolynomial &polynom,
                                         const VehiclePoint &start_point,
                                         const VehiclePoint &end_point) const;
  /*!
   * \brief createScanLines creates scanlines between start- and end-points
   * \param start_points startpoints for scanlines
   * \param end_points endpoints for scanlines
   * \return created scanlines
   */
  virtual ScanLines createScanLines(const ImagePoints &start_points,
                                    const ImagePoints &end_points) const;
  /*!
   * \brief clipScanLines
   * \param unclipped the yet to be clipped scanlines.
   * \param img_size the image size to clipp against.
   * \return the clipped scan lines.
   */
  ScanLines clipScanLines(const ScanLines& unclipped, const cv::Size &img_size) const;

  FeaturePointClusters extractClusters(const ImagePoints &feature_points) const;
  /*!
   * \brief generateID generates an ID for every new Candidate Line, that has
   * been found
   * \return the generated ID
   */
  std::size_t generateID();


  void updateEndlines(const FeaturePointCluster &cluster,
                      const common::DynamicPolynomial &left_lane_polynom,
                      const ros::Time &stamp,
                      const boost::optional<double> &free_space);

  virtual std::vector<common::DynamicPolynomial> fitLinesR(const std::size_t nr_lines,
                                                           const VehiclePoints &points) const;

  common::DiscretizationParams generateDiscParams(const LineVehiclePoints &lane_points,
                                                  const ParkingSpotsConstRef &all_parking_spots,
                                                  const MapPose &world_T_map) const;

  /*!
   * \brief parameters_ptr_ pointer to parameter interface in order to be able
   * to access parameters stored in .yaml-file
   */
  ParameterInterface *const parameters_ptr_;
  /*!
   * \brief cam_transform_ the transformation between image and vehicle frame
   */
  const common::CameraTransformation *const cam_transform_;
  /*!
   * \brief world_coordinates_helper_ provides information about transform
   * between world and vehicle frame
   */
  const tf_helper::TFHelperInterface<double> *const world_coordinates_helper_;
  /*!
   * \brief db_clusterer_ clusters detected feature points
   */
  const DBScanClusterer db_clusterer_;


  std::size_t id_count_ = 0;

  std::vector<Endline> parking_endlines_;

  static const ParameterString<double> LEFT_LANE_POLY_STEP;
  static const ParameterString<double> SCAN_LINE_LENGTH;
  static const ParameterString<double> STEP_DETECTION_THLD;
  static const ParameterString<int> STEP_DETECTION_REF_FUNC_SIZE;
  static const ParameterString<double> PARKING_SPOT_DEPTH;
  static const ParameterString<double> LEFT_LANE_SCANLINE_PADDING;
  static const ParameterString<double> ADDITIONAL_PADDING;
  static const ParameterString<double> SCANLINE_X_OFFSET;
  static const ParameterString<double> MINIMAL_HYPOTHESE_BELIEF;
  static const ParameterString<double> MARKING_SCANLINE_SHIFT;
  static const ParameterString<int> NO_MARKING_SCANLINES;
  static const ParameterString<int> MIN_CLUSTER_SIZE;
  static const ParameterString<double> RANSAC_MIN_DIST;
  static const ParameterString<int> RANSAC_MIN_CLUSTER_SIZE;
  static const ParameterString<int> RANSAC_NR_LINES;
  static const ParameterString<double> MINIMUM_ENDLINE_ANGLE;
  static const ParameterString<double> MAXIMUM_ENDLINE_ANGLE;
};
}  // namespace perpendicular_parking
