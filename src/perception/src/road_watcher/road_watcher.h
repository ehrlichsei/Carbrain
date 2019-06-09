#ifndef ROAD_WATCHER_H
#define ROAD_WATCHER_H
#include <common/macros.h>

THIRD_PARTY_HEADERS_BEGIN
#include <ros/time.h>
#include <tf2_ros/buffer.h>
#include <array>
#include <opencv2/core/core.hpp>
THIRD_PARTY_HEADERS_END

#include "../utils/ego_vehicle.h"
#include "common/camera_transformation.h"
#include "common/discretisize.h"
#include "common/parameter_interface.h"
#include "common/polynomial.h"
#include "line_vehicle_points.h"
#include "perception_types.h"
#include "vehicle_scan_line.h"

#include "../utils/dbscan_clusterer.h"
#include "../utils/step_detection.h"

/*!
 * \brief Description
 */
class RoadWatcher {
 public:
  /*!
   * \brief RoadWatcher is the consstructor. A ros indipendent functionality
   * containing
   * class needs a pointer to a ParameterInterface (in fact a ParameterHandler)
   * to get access to parameters.
   * \param parameters the ParameterInterface
   */
  RoadWatcher(ParameterInterface *parameters,
              const common::CameraTransformation *const camera_transformation,
              const EgoVehicle *const ego_vehicle);

  RoadWatcher(RoadWatcher &&) = default;
  virtual ~RoadWatcher() = default;

  /*!
   * \brief creates polynomial for the middle lane; creates and applies scan
   * line pattern finding feature points; clusters these feature points using
   * simplified DBScan
   * \param img_gray gray level image
   * \param points lane points detected by lane_detection
   * \param clusters the found feature point clusters
   * \param middle_polynomial the polynomial representing the middle lane;
   * created by shifting, fusing & fitting the input lane points
   */
  virtual void scanRoadAndFindClusters(const cv::Mat &img_gray,
                                       const LineVehiclePoints &points,
                                       std::vector<FeaturePointCluster> &clusters,
                                       common::DynamicPolynomial &middle_polynomial) const;

  const std::array<cv::Point2f, 4> &getFieldOfVision() const {
    return field_of_vision;
  }

 protected:
  //! \brief uses cv::LineIterator to check along scan lines for high gray level
  //! gradients
  virtual ImagePoints apply1dGradientDetector(const cv::Mat &img,
                                              const ScanLines &scan_lines) const;

  /*!
   * \brief createScanLineGrid creates scan pattern on left and right lane;
   * points will be sampled from the fitted polynomials with consistent distance
   * in vehicle x coordinates
   * \param points array of VehiclePoints representing the left, middle, right
   * and - if present - no passing lanes \param middle_polynomial middle lane
   * polynomial, calculated by shifting all lanes to the middle of the track
   * \return every two subsequent 2i points represent a line segment to be
   * scanned
   */

  virtual ScanLines createScanLineGrid(const LineVehiclePoints &points,
                                       const common::DynamicPolynomial &middle_polynomial) const;

  /*!
   * \brief createScanLineGridLane creates scan pattern on left and right lane;
   * points will be sampled from the fitted polynomials with consistent distance
   * in vehicle x coordinates
   * \param discretization_params controls start, stop and distance of sampling
   * \param lane_detection_points right, middle and left lane points
   * \return every two subsequent 2i points represent a line segment to be
   * scanned
   */
  ScanLines createScanLineGridLane(const common::DiscretizationParams &discretization_params,
                                   const common::DynamicPolynomial &middle_polynomial,
                                   const LineVehiclePoints &lane_detection_points) const;

  ScanLines createScanLineGridNoPassingZone(const common::DiscretizationParams &discretization_params,
                                            const common::DynamicPolynomial &middle_polynomial,
                                            const LineVehiclePoints &lane_detection_points) const;

  ScanLines createScanLineGridPedestrianIsland(
      const common::DynamicPolynomial &middle_polynomial,
      const std::vector<double> &distances_right,
      const common::discretization::DiscretizationParams &params,
      const VehiclePoints &middle_samples) const;

  virtual VehiclePoints allPointToMiddle(LineVehiclePoints points) const;

  const common::CameraTransformation *const cam_transform_;

 private:
  //! \brief fuses points found middle lane and shifted outer lanes and
  //! fits middle polynomial
  boost::optional<common::DynamicPolynomial> createMiddlePolynomialByFusion(
      const LineVehiclePoints &points) const;

  std::vector<FeaturePointCluster> clusterWithMiddlePolynom(
      const VehiclePoints &unassigend_feature_points,
      const common::DynamicPolynomial &middle_lane_polynom) const;
  //! \brief Initialises the sampling parametes sampling_x_start, sampling_x_end
  //! and sampling_x_step.
  //! nitIf DYNAMICALLY_RESTRICT_SEARCH_PATTERN if set, the VehiclePoints are
  // used
  //! to calculate sampling_x_end. Results are written into the respecitve
  //! parameters.
  common::DiscretizationParams initSamplingParameters(const LineVehiclePoints &points,
                                                      const common::DynamicPolynomial &middle_polynomial) const;

  std::vector<double> estimateLaneWidth(const common::DynamicPolynomial &lane_polynom,
                                        const common::DiscretizationParams &params,
                                        const VehiclePoints &outer_lane_points) const;

  bool isLeftLinePlausibel(const std::vector<double> &left_distances) const;

  bool arePointsPlausible(const VehiclePoints &side_points,
                          const VehiclePoints &middle_points) const;

  double fieldOfViewNearX() const;
  double fieldOfViewFarX() const;
  bool isInFieldOfView(const cv::Point2f &point) const;
  bool isInFieldOfView(const ScanLine &line) const;

  //  void appendToScanlines(const ImagePoint& start, const ImagePoint& end, const double trim, ScanLines& lines) const;

  /*!
   * \brief parameters_ptr_ is needed for parameter access
   */
  const ParameterInterface *const parameters_ptr_;
  const EgoVehicle *const ego_vehicle;

  const DBScanClusterer clusterer;

  std::array<cv::Point2f, 4> field_of_vision;  // vehicle coordinates
};

#endif  // ROAD_WATCHER_H
