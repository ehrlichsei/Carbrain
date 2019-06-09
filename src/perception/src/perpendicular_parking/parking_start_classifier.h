#pragma once
#include <common/macros.h>

THIRD_PARTY_HEADERS_BEGIN
#include <ros/console.h>
THIRD_PARTY_HEADERS_END

#include "common/camera_transformation.h"
#include "common/polynomial.h"
#include "../utils/dbscan_clusterer.h"
#include "../utils/tf_helper_interface.h"

#include "parking_line.h"

namespace perpendicular_parking {
/**
 * \brief The ParkingStartClassifier class
 *
 * Given a parking area for perpendicular parking,
 * this classifier should detect the start marking
 * which is the rotated (45°-60°) line on the left
 *
 *       ____ ____ ____ ____
 *     /|    |    |    |    |\
 *    / |    |    |    |    | \
 *   /  |    |    |    |    |  \
 * _/___|____|____|____|____|___\____ <- left lane markings
 *
 *
 *  ---  ---  ---  ---  ---  ---  ---
 *
 * __________________________________
 *
 */

using FeaturePointClusters = std::vector<FeaturePointCluster>;

//  using IntersectionLine = Eigen::Hyperplane<double, 2>;
//  using IntersectionLines = std::vector<IntersectionLine>;

class ParkingStartClassifier {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  /*!
   * \brief ParkingStartClassifier constructor for class
   * \param parameters_ptr pointer to parameter_handler
   * \param camera_transform camera_transformation
   * \param world_coordinates_helper world_coordinates_helper for transformation
   * from vehicle to world_frame
   */
  ParkingStartClassifier(ParameterInterface *const parameters_ptr,
                         const common::CameraTransformation *const camera_transform,
                         const tf_helper::TFHelperInterface<double> *const world_coordinates_helper);

  ParkingStartClassifier(ParkingStartClassifier &&) = default;
  ParkingStartClassifier &operator=(ParkingStartClassifier &&) = default;
  virtual ~ParkingStartClassifier() = default;
  /*!
   * \brief classify classifies feature_points detected on the left side of left
   * lane
   * \param cluster raw_cluster of all feature_points detected on left side of
   * left lane
   * \param left_lane_polynom left_lane_polynom, constructed with lane detection
   * points \param timestamp timestamp of current image frame \return Type
   * START, if startline with required belief has been found
   */
  virtual Type classify(FeaturePointCluster &cluster,
                        const common::polynomial::DynamicPolynomial &left_lane_polynom,
                        const ros::Time &timestamp);
  /*!
   * \brief startLinePose getter for pose of start_line for parking_lot, if it
   * is detected
   * \return pose of start line in world_frame
   */
  virtual WorldPose startLinePose() const;
  /*!
   * \brief reset resets all the states inside parking_start_classifier
   */
  void reset();

  static const std::string NAMESPACE;

 protected:
  /*!
   * \brief HORIZON horizon, in which points clusters has to be located to be
   * classified
   */
  static const ParameterString<double> HORIZON;
  /*!
   * \brief MAX_DISTANCE_END_OF_POINTS maximum distance of reference point for
   * matching of lines that have already been seen
   */
  static const ParameterString<double> MAX_DISTANCE_END_OF_POINTS;
  /*!
   * \brief MAX_ANGLE_DELTA maximum tolerance of angle for line-matching
   */
  static const ParameterString<double> MAX_ANGLE_DELTA;
  /*!
   * \brief SECOND_PCA_THLD threshold for second component computed by pca. If
   * score of second component is smaller than this, cluster is considered as
   * line
   */
  static const ParameterString<double> SECOND_PCA_THLD;
  /*!
   * \brief MAX_DIST_TO_BE_VALID maximum distance of a cluster point to be
   * considered
   * as close point for determination of valid clusters
   */
  static const ParameterString<double> MAX_DIST_TO_BE_CLOSE;
  /*!
   * \brief MIN_NR_CLOSE_PTS required number of close points inside a cluster to
   * be
   * considered as a valid one
   */
  static const ParameterString<int> MIN_NR_CLOSE_PTS;
  /*!
   * \brief REQUIRED_CERTAINTY required certainity for classification of a line
   * as parking_start_line
   */
  static const ParameterString<double> REQUIRED_CERTAINTY;

  static const ParameterString<int> RANSAC_MIN_SET_SIZE;
  static const ParameterString<int> RANSAC_NR_EXPECTED_LINES;
  static const ParameterString<double> RANSAC_MIN_MODEL_DIST;
  static const ParameterString<double> MAXIMUM_STARTLINE_ANGLE;
  static const ParameterString<double> MINIMUM_STARTLINE_ANGLE;

  const ParameterInterface *const parameters_ptr_;
  const common::CameraTransformation *const cam_transform_;
  const tf_helper::TFHelperInterface<double> *const world_coordinates_helper_;

  void updateLines(const FeaturePointCluster &cluster,
                   const common::DynamicPolynomial &left_lane_polynom,
                   const ros::Time &stamp);
  /*!
   * \brief generateId generates a unique ID for every new line
   * \return ID
   */
  std::size_t generateId();

  virtual std::vector<common::DynamicPolynomial> fitLinesR(const std::size_t nr_lines,
                                                           const VehiclePoints &points) const;

  std::size_t id_count = 0;

  WorldPose start_line_pose;

  const DBScanClusterer db_clusterer;

  std::vector<Startline> startlines_;
};
}  // namespace perpendicular_parking
