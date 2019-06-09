#ifndef PATH_PREPROCESSING_H
#define PATH_PREPROCESSING_H
#include <common/macros.h>

THIRD_PARTY_HEADERS_BEGIN
#include <nav_msgs/Path.h>
#include <Eigen/Eigen>
#include <utility>
THIRD_PARTY_HEADERS_END

#include "common/parameter_interface.h"
#include "common/polynomial.h"
#include "common/types.h"
#include "navigation/gate.h"
#include "navigation/driving_corridor.h"

#include "trackedlane.h"
#include "boundingbox.hpp"
#include "lane_utils.h"

const ParameterString<double> PARAM_TRACKING_DISTANCE_POLYNOMIAL =
    ParameterString<double>("tracking_distance_polynomial");
const ParameterString<double> PARAM_TRACKING_DISTANCE_BEHIND =
    ParameterString<double>("tracking_distance_behind");
const ParameterString<double> PARAM_TRACKING_DISTANCE_IN_FRONT =
    ParameterString<double>("tracking_distance_in_front");
const ParameterString<double> PARAM_LANE_WIDTH =
    ParameterString<double>("lane_width");
const ParameterString<double> PARAM_INITIAL_GATE_DISTANCE =
    ParameterString<double>("initial_gate_distance");
const ParameterString<int> PARAM_MAX_NUMBER_GATES =
    ParameterString<int>("max_number_gates");
const ParameterString<double> PARAM_DISTANCE_OF_TURNING_POINT_TO_RIGHT_LANE_MARKING =
    ParameterString<double>("distance_of_turning_point_to_right_lane_marking");
const ParameterString<double> PARAM_STRAIGHT_ON_DISTANCE_AFTER_TURNING =
    ParameterString<double>("straight_on_distance_after_turning");
const ParameterString<double> PARAM_CORRIDOR_SIMPLIFICATION_MAX_MERGE_DISTANCE =
    ParameterString<double>("corridor_simplification_max_merge_distance");
const ParameterString<bool> PARAM_ENABLE_PERPENDICULAR_PARKING =
    ParameterString<bool>("enable_perpendicular_parking");
const ParameterString<bool> PARAM_ENABLE_TURNING =
    ParameterString<bool>("enable_turning");
const ParameterString<bool> PARAM_USE_ADAPTIVE_POLYNOMIAL_DEGREE =
    ParameterString<bool>("use_adaptive_polynomial_degree");
const ParameterString<int> PARAM_PATH_POLYNOMIAL_DEGREE =
    ParameterString<int>("path_polynomial_degree");
const ParameterString<double> WEIGHT_MIDDLE_INTERSECTION =
    ParameterString<double>("weight_middle_intersection");
const ParameterString<double> WEIGHT_OPPOSITE_INTERSECTION =
    ParameterString<double>("weight_opposite_intersection");
const ParameterString<double> WEIGHT_OWN_INTERSECTION =
    ParameterString<double>("weight_own_intersection");
const ParameterString<double> THRESHOLD_USE_ALWAYS_DEGREE_1 =
    ParameterString<int>("threshold_use_always_degree_1");
const ParameterString<double> THRESHOLD_USE_NEVER_DEGREE_5 =
    ParameterString<int>("threshold_use_never_degree_5");
const ParameterString<double> THRESHOLD_RELATIVE_ERROR_1_5 =
    ParameterString<int>("threshold_relative_error_1_5");
const ParameterString<double> THRESHOLD_RELATIVE_ERROR_3_5 =
    ParameterString<int>("threshold_relative_error_3_5");
const ParameterString<double> TURNING_ANGLE =
    ParameterString<double>("turning_angle");
const ParameterString<double> PERPENDICULAR_PARKING_ANGLE =
    ParameterString<double>("perpendicular_parking_angle");
const ParameterString<double> TURNING_ANGLE_START_OFFSET =
    ParameterString<double>("turning_angle_start_offset");
const ParameterString<double> TURNING_ANGLE_STEP_SIZE =
    ParameterString<double>("turning_angle_step_size");
const ParameterString<double> PARAM_LENGTH_OF_PARKING_SPOT =
    ParameterString<double>("length_of_parking_spot");


/*!
 * \brief Description
 */
class PathPreprocessing {
 public:
  /*!
  * \brief PathPreprocessing is the consstructor. A ros indipendent
  * functionality containing class needs a pointer to a ParameterInterface (in
  * fact a ParameterHandler) to get access to parameters.
  *
  * \param parameters the ParameterInterface
  */
  PathPreprocessing(ParameterInterface* parameters);

  /**
   * creates gates from given lane lines and tracked gates
   *
   * @param vehicle_to_world_3d 3d transformation from vehicle to world
   * coordinates
   * @param lane_paths lane points from all lanes in world corrdinates
   * @return whether a new corridor was successfull created
   */
  bool createDrivingCorridor(const Eigen::Affine3d& vehicle_to_world_3d,
                             const LaneUtils::Lanes& lane_paths);

  /**
   * integrates given lane points into tracked lane
   */
  void integrateNewLanePaths(const Eigen::Vector2d& vehicle_pos,
                             const LaneUtils::Lanes& lanes);

  /**
   * gets current tracked lane points
   *
   * @return the current tracked lanes.
   */
  const TrackedLane& getTrackedLane() const;

  /**
   * @brief returns a transformation matrix
   * that transforms a Point in World-Space to it's coordinates in Polynomial
   * space used for polynomial to create the gates from
   *
   * The polynomial space is aligned along the vectors obtained by a principal
   * component analysis of all tracked lane points
   *
   * @return the transformation matrix
   */
  Eigen::Affine3d determinePathTransform(const Eigen::Vector2d& vehicle_pos,
                                         const Eigen::Vector2d& vehicle_orientation,
                                         const common::EigenAlignedVector<Eigen::Vector3d>& points) const;

  /**
   * @brief creates coordinate system from the corridor for path created in
   * path_planning
   */
  Eigen::Affine3d determinePathTransformFullCorridor(const Eigen::Affine3d& vehicle_to_world_3d) const;

  /**
   * returns path to world transformation
   *
   * @return transformation from path to world
   */
  const Eigen::Affine3d& getPathTransform() const;

  /**
   * initial gates
   */
  const DrivingCorridor& getRawCorridor() const;

  /**
   * gets final gates
   */
  const DrivingCorridor& getFullCorridor() const;

  /**
   * @brief fits a polynomials through given path
   *
   * Tries to find the best polynomial degree for the given situation.
   */
  void fitPolynomial(const common::EigenAlignedVector<Eigen::Vector2d>& path,
                     common::DynamicPolynomial& polynomial) const;

  /**
   * \brief reset state of pathplanning
   *
   * resetting includes
   * - forget old path
   * - do not turn left or right
   * - do not park into parking spot
   */
  void reset();

  /**
   * @brief sets point contol uses as lookahead
   */
  void setVehicleLookaheadPoint(const Eigen::Vector3d& point);

  /**
   * \brief start turning at specified point
   *
   * \param turn_left true -> turn left, false -> turn right
   */
  void setTurning(bool turn_left, Eigen::Affine3d turning_point);

  /**
   * \brief reset state of turning, do not turn anymore
   */
  void resetTurning();

  /**
   * \brief start parking routine at specified point
   *
   * \param left left starting point of parking slot
   * \param right right starting point of parking slot
   */
  void parkPerpendicular(const Eigen::Affine3d& left, const Eigen::Affine3d& right);

  bool pointBehindStartGateParking(const Eigen::Vector3d& point) const;

 private:
  /**
   * creates gates from given lane lines
   *
   * @param gates to create the gates in
   * @param vehicle_to_world_3d 3d transformation from vehicle to world
   * coordinates
   * @param lanes lane points of all lanes in world corrdinates for clamping the
   *gates
   * @param lane_polynomial polynomial for gate directions
   * @param tracked_lane_path used for min and max point on polynomial
   */
  void createInitialGates(Gate::GateList* gates,
                          const Eigen::Affine3d& vehicle_to_world_3d,
                          const LaneUtils::Lanes& lanes,
                          const common::DynamicPolynomial& lane_polynomial,
                          const common::Vector2dVector& tracked_lane_path);

  /**
   * create gates based on old gates and new lanes
   *
   * @param gates to add the new gates at
   * @param vehicle_to_world_3d 3d transformation from vehicle to world
   * coordinates
   * @param lanes lane points of all lanes in world corrdinates
   */
  void createUpdatedGates(Gate::GateList* gates,
                          const Eigen::Affine3d& vehicle_to_world_3d,
                          const LaneUtils::Lanes& lanes);

  /**
   * @brief create gates based on old gates around the vehicle
   *
   * uses lanes and vehicle position to decide whether a gate has to be updated
   *(and not added to the list) or kept (and added to the list)
   *
   * @param gates to create the new gates at
   * @param vehicle_to_world_3d 3d transformation from vehicle to world
   * coordinates
   * @param lanes lane points of all lanes in world corrdinates
   */
  bool keepTrackedGates(Gate::GateList* gates,
                        const Eigen::Affine3d& vehicle_to_world_3d,
                        const LaneUtils::Lanes& lanes);

  /**
   * create virtual lanes from given gates
   *
   * @param old_lanes lane points based on gate points
   */
  void extractOldLanes(LaneUtils::Lanes* old_lanes);

  /**
   * update tracked lane points for polynomial generation
   */
  bool updateLanePoints(const Eigen::Affine3d& vehicle_to_world_3d,
                        const LaneUtils::Lanes& lanes,
                        Eigen::Affine3d* world_to_path_transform);

  /**
   * creates polynomial from tracked lane
   *
   * @param tracked_lane_path points to create polynomial from
   * @param lane_polynomial contains created polynomial
   * @param world_to_path_transform created transformation for polynomial
   */
  void createLanePolynomial(common::EigenAlignedVector<Eigen::Vector2d>* tracked_lane_path,
                            common::DynamicPolynomial* lane_polynomial,
                            Eigen::Affine3d* world_to_path_transform) const;

  /**
   * \brief reached point for start turning
   */
  bool startTurning(const Gate& gate) const;

  Gate createFirstParkingGate(const int gate_id) const;

  /**
   * add parking gates at the end of the raw_corridor
   */
  void addPerpendicularParkingToCorridor();

  /**
   * add turning gates at the end of the raw_corridor
   */
  void addTurningGatesToCorridor();

  /*!
  * \brief parameters_ptr_ is needed for parameter access
  */
  const ParameterInterface* parameters_ptr_;

  TrackedLane tracked_lane;
  DrivingCorridor raw_corridor_;
  DrivingCorridor full_corridor_;
  BoundingBox lane_aabb;
  Eigen::Affine3d path_to_world_transform_;

  unsigned int next_gate_id_;

  enum Turning { DONT_TURN, TURN_LEFT, TURN_RIGHT };

  bool turning_enabled = false;
  Turning turning_state;
  Eigen::Vector3d start_turning;
  Eigen::Matrix3d start_turning_orientation;

  Eigen::Vector3d vehicle_lookahead_point_ = Eigen::Vector3d(0.0, 0.0, 0.0);

  bool perpendicular_parking_enabled = false;
  bool park_at_perpendicular_spot_;
  Eigen::Affine3d perpendicular_parking_left_entrance_;
  Eigen::Affine3d perpendicular_parking_right_entrance_;

  double weight_middle_intersection = 1.0;
  double weight_opposite_intersection = 1.0;
  double weight_own_intersection = 1.0;
};

#endif  // PATH_PREPROCESSING_H
