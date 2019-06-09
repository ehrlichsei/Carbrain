#ifndef OBSTACLE_CLASSIFIER_NEW_H
#define OBSTACLE_CLASSIFIER_NEW_H

#include "../../utils/ego_vehicle.h"
#include "../classifier.h"
#include "scan_line.h"

#include <common/eigen_utils.h>
#include <common/math.h>
#include <common/minmax_element.h>
#include <common/types.h>
#include <vehicle_scan_line.h>

THIRD_PARTY_HEADERS_BEGIN
#include <cv.h>
#include <boost/range/adaptor/transformed.hpp>
THIRD_PARTY_HEADERS_END



namespace road_object_detection {


using LineModel = Eigen::ParametrizedLine<double, 2>;
using boost::adaptors::transformed;


/******************************************************************************
 *Line*************************************************************************
 *****************************************************************************/

template <class EigenVectorTypeD>
/*!
 *\brief The Line class defines a line with a start and end point.
 *Start and end points are defined with Eigen::Vector types
 */
class Line {
 public:
  EigenVectorTypeD start;
  EigenVectorTypeD end;


  /*!
   * \brief Line constructs a valid line with start and end point
   * \param start start point of line
   * \param end end poitn of line
   */
  Line(const EigenVectorTypeD& start, const EigenVectorTypeD& end)
      : start(start), end(end) {}

  /*!
   * \brief Line consturcts a line with both start and end point beeing
   * assign to the null vector.
   */
  Line() : start(EigenVectorTypeD::Zero()), end(EigenVectorTypeD::Zero()) {}

  /*!
   * \brief isEqualTo compares to lines
   * \param line line which is compared to this
   * \return returns true if this line has the same start and end point
   * as the line provided as parameter.
   */
  bool isEqualTo(const Line& line) const {
    if (start == line.start && end == line.end) {
      return true;
    }
    return false;
  }

  /*!
   * \brief mean
   * \return returns the mean between start end end point
   */
  EigenVectorTypeD mean() const { return (start + end) * 0.5; }


  /*!
   * \brief diff
   * \return returns a vector that points from start to end
   */
  EigenVectorTypeD diff() const { return end - start; }

  /*!
   * \brief directionNormalized
   * \return returns a unit vector that points from the start point in direction
   * of the end point.
   */
  EigenVectorTypeD directionNormalized() const {
    return (end - start).normalized();
  }

  /*!
   * \brief length
   * \return the length of the line
   */
  double length() const { return (end - start).norm(); }

  /*!
   * \brief distanceVectorInf
   * \param p Point to which the distance_vector is calculated
   * \return returns the shortest vector (=perpendicular on the line) pointing
   * from the line to the given point assuming the line is infinitely long.
   */
  EigenVectorTypeD distanceVectorInf(const EigenVectorTypeD& p) const {
    const auto ps = p - start;
    const auto line_vector = directionNormalized();
    return (ps - ps.dot(line_vector) * line_vector);
  }

  /*!
   * \brief distanceInf
   * \param p Point to which the distance is calculated
   * \return returns the distance of point p to the line assuming the line is
   * infinitely long
   */
  double distanceInf(const EigenVectorTypeD& p) const {
    return distanceVectorInf(p).norm();
  }


  /*!
   * \brief distanceVectorBounded
   * \param p Point to which the distance_vector is calculated
   * \return returns the shortest vector (=perpendicular on the line) pointing
   * from the line to the given point assuming the line starts at start and ends
   * at end.
   */
  EigenVectorTypeD distanceVectorBounded(const EigenVectorTypeD& p) const {
    const auto ps = p - start;
    const auto line_vector = directionNormalized();
    const double t = ps.dot(line_vector);
    if (t >= 0 && t <= length()) {
      return (ps - t * line_vector);
    } else if (t > 0) {
      return (p - end);
    } else {
      return (p - start);
    }
  }

  /*!
   * \brief distanceBounded
   * \param p Point to which the distance is calculated
   * \return returns the distance of point p to the line assuming the line
   * starts at start and ends at end.
   */
  double distanceBounded(const EigenVectorTypeD& p) const {
    return distanceVectorBounded(p).norm();
  }


  /*!
   * \brief distanceToStart
   * \param p Point to which the distance is calculated
   * \return returns the euclidean distance of p and start
   */
  double distanceToStart(const EigenVectorTypeD& p) const {
    return (p - start).norm();
  }

  /*!
   * \brief squaredDistanceToStart
   * \param p Point to which the distance is calculated
   * \return returns the squared euclidean distance of p and start
   */
  double squaredDistanceToStart(const EigenVectorTypeD& p) const {
    return (p - start).squaredNorm();
  }

  /*!
   * \brief distanceToEnd
   * \param p Point to which the distance is calculated
   * \return returns the euclidean distance of p and end
   */
  double distanceToEnd(const EigenVectorTypeD& p) const {
    return (p - end).norm();
  }

  /*!
   * \brief squaredDistanceToEnd
   * \param p Point to which the distance is calculated
   * \return returns the squared euclidean distance of p and end
   */
  double squaredDistanceToEnd(const EigenVectorTypeD& p) const {
    return (p - end).squaredNorm();
  }

  /*!
   * \brief signedDistanceAlongFromStart
   * \param p Point to which the distance is calculated
   * \return returns signed distance from start to p such that distances in line
   * direction have a positive sign and distances against line direction have
   * negative sign.
   */
  double signedDistanceAlongFromStart(const EigenVectorTypeD& p) const {
    return (p - start).dot(directionNormalized());
  }

  /*!
   * \brief angleTo
   * \param v vector to which the angle is calculated
   * \return returns the angle (<90°) between the line and the given vector v
   */
  double angleTo(const EigenVectorTypeD& v) const {
    const double a = std::acos(directionNormalized().dot(v.normalized()));
    return (a > M_PI_2) ? -(a - M_PI) : a;
  }

  /*!
   * \brief addOffset adds the same vector offsett to start and end point
   * \param v a Vector defining the offset
   */
  void addOffset(const EigenVectorTypeD& v) {
    start = start + v;
    end = end + v;
  }

  /*!
   * \brief shrinkSymetricAbs reduces the length of line symmertically on both
   * sides by shrink_distance_abs. I  total the length of the line is reduced b<
   * 2*shrink_distance_abs
   * \param shrink_distance_abs abs scalar value
   */
  void shrinkSymetricAbs(const double shrink_distance_abs) {
    start = start + shrink_distance_abs * directionNormalized();
    end = end - shrink_distance_abs * directionNormalized();
  }

  /*!
   * \brief shrinkSymetricRel reduces the length of the line symmetrically such
   * that the after calling this function the line has the length shrink_factor
   * time the old length.
   * \param shrink_factor element [0,1[, defines how much the length of the line
   *is reduced 1=>no reduction 0.5 => line length reduced by factor 2.
   */
  void shrinkSymetricRel(const double shrink_factor) {
    shrinkSymetricAbs(0.5 * (1 - shrink_factor) * length());
  }

  /*!
   * \brief flipOrientation flips start and end point
   */
  void flipOrientation() {
    const auto temp_start = start;
    start = end;
    end = temp_start;
  }

  /*!
   * \brief sortForDimension  sorts start and end point of the line such that
   * the start point has a smaller or equal value in dimension dim than the end
   * point of the line.
   * \param dim index of the dimension to use
   */
  void sortForDimension(const std::size_t dim) {
    if (start[dim] > end[dim]) {
      const auto s = start;
      start = end;
      end = s;
    }
  }

  /*!
   * \brief sortForDimensionInverse sorts start and end point of the line such
   * that
   * the start point has a larger or equal value in dimension dim than the end
   * point of the line.
   * \param dim index of the dimension to use
   */
  void sortForDimensionInverse(const std::size_t dim) {
    if (start[dim] < end[dim]) {
      const auto s = start;
      start = end;
      end = s;
    }
  }

  /*!
   * \brief getMaxDistantPointOnLine
   * \param p Point
   * \return returns the point on the line that has the largest euclidean
   * distance from point p provided as argument.
   */
  EigenVectorTypeD getMaxDistantPointOnLine(const EigenVectorTypeD& p) {
    return ((p - start).squaredNorm() > (p - end).squaredNorm()) ? start : end;
  }
};


// forward declaration
class Line3d;

/******************************************************************************
 *Line2D***********************************************************************
 *****************************************************************************/
/*!
 * \brief The Line2d class inherits from Line class and represents a
 * 2-dimensional line. Eigen::Vector2d is used to represent the start and the
 * end point of the line.
 */
class Line2d : public Line<Eigen::Vector2d> {

 public:
  /*!
   * \brief Line2d constructs a valid 2-dimensional line with start and end
   * point
   * \param start start point of the line
   * \param end end point of the line
   */
  Line2d(const Eigen::Vector2d& start, const Eigen::Vector2d& end);

  Line2d() = default;

  /*!
   * \brief intersectInfiniteRange calculates the intersection of 2 lines
   * assuming both of them are infinitely long.
   * \param other 2nd line for intersection calculation
   * \return returns a boost::optional which contains the intersection point if
   * the calculation was successful (lines are not parallel)
   */
  boost::optional<Eigen::Vector2d> intersectInfiniteRange(const Line2d& other) const;

  /*!
   * \brief intersectInBounds calculates the intersection point of 2 lines which
   * have both finite length.
   * \param other 2nd line for intersection calculation
   * \return returns a boost::optional which contains the intersection point if
   * an intersection point exists
   */
  boost::optional<Eigen::Vector2d> intersectInBounds(const Line2d& other) const;


  /*!
   * \brief normalVectorLeftUsingLeftCoord
   * \return returns directionNormalized() rotated 90° in mathmatical positive
   * direction using left handed coordinates (like ImagePoints use).
   */
  ImagePointExact normalVectorLeftUsingLeftCoord() const;


  /*!
   * \brief normalVectorRightUsingLeftCoord
   * \return returns directionNormalized() rotated 90° in mathmatical negative
   * direction (using left handed coordinates)
   */
  ImagePointExact normalVectorRightUsingLeftCoord() const;


  /*!
   * \brief isPartiallyInsidePolygon
   * \param polygon_points
   * \return returns true if the line is partially inside (intersects with) the
   * polygon provided.
   */
  bool isPartiallyInsidePolygon(const ImagePointsExact& polygon_points) const;

  /*!
   * \brief asGroundLineInVehicle applies the camera-transformtaion
   * imageToGround to both the start and the end point of the line
   * \param camera_transformation
   * \return returns imageToGround transformed 3dline
   */
  Line3d asGroundLineInVehicle(const common::CameraTransformation* const camera_transformation) const;



  /*!
   * \brief calcConsensusSetInfLine calculates the comsesnus points in pts which
   * have a smaller distance to the line than pixel_distance_thld.
   * \param pts all points to be tested
   * \param pixel_distance_thld threshold whether a point is considered
   * comsesnus or not
   * \param comsesnus all points from pts that are considered consensus.
   * \param non_comsesnus all points from pts that are considered non_consensus.
   */
  void calcConsensusSetInfLine(const ImagePointsExact& pts,
                               const double pixel_distance_thld,
                               ImagePointsExact& comsesnus,
                               ImagePointsExact& non_comsesnus) const;


  /*!
   * \brief estimate calculates an approcimation of a 2-dim_Line based on the
   * points porvided.
   * \param points points used to fit a line
   * \return returns a 2-dim-line (mdoel) for the given points
   */
  static Line2d estimate(const ImagePointsExact& points);
};
using Lines2d = std::vector<Line2d>;

/******************************************************************************
 *Line3D***********************************************************************
 *****************************************************************************/
/*!
 * \brief The Line3d class inherits from Line class and represents a
 * 3-dimensional line. Eigen::Vector3d is used to represent the start and the
 * end point of the line.
 */
class Line3d : public Line<Eigen::Vector3d> {
 public:
  /*!
   * \brief Line3d constructs a 3-d line with start and end point
   * \param start point
   * \param end point
   */
  Line3d(const Eigen::Vector3d& start, const Eigen::Vector3d& end);

  Line3d() = default;

  /*!
   * \brief asImageLine transforms the 3d-line in a 2d line using the
   * camera-transformation transformVehicleToImageExact. Note: it is asumed that
   * the 3d line is given in Vehicle coordinates.
   * \param camera_transformation camera_transformation
   * \return the transformation result of "transformVehicleToImageExact"
   */
  Line2d asImageLine(const common::CameraTransformation* const camera_transformation) const;
};
using Lines3d = std::vector<Line3d>;


/******************************************************************************
 ***** Interval**************************************************************
 *****************************************************************************/
template <typename T>
struct Interval {
  /*!
   * \brief Interval constructs a interval containing a start and an end point
   * \param start point
   * \param end point
   */
  Interval(T start, T end) : start(start), end(end) {}

  /*!
   * \brief sort ensures the start point compares smaller or equal to the end
   * point
   */
  void sort() {
    const T s = start;
    const T e = end;
    start = std::min(s, e);
    end = std::max(s, e);
  }

  /*!
   * \brief insideInc
   * \param value which is tested
   * \return returns true if value is inside the closed interval [start, end]
   */
  bool insideInc(const T& value) const {
    return (start <= value && value <= end);
  }

  /*!
   * \brief insideExc
   * \param value which is tested
   * \return returns true if value is inside the open interval (start, end)
   */
  bool insideExc(const T& value) const {
    return (start < value && value < end);
  }

  /*!
   * \brief width of the intervaö
   * \return returns end-start
   */
  double width() const { return end - start; }

  T start;
  T end;
};


/******************************************************************************
 *PolynomialWithXRange*********************************************************
 *****************************************************************************/
/*!
 * \brief The PolynomialWithXRange struct contains a polynomial and an interval
 * [x_min, x_max] whithin which the polynomial is valid
 */
struct PolynomialWithXRange {
  PolynomialWithXRange(const common::DynamicPolynomial& polynomial,
                       const Interval<double>& x_range)
      : polynomial(polynomial), x_range(x_range) {}

  common::DynamicPolynomial polynomial;
  Interval<double> x_range;
};


/******************************************************************************
 ***** Roi Creation Params*****************************************************
 *****************************************************************************/
struct RoiParams {
  RoiParams(ParameterInterface* const parameters_ptr_,
            const std::string& parameter_namespace,
            const common::CameraTransformation* const camera_transformation_) {

    // paramter using roi creation based on road watcher cluster
    const ParameterString<double> ROI_HEIGHT_IN_M(parameter_namespace +
                                                  "/roi_height_in_m");
    const ParameterString<double> AREA_ROI_EXTENSION_IN_M(
        parameter_namespace + "/area_roi_extension_in_m");

    const ParameterString<double> MIN_ROI_WIDTH_IN_M(parameter_namespace +
                                                     "/min_roi_width_in_m");
    const ParameterString<double> MIN_ROI_HEIGHT_IN_M(parameter_namespace +
                                                      "/min_roi_length_in_m");
    const ParameterString<double> MAX_ROI_WIDTH_IN_M(parameter_namespace +
                                                     "/max_roi_width_in_m");
    const ParameterString<double> MAX_ROI_LENGTH_IN_M(parameter_namespace +
                                                      "/max_roi_length_in_m");


    const ParameterString<int> FIELD_OF_VISION_BOTTOM("field_of_vision/bottom");



    parameters_ptr_->registerParam(ROI_HEIGHT_IN_M);
    parameters_ptr_->registerParam(AREA_ROI_EXTENSION_IN_M);


    parameters_ptr_->registerParam(MIN_ROI_WIDTH_IN_M);
    parameters_ptr_->registerParam(MIN_ROI_HEIGHT_IN_M);
    parameters_ptr_->registerParam(MAX_ROI_WIDTH_IN_M);
    parameters_ptr_->registerParam(MAX_ROI_LENGTH_IN_M);

    parameters_ptr_->registerParam(FIELD_OF_VISION_BOTTOM);

    roi_height_in_m = parameters_ptr_->getParam(ROI_HEIGHT_IN_M);
    area_roi_extension_in_m =
        static_cast<float>(parameters_ptr_->getParam(AREA_ROI_EXTENSION_IN_M));

    min_roi_width_in_m =
        static_cast<float>(parameters_ptr_->getParam(MIN_ROI_WIDTH_IN_M));
    min_roi_length_in_m =
        static_cast<float>(parameters_ptr_->getParam(MIN_ROI_HEIGHT_IN_M));
    max_roi_width_in_m =
        static_cast<float>(parameters_ptr_->getParam(MAX_ROI_WIDTH_IN_M));
    max_roi_length_in_m =
        static_cast<float>(parameters_ptr_->getParam(MAX_ROI_LENGTH_IN_M));



    const int fov_bottom = parameters_ptr_->getParam(FIELD_OF_VISION_BOTTOM);
    const ImagePoint fov_bottom_ip(640, fov_bottom);
    const VehiclePoint closest_ground_point =
        camera_transformation_->transformImageToGround(fov_bottom_ip);
    field_of_vision_min_x = closest_ground_point[0];
  }

  double roi_height_in_m;
  float area_roi_extension_in_m;
  float min_roi_width_in_m;
  float min_roi_length_in_m;
  float max_roi_width_in_m;
  float max_roi_length_in_m;

  // note parameter from global scope used
  double field_of_vision_min_x;  // vehicle coordinates
};



/******************************************************************************
 ***** Feature Point Detection Params ****************************************
 *****************************************************************************/
struct FeaturePointDetectionParams {
  FeaturePointDetectionParams(ParameterInterface* const parameters_ptr_,
                              const std::string& parameter_namespace) {


    // Paramter for step detection
    const ParameterString<double> STEP_THLD(parameter_namespace + "/step_thld");
    const ParameterString<int> STEP_FUNC_SIZE(parameter_namespace +
                                              "/step_func_size");
    // Parameter for ScanLine generation
    const ParameterString<double> MIN_SCAN_LINE_LENGTH(
        parameter_namespace + "/min_scan_line_length_in_pxl");
    const ParameterString<double> SCAN_LINE_DENSITY(parameter_namespace +
                                                    "/scan_line_density");

    // lane line point removal
    const ParameterString<double> LANE_LINE_REMOVAL_THLD_IN_M(
        parameter_namespace + "/lane_line_removal_thld_in_m");

    const ParameterString<double> BASE_ESTIMATE_SCAN_LINE_SPACING_IN_M(
        parameter_namespace + "/base_estimate_scan_line_spacing_in_m");
    const ParameterString<double> BASE_ESTIMATE_NUMBER_OF_SCAN_LINES(
        parameter_namespace + "/base_estimate_number_of_scan_lines");
    const ParameterString<double> BASE_ESTIMATE_SCAN_LINE_EDGE_OVERLAP_IN_M(
        parameter_namespace + "/base_estimate_scan_line_edge_overlap_in_m");



    parameters_ptr_->registerParam(STEP_THLD);
    parameters_ptr_->registerParam(STEP_FUNC_SIZE);

    parameters_ptr_->registerParam(MIN_SCAN_LINE_LENGTH);
    parameters_ptr_->registerParam(SCAN_LINE_DENSITY);

    parameters_ptr_->registerParam(LANE_LINE_REMOVAL_THLD_IN_M);

    parameters_ptr_->registerParam(BASE_ESTIMATE_SCAN_LINE_SPACING_IN_M);
    parameters_ptr_->registerParam(BASE_ESTIMATE_NUMBER_OF_SCAN_LINES);
    parameters_ptr_->registerParam(BASE_ESTIMATE_SCAN_LINE_EDGE_OVERLAP_IN_M);


    step_thld = static_cast<float>(parameters_ptr_->getParam(STEP_THLD));
    step_func_size = parameters_ptr_->getParam(STEP_FUNC_SIZE);

    min_scan_line_length_in_pxl = parameters_ptr_->getParam(MIN_SCAN_LINE_LENGTH);
    scan_line_density = parameters_ptr_->getParam(SCAN_LINE_DENSITY);

    lane_line_removal_thld_in_m = parameters_ptr_->getParam(LANE_LINE_REMOVAL_THLD_IN_M);

    base_estimate_scan_line_spacing_in_m =
        parameters_ptr_->getParam(BASE_ESTIMATE_SCAN_LINE_SPACING_IN_M);
    base_estimate_number_of_scan_lines =
        parameters_ptr_->getParam(BASE_ESTIMATE_NUMBER_OF_SCAN_LINES);
    base_estimate_scan_line_edge_overlap_in_m =
        parameters_ptr_->getParam(BASE_ESTIMATE_SCAN_LINE_EDGE_OVERLAP_IN_M);
  }

  // params step detection
  float step_thld;
  int step_func_size;

  // params scan line generation
  double min_scan_line_length_in_pxl;
  double scan_line_density;

  // params lane line point removal
  double lane_line_removal_thld_in_m;

  // parameter for base_hull_estimation
  double base_estimate_scan_line_spacing_in_m;
  std::size_t base_estimate_number_of_scan_lines;
  double base_estimate_scan_line_edge_overlap_in_m;
};


/******************************************************************************
 ***** Foot Finder Params*****************************************************
 *****************************************************************************/
struct FootFinderParams {
  FootFinderParams(ParameterInterface* const parameters_ptr_,
                   const std::string& parameter_namespace) {
    const ParameterString<double> EPSILON_NEWTON_METHOD(
        parameter_namespace + "/epsilon_newton_method");
    const ParameterString<int> MAX_ITERATIONS_NEWTON_METHOD(
        parameter_namespace + "/max_iterations_newton_method");

    parameters_ptr_->registerParam(EPSILON_NEWTON_METHOD);
    parameters_ptr_->registerParam(MAX_ITERATIONS_NEWTON_METHOD);

    epsilon_newton_method = parameters_ptr_->getParam(EPSILON_NEWTON_METHOD);
    max_iterations_newton_method = static_cast<unsigned int>(
        parameters_ptr_->getParam(MAX_ITERATIONS_NEWTON_METHOD));
  }

  double epsilon_newton_method;
  unsigned int max_iterations_newton_method;
};


/******************************************************************************
 ***** Ransac Parmas ********************************************************
 *****************************************************************************/
struct RansacParams {
  RansacParams(ParameterInterface* const parameters_ptr_, const std::string& parameter_namespace) {
    const ParameterString<int> MAX_ITERATION(parameter_namespace +
                                             "/max_iteration");
    const ParameterString<int> MIN_DATA_SET_SIZE(parameter_namespace +
                                                 "/min_data_set_size");
    const ParameterString<double> MIN_MODEL_SCORE_THLD(parameter_namespace +
                                                       "/min_model_score_thld");
    const ParameterString<int> MAX_IT_RAND_L_LINES_PTS_SELECT(
        parameter_namespace + "/max_it_rand_L_lines_pts_select");
    const ParameterString<int> MAX_IT_2ND_GND_MODEL_EXTENSION(
        parameter_namespace + "/max_it_2nd_gnd_model_extension");

    parameters_ptr_->registerParam(MAX_ITERATION);
    parameters_ptr_->registerParam(MIN_DATA_SET_SIZE);
    parameters_ptr_->registerParam(MIN_MODEL_SCORE_THLD);
    parameters_ptr_->registerParam(MAX_IT_RAND_L_LINES_PTS_SELECT);
    parameters_ptr_->registerParam(MAX_IT_2ND_GND_MODEL_EXTENSION);

    max_iteration = parameters_ptr_->getParam(MAX_ITERATION);
    min_data_set_size = parameters_ptr_->getParam(MIN_DATA_SET_SIZE);
    min_model_score_thld = parameters_ptr_->getParam(MIN_MODEL_SCORE_THLD);
    max_it_rand_L_lines_pts_select =
        parameters_ptr_->getParam(MAX_IT_RAND_L_LINES_PTS_SELECT);
    max_it_2nd_gnd_model_extension =
        parameters_ptr_->getParam(MAX_IT_2ND_GND_MODEL_EXTENSION);
  }

  std::size_t max_iteration;
  std::size_t min_data_set_size;
  double min_model_score_thld;
  std::size_t max_it_rand_L_lines_pts_select;
  std::size_t max_it_2nd_gnd_model_extension;
};



/******************************************************************************
 ***** Obstacle Params*****************************************************
 *****************************************************************************/
struct ObstacleParams {

  ObstacleParams(ParameterInterface* const parameters_ptr_,
                 const std::string& parameter_namespace) {
    const ParameterString<double> MIN_OBSTACLE_WIDTH(parameter_namespace +
                                                     "/min_obstacle_width");
    const ParameterString<double> MIN_OBSTACLE_HEIGHT(parameter_namespace +
                                                      "/min_obstacle_height");
    const ParameterString<double> MIN_OBSTACLE_LENGTH(parameter_namespace +
                                                      "/min_obstacle_length");
    const ParameterString<double> MAX_OBSTACLE_WIDTH(parameter_namespace +
                                                     "/max_obstacle_width");
    const ParameterString<double> MAX_OBSTACLE_HEIGHT(parameter_namespace +
                                                      "/max_obstacle_height");
    const ParameterString<double> MAX_OBSTACLE_LENGTH(parameter_namespace +
                                                      "/max_obstacle_length");

    const ParameterString<double> DEFAULT_OBSTACLE_SIZE(
        parameter_namespace + "/default_obstacle_size");

    parameters_ptr_->registerParam(MIN_OBSTACLE_WIDTH);
    parameters_ptr_->registerParam(MIN_OBSTACLE_HEIGHT);
    parameters_ptr_->registerParam(MIN_OBSTACLE_LENGTH);
    parameters_ptr_->registerParam(MAX_OBSTACLE_WIDTH);
    parameters_ptr_->registerParam(MAX_OBSTACLE_HEIGHT);
    parameters_ptr_->registerParam(MAX_OBSTACLE_LENGTH);

    parameters_ptr_->registerParam(DEFAULT_OBSTACLE_SIZE);

    min_width = parameters_ptr_->getParam(MIN_OBSTACLE_WIDTH);
    min_height = parameters_ptr_->getParam(MIN_OBSTACLE_HEIGHT);
    min_length = parameters_ptr_->getParam(MIN_OBSTACLE_LENGTH);
    max_width = parameters_ptr_->getParam(MAX_OBSTACLE_WIDTH);
    max_height = parameters_ptr_->getParam(MAX_OBSTACLE_HEIGHT);
    max_length = parameters_ptr_->getParam(MAX_OBSTACLE_LENGTH);

    default_size = parameters_ptr_->getParam(DEFAULT_OBSTACLE_SIZE);
  }

  // all values in m
  double min_width;
  double min_height;
  double min_length;

  double max_width;
  double max_height;
  double max_length;


  // obstacle base size set when no estimation of the actual size is possible
  double default_size;
};

/******************************************************************************
 ***** ModelParams*****************************************************
 *****************************************************************************/

struct ModelParams {

  ModelParams(ParameterInterface* const parameters_ptr_, const std::string& parameter_namespace) {
    const ParameterString<double> MIN_DISTANCE_THLD_PIXEL(
        parameter_namespace + "/distance_thld_pixel");

    const ParameterString<std::vector<double>> B0_EDGE_CNT_EVAL_FCT(
        parameter_namespace + "/b0_edge_cnt_eval_fct");
    const ParameterString<std::vector<double>> V0_EDGE_CNT_EVAL_FCT(
        parameter_namespace + "/v0_edge_cnt_eval_fct");
    const ParameterString<std::vector<double>> B1_EDGE_CNT_EVAL_FCT(
        parameter_namespace + "/b1_edge_cnt_eval_fct");

    const ParameterString<std::vector<double>> PENALIZED_PTS_EVAL_FCT(
        parameter_namespace + "/penalized_pts_eval_fct");
    const ParameterString<std::vector<double>> GRAY_MEAN_EVAL_FCT(
        parameter_namespace + "/gray_mean_eval_fct");
    const ParameterString<std::vector<double>> GRAY_STDDEV_EVAL_FCT(
        parameter_namespace + "/gray_stddev_eval_fct");
    const ParameterString<std::vector<double>> VERTICAL_ANGLE_EVAL_FCT(
        parameter_namespace + "/vertical_angle_image_eval_fct");
    const ParameterString<std::vector<double>> GROUND_LINE_ANGLE_EVAL_FCT(
        parameter_namespace + "/ground_lines_angle_eval_fct");


    const ParameterString<double> INCOMPLETE_MODEL_PENALTY(
        parameter_namespace + "/incomplete_model_penalty");

    const ParameterString<double> MAX_INCLINE_ANGLE(parameter_namespace +
                                                    "/max_incline_angle");
    const ParameterString<double> VERTICAL_ANGLE_TOLERANCE_IMAGE(
        parameter_namespace + "/vertical_angle_tolerance_image");
    const ParameterString<double> GROUND_LINE_PERPENDICULAR_ANGLE_TOLERANCE(
        parameter_namespace + "/ground_line_perpendicular_angle_tolerance");
    const ParameterString<double> GNDLINE2_VISIBLE_ANGLE_THLD(
        parameter_namespace + "/gndline2_visible_angle_thld");
    const ParameterString<double> HISTOGRAM_INIT_VALUE(parameter_namespace +
                                                       "/histogram_init_value");

    const ParameterString<double> MIN_MODEL_SIZE_IN_M(parameter_namespace +
                                                      "/min_model_size_in_m");



    // register params
    parameters_ptr_->registerParam(MIN_DISTANCE_THLD_PIXEL);

    parameters_ptr_->registerParam(B0_EDGE_CNT_EVAL_FCT);
    parameters_ptr_->registerParam(V0_EDGE_CNT_EVAL_FCT);
    parameters_ptr_->registerParam(B1_EDGE_CNT_EVAL_FCT);
    parameters_ptr_->registerParam(PENALIZED_PTS_EVAL_FCT);
    parameters_ptr_->registerParam(GRAY_MEAN_EVAL_FCT);
    parameters_ptr_->registerParam(GRAY_STDDEV_EVAL_FCT);
    parameters_ptr_->registerParam(VERTICAL_ANGLE_EVAL_FCT);
    parameters_ptr_->registerParam(GROUND_LINE_ANGLE_EVAL_FCT);

    parameters_ptr_->registerParam(INCOMPLETE_MODEL_PENALTY);

    parameters_ptr_->registerParam(MAX_INCLINE_ANGLE);
    parameters_ptr_->registerParam(VERTICAL_ANGLE_TOLERANCE_IMAGE);
    parameters_ptr_->registerParam(GROUND_LINE_PERPENDICULAR_ANGLE_TOLERANCE);
    parameters_ptr_->registerParam(GNDLINE2_VISIBLE_ANGLE_THLD);
    parameters_ptr_->registerParam(HISTOGRAM_INIT_VALUE);

    parameters_ptr_->registerParam(MIN_MODEL_SIZE_IN_M);



    // load params
    distance_thld_pixel = parameters_ptr_->getParam(MIN_DISTANCE_THLD_PIXEL);

    b0_edge_cnt_eval_fct = loadFctValues(parameters_ptr_, B0_EDGE_CNT_EVAL_FCT);
    v0_edge_cnt_eval_fct = loadFctValues(parameters_ptr_, V0_EDGE_CNT_EVAL_FCT);
    b1_edge_cnt_eval_fct = loadFctValues(parameters_ptr_, B1_EDGE_CNT_EVAL_FCT);
    penalized_pts_eval_fct = loadFctValues(parameters_ptr_, PENALIZED_PTS_EVAL_FCT);
    gray_mean_eval_fct = loadFctValues(parameters_ptr_, GRAY_MEAN_EVAL_FCT);
    gray_stddev_eval_fct = loadFctValues(parameters_ptr_, GRAY_STDDEV_EVAL_FCT);
    vertical_angle_image_eval_fct = loadFctValues(parameters_ptr_, VERTICAL_ANGLE_EVAL_FCT);
    ground_lines_angle_eval_fct = loadFctValues(parameters_ptr_, GROUND_LINE_ANGLE_EVAL_FCT);

    incomplete_model_penalty = parameters_ptr_->getParam(INCOMPLETE_MODEL_PENALTY);

    max_incline_angle = parameters_ptr_->getParam(MAX_INCLINE_ANGLE);
    vertical_angle_tolerance_image =
        parameters_ptr_->getParam(VERTICAL_ANGLE_TOLERANCE_IMAGE);
    ground_line_perpendicular_angle_tolerance =
        parameters_ptr_->getParam(GROUND_LINE_PERPENDICULAR_ANGLE_TOLERANCE);
    gndline2_visible_angle_thld = parameters_ptr_->getParam(GNDLINE2_VISIBLE_ANGLE_THLD);
    histogram_init_value = parameters_ptr_->getParam(HISTOGRAM_INIT_VALUE);

    min_model_size_in_m = parameters_ptr_->getParam(MIN_MODEL_SIZE_IN_M);
  }



  // threshold used to calculate consesus set
  double distance_thld_pixel;

  // function points (which will be linearly interpolated) for score evaluation
  std::vector<Eigen::Vector2d> b0_edge_cnt_eval_fct;
  std::vector<Eigen::Vector2d> v0_edge_cnt_eval_fct;
  std::vector<Eigen::Vector2d> b1_edge_cnt_eval_fct;
  std::vector<Eigen::Vector2d> penalized_pts_eval_fct;
  std::vector<Eigen::Vector2d> gray_mean_eval_fct;
  std::vector<Eigen::Vector2d> gray_stddev_eval_fct;
  std::vector<Eigen::Vector2d> vertical_angle_image_eval_fct;
  std::vector<Eigen::Vector2d> ground_lines_angle_eval_fct;
  // penalty factor for a simple L-model (that is not extended with a 2nd ground
  // line)
  double incomplete_model_penalty;

  // parameters for angle constraints
  double max_incline_angle;
  // this param adds a tolerance to the 2d angle constraints in image
  // coordintes for vertical seclection
  double vertical_angle_tolerance_image;
  // both ground lines of obstacle should have 90° => this param defines the
  // accepted tolerance
  double ground_line_perpendicular_angle_tolerance;

  // 2nd ground line must not be hidden behind obstacle vertical
  // must have at least an angle of this parameter between gnd line and vertical
  // prjected in 2d image
  double gndline2_visible_angle_thld;

  // init value for histogram used for entropy calculateion
  double histogram_init_value;

  // min dimension in meter for the obstacle model => should be smaller or equal
  // to the minimum obstacle width/length/height
  double min_model_size_in_m;

 private:
  static std::vector<Eigen::Vector2d> loadFctValues(
      const ParameterInterface* const parameters_ptr_,
      const ParameterString<std::vector<double>>& parameter_string) {

    const std::vector<double> fct_data = parameters_ptr_->getParam(parameter_string);
    ROS_ASSERT(fct_data.size() % 2 == 0);
    std::vector<Eigen::Vector2d> eval_fct;
    eval_fct.reserve(fct_data.size() / 2);
    for (std::size_t i = 0; i < (fct_data.size() - 1); i += 2) {
      eval_fct.emplace_back(fct_data[i], fct_data[i + 1]);
    }
    return eval_fct;
  }
};

/******************************************************************************
 ***** ObstacleVertices*****************************************************
 *****************************************************************************/
struct ObstacleVertices {


  ObstacleVertices(const VehiclePoints& vertices,
                   const std::vector<Obstacle::DetectionState>& vertices_detection_state)
      : vertices(vertices), vertices_detection_state(vertices_detection_state) {
    assert(vertices.size() == 4);
    assert(vertices_detection_state.size() == 4);
  }
  // contains 4 obstacle vertices (= base_hull_polygon)
  VehiclePoints vertices;
  // contains info whether  given vertex has actually been detected
  std::vector<Obstacle::DetectionState> vertices_detection_state;
};


/******************************************************************************
 ***** ObstacleFrontModel*****************************************************
 *****************************************************************************/
class ObstacleFrontModel {

 public:
  /*!
   * \brief The ModelType enum defines in which state the currnt model is.
   */
  enum class ModelType { NONE = 0, L = 1, L_2ND_GND_LINE = 2 };
  /*!
   * \brief hist_size defines the number of bins used for the histogram which is
   * used to judge the distribution of points along the a single line of the
   * model
   */
  static const std::size_t hist_size = 3;


  ObstacleFrontModel() = default;


  /*!
   * \brief reset internal model counters and scores
   */
  void reset();

  //void resetDebugData();

  /*!
   * \brief resetCnts reset internal counters and hists with the specified model
   * index
   * \param index model_index (either 0 or 1)
   */
  void resetCnts(const std::size_t index);

  /*!
   * \brief setModelState set the ModelType. Note: This thus not change the
   * underlying data, only the wey it is interpreted.
   * \param model_state which should be applied.
   */
  void setModelState(const ModelType& model_state);


  /*!
   * \brief generateModelL generates a Model of type "L" (model index 0) with
   * the provided data
   * \param bottom0 a ground line candidate of the obstacle
   * \param vertical0 a vertical candidate of the obstacle
   * \param img_size the size of the image complete
   * \param model_params model_params object
   * \param camera_transformation camera_transformation
   * \return true if a valid model was generated, false otherwise
   */
  bool generateModelL(const Line2d& bottom0,
                      const Line2d& vertical0,
                      const cv::Size& img_size,
                      const ModelParams& model_params,
                      const common::CameraTransformation* const camera_transformation,
                      const bool vertical_from_left_dataset);


  /*!
   * \brief extendModelWith2ndGroundLine extends the model with a second
   * ground_line (bottom2) by generating a second L-type model (model_index =
   * 1).
   * \param bottom2 a geound_line_candidate of the obstacle
   * \param model_params model_params object
   * \param img_size the size of the image complete
   * \param camera_transformation camera_transformation
   * \return true if the extendsion was succesful (all requirements on bottom2
   * are met). False otherwise
   */
  bool extendModelWith2ndGroundLine(const Line2d bottom2,
                                    const ModelParams& model_params,
                                    const cv::Size& img_size,
                                    const common::CameraTransformation* const camera_transformation);



  /*!
   * \brief getGroundVertices returns the ground vertices in vehicle coordinates
   * based on the current model. Note that these do not yet try to estimate the
   * true obstacle size
   * \param camera_transformation camera_transformation
   * \return ObstacleVertices
   */
  ObstacleVertices getGroundVertices(const common::CameraTransformation* const camera_transformation) const;



  /*!
   * \brief calcConsensusSet calculates the consensus set (which points in b_pts
   * support the sub_model specified by model_index. All internal counters are
   * updated so a call to evaluateScore will produce a valid result for the
   * given sub_model. (In this case no vector for the consensus points will be
   * created)
   * \param b_pts data points which are to be checked whether they are consensus
   * to the models ground line
   * \param model_params object containing all rosprams needed
   * \param model_index specifies which submodel (0 or 1) is used.
   * */
  void calcConsensusSet(const ImagePointsExact& b_pts,
                        const ModelParams& model_params,
                        const std::size_t model_index);



  /*!
   * \brief calcConsensusSet calculates the consensus set (which points in
   * b_pts, v_pts support the sub_model specified by model_index). All
   * internalcounters areupdated so a call to evaluateScore will produces a
   * valid result for thegiven sub_model. (In this case no vector for the
   * consensus points will becreated)
   * \param b_pts data points which are to be checked whether they are consensus
   * to the models ground linene
   * \param v_pts data points which are to be checked whether they are consensus
   * to the models vertical
   * \param out_not_b_inf_line all_points in b_pts which are not part of
   * out_b_consensus
   * \param model_params object containing all rosprams needed
   * \param model_index specifies which submodel (0 or 1) is used.
   */
  void calcConsensusSet(const ImagePointsExact& b_pts,
                        const ImagePointsExact& v_pts,
                        ImagePointsExact& out_not_b_inf_line,
                        const ModelParams& model_params,
                        const std::size_t model_index);



  static void incrementLineSupportHistogram(std::array<int, hist_size>& hist_cnts,
                                            const Line2d& line,
                                            const ImagePointExact& p);



  static double getNormalizedEntropyMeasure(const std::array<int, hist_size>& hist_cnts,
                                            const std::size_t nr_pts_in_hist,
                                            const ModelParams& model_params);


  ObstacleFrontModel::ModelType getModelType() const;



  void meanStdDevOfGrayInsideModel(const cv::Mat& image_gray8,
                                   double& out_mean,
                                   double& out_std_dev,
                                   const std::size_t model_index) const;



  double evaluateScore(const cv::Mat& image_complete,
                       const ModelParams& model_params,
                       const common::CameraTransformation* const camera_transformation);


  // note:: also updates mode resent total score
  void setPenaltyFactor(const double factor);
  // note:: also updates mode resent total score
  void resetPenaltyFactor();

  double getMostRecentScore() const;

  double getFirstSubModelRecentScore() const;

  double getModelArea() const;


  Line2d getBottomLine(const std::size_t index) const;
  Line2d getTopLine(const std::size_t index) const;
  Line2d getVerticalLeft(const std::size_t index) const;
  Line2d getVerticalRight(const std::size_t index) const;


  std::vector<cv::Point> asConvexPolygonCV(const std::size_t index) const;


  bool isSecondGroundLineClearlyVisible() const;


  void rosDebugAll() const;


  bool isInsideFrontModel(const ImagePointExact& p,
                          const double pixel_distance_thld,
                          const std::size_t index) const;


  bool isSecondGroundLineClearlyVisible(const double angle_threshold,
                                        const common::CameraTransformation* const camera_transformation) const;

 protected:
  double evaluateScore(const cv::Mat& image_complete,
                       const std::size_t index,
                       const ModelParams& model_params,
                       const common::CameraTransformation* const camera_transformation);



  bool supportsModel(const Line2d& line, const ImagePointExact& p, const double pixel_distance_thld) const;

  bool supportsInfLine(const Line2d& line,
                       const ImagePointExact& p,
                       const double pixel_distance_thld) const;



  // Model evaluation
  double total_score;
  std::array<double, 2> score{{0, 0}};

  // global penalty factor
  double penalty_factor = 1;

  // For L type model vectors conatininf one element each
  // if additional ground line is set each vector contains 2 elements
  std::array<std::size_t, 2> b_set_sizes{{0, 0}};
  std::array<std::size_t, 2> v_set_sizes{{0, 0}};
  std::array<std::size_t, 2> nr_penaltized{{0, 0}};

  std::array<Line2d, 2> b;
  std::array<Line2d, 2> v;


  std::array<bool, 2> vertical_is_left{{false, false}};


  std::array<std::array<int, hist_size>, 2> b_hists;
  std::array<std::array<int, hist_size>, 2> v_hists;
  ModelType model_type = ModelType::NONE;

  // save some more variabales for debugging
//  std::array<double, 2> debug_edge_score_b{{0, 0}};
//  std::array<double, 2> debug_edge_score_v{{0, 0}};
//  std::array<double, 2> debug_peneltizde_score{{0, 0}};
//  std::array<double, 2> debug_gray_mean{{0, 0}};
//  std::array<double, 2> debug_gray_mean_score{{0, 0}};
//  std::array<double, 2> debug_gray_std_dev{{0, 0}};
//  std::array<double, 2> debug_gray_std_dev_score{{0, 0}};
//  std::array<double, 2> debug_b_entropie_score{{0, 0}};
//  std::array<double, 2> debug_v_entropie_score{{0, 0}};
//  std::array<double, 2> debug_vertical_angle_score{{0, 0}};
//  std::array<double, 2> debug_ground_angle_score{{0, 0}};
  // debugging end
};

using ObstacleFrontModels = std::vector<ObstacleFrontModel>;


/******************************************************************************
 *ObstacleCLassifierNew***************************************************
 *****************************************************************************/
using Obstacles = std::vector<Obstacle>;

class ObstacleClassifierNew : public virtual Classifier {
 public:
  ObstacleClassifierNew(const common::CameraTransformation* const camera_transformation,
                        ParameterInterface* const parameters_ptr);

  virtual ~ObstacleClassifierNew() override = default;

  virtual RoadObjects classify(const Features& features) override;

  virtual size_t getClassifierId() const final override;



 protected:
  /*############################################
   *########## Roi defining methods ############
   *############################################*/

  virtual ImagePoints getGroundAreaRoi(const Features& features) const;

  virtual ImagePoints getVolumeRoi(const ImagePoints& area_roi, const cv::Size& img_size) const;


  /*############################################
   *########## Criteria Calulcation ############
   *############################################*/


  virtual bool isLineAngleAcceptedAsVertical(const Line2d& vertical_candidate,
                                             const Features& features) const;


  bool isGroundLine(const Line2d& ground_line_candidate,
                    const Line2d& ref_vertical_b2t_oriented) const;



  VehiclePoint projectOnPolynomial(const common::DynamicPolynomial& middle_lane_polynomial,
                                   const VehiclePoint& point) const;


  virtual VehiclePoint getLaneDirectionVector(const common::DynamicPolynomial& middle_lane_polynomial,
                                              const VehiclePoint& ground_point) const;

  VehiclePoint getPerpendicularLaneVector(const VehiclePoint& ground_point,
                                          const Features& features,
                                          const bool& right_not_left_lane) const;



  /*############################################
   *###### Shape Extraction Methods ##########
   *############################################*/

  /// constants defining different modes for step detection
  enum DetectionMode { ALL_EDGES = 0, BLACK_TO_WHITE = 1, WHITE_TO_BLACK = 2 };



  /*!
   * \brief generateScanLines generates scan lines perpendiculat to the
   * expected edge direction
   * \param rect is the bounding rect of the roi
   * \param edge_angle is the expected angle between the y axis and the
   * expected
   * edge in image coordinates (edge_angle must be in the range [0, pi])
   * \param out_scan_line_spacing output param for the scan line spacing
   * \param out_scan_direction output the scan line direction
   * \return scan_lines perpendicular to the line defined by edge_angle and
   * the top left point od the rect. The scan_lines start and end at the
   * bounds
   * of the rect
   */
  virtual ScanLines generateScanLines(const cv::Rect& rect,
                                      const double edge_angle,
                                      double& out_scan_line_spacing,
                                      ImagePointExact& out_scan_direction) const;


  virtual ImagePoints getFeaturePoints(const cv::Mat& image_complete,
                                       cv::InputArray& roi_defining_points,
                                       const ScanLines& scan_lines,
                                       const DetectionMode& mode) const;


  virtual cv::Rect getExtendedRoiRect(const std::vector<cv::Point>& convex_roi,
                                      const int expand_pixel_even,
                                      const cv::Size& img_size) const;


  VehiclePoint estimateRoiCenter(const VehiclePoints& ground_area_roi) const;

  virtual void getFrontShapeFeaturePoints(const Features& features,
                                          const ImagePoints& area_roi,
                                          ImagePoints& fp_bottom_out,
                                          ImagePoints& fp_vertical_left_out,
                                          ImagePoints& fp_vertical_right_out) const;


  virtual boost::optional<PolynomialWithXRange> getPolynomial(const VehiclePoints& points) const;

  virtual void erasePointsOnLaneLines(ImagePoints& in_out_ips,
                                      const boost::optional<PolynomialWithXRange> poly_left,
                                      const boost::optional<PolynomialWithXRange> poly_middle,
                                      const boost::optional<PolynomialWithXRange> poly_right,
                                      const double rem_distance_m_thld) const;

  void erasePointsOnLaneLines(VehiclePoints& in_out_ips,
                              const boost::optional<PolynomialWithXRange> poly_left,
                              const boost::optional<PolynomialWithXRange> poly_middle,
                              const boost::optional<PolynomialWithXRange> poly_right,
                              const double rem_distance_m_thld) const;



  /*!
   * \brief get2RandomIndices selects 2 random indices from the interval
   * [index_begin, index_end-1]. The Precondition (index_end > index_begin)
   * and
   * ((index_end-index_begin)>=2) must be sattisfied.
   * \param index_begin first valid index
   * \param index_end last_valid_index+1 such that (index_end-index_begin) =
   * number of valid indices
   * \return a pair of random indices within the given range.
   */
  virtual std::pair<std::size_t, std::size_t> get2RandomIndices(const std::size_t index_begin,
                                                                const std::size_t index_end) const;


  template <class UnaryPredicate>  // takes const Line2d& line as argument
  boost::optional<Line2d> getRandomCondTrueLine(const ImagePointsExact& points,
                                                UnaryPredicate isLineAccepted,
                                                const std::size_t max_it) const {
    if (points.size() < 2) {
      return boost::none;
    }
    for (std::size_t k = 0; k < max_it; k++) {
      const auto indices = get2RandomIndices(0, points.size());
      const Line2d candidate_line2d =
          Line2d(points[indices.first], points[indices.second]);
      if (isLineAccepted(candidate_line2d)) {
        return boost::make_optional<Line2d>(candidate_line2d);
      }
    }
    return boost::none;
  }



  virtual boost::optional<Line2d> getRandomGroundLine(const ImagePointsExact& points,
                                                      const Line2d& ref_vertical_b2t_oriented) const;

  virtual boost::optional<Line2d> getRandomVertical(const ImagePointsExact& points,
                                                    const Features& features) const;

  bool randFindAndSet2ndGroundLine(const ImagePointsExact& points,
                                   ObstacleFrontModel& model,
                                   const Features& features) const;



  virtual boost::optional<ObstacleFrontModel> fitObstacleFrontModel(
      const ImagePoints& bottom_pts,
      const ImagePoints& vertical_left_pts,
      const ImagePoints& vertical_right_pts,
      const Features& features) const;



  /*!
   * \brief generateBaseHullEstmateScanLine generates a scan_line with height z0
   * parallel to the given ground line starting short before the previous end
   * point and extending to the maximum possible obstacle size.
   * \param model_ground_line one ground line of the Obstacle model
   * \param z0 the height in Vehicle coordinates at which the scanLine should be
   * created
   * \return the scan_line in image coordinates
   */
  virtual ScanLine generateBaseHullEstmateScanLine(const Line3d& model_ground_line,
                                                   double z0) const;

  /*!
   * \brief estimateObstacleEndPoint estimates a new end-point of the obstacle
   * (used for the base_hull_polygon) assuming that the obstacle model which is
   * used to provide the ground line is correct
   * \param model_ground_line one ground line of the obstacle model which should
   * be extended
   * \param features features provided by road_watcher
   * \return a new end point if one is found
   */
  virtual boost::optional<VehiclePoint> estimateObstacleEndPoint(const Line3d& model_ground_line,
                                                                 const Features& features) const;


  /*!
   * \brief setToObstacleDefaultSize resizes the obstacle to default size.
   * Note: this should only be used if only 1 vertex is detected and width
   * estimation failed.
   * \param vertices vertices to resize
   */
  void setToObstacleDefaultSize(ObstacleVertices& vertices) const;

  /*!
   * \brief estimateBaseHullPolygon estimates a base_hull_polygon by creating
   * extra scan lines parallel ti the obstacle ground line to estimate the true
   * size of the obstale. the function expects the model to be correc.
   * \param model obstacle_model
   * \param features features provided by road_watcher
   * \return an estimation for the obstacle vertices (including information
   * about which vertices have been used by the ModelFit)
   */
  virtual ObstacleVertices estimateBaseHullPolygon(const ObstacleFrontModel& model,
                                                   const Features& features) const;



  // namespaces for parameters
  static const std::string PARAM_NAMESPACE;
  static const std::string PARAM_NAMESPACE_OBSTALCE_DIMENSION;
  static const std::string PARAM_NAMESPACE_FRONT_MODEL;
  static const std::string PARAM_NAMESPACE_FOOT_FINDER;
  static const std::string PARAM_NAMESPACE_ROI;
  static const std::string PARAM_NAMESPACE_FP_DETECT;
  static const std::string PARAM_NAMESPACE_RANSAC;


  const common::CameraTransformation* const camera_transformation_;
  ParameterInterface* const parameters_ptr_;


  // Parameter loaded when Classifier is instantiated
  const FootFinderParams foot_finder_params_;
  const ObstacleParams obstacle_params_;
  const ModelParams model_params_;
  const RoiParams roi_params_;
  const FeaturePointDetectionParams fp_detect_params_;
  const RansacParams ransac_params_;
  const EgoVehicle ego_vehicle_;
};

/*******************************************************************
 **************  Free Functions ***********************************
 *****************************************************************/
double length(const ScanLine& scan_line);

void imagePointsToImagePointsExact(const ImagePoints& ips, ImagePointsExact& out_exact_points);


cv::Rect getExpandedImgCroppedRect(const std::vector<cv::Point>& convex_poly,
                                   const int expand_x,
                                   const int expand_y,
                                   const cv::Size& img_size);



bool prob_true(const double p);


double linearInterpolFctPts(const double value,
                            const std::vector<Eigen::Vector2d>& sorted_function_pts);

/*!
 * \brief transformImageToZPlane transforms an imagepoint ip to a plane parallel
 * to the ground plane but with height z0
 * \param ip imagepoint to transform
 * \param z0 z coordinate of the expected vehicle_point
 * \param camera_transformation camera_transformation
 * \return a VehiclePoint with z=z0 which corresponds to the image point ip
 */
VehiclePoint transformImageToZPlane(const ImagePoint& ip,
                                    const double z0,
                                    const common::CameraTransformation* const camera_transformation);

#endif  // OBSTACLE_CLASSIFIER_NEW_H

}  // namespace road_object_detection
