#ifndef LANE_DETECTION_H
#define LANE_DETECTION_H

#include "common/parameter_interface.h"
#include "common/discretisize.h"

#include "perception_types.h"
#include "feature_extraction.h"
#include "vehicle_point_filter.h"
#include "lane_model.h"

/*!
 * \brief This class perfroms the lane detection.
 */
class LaneDetection {
 public:
  /*!
  * \brief LaneDetection is the constructor. A ros indipendent functionality
  * containing
  * class needs a pointer to a ParameterInterface (in fact a ParameterHandler)
  * to get access to parameters.
  * \param parameters the ParameterInterface
  */
  LaneDetection(ParameterInterface* parameters, FeatureExtraction* featureExtraction);
  LaneDetection(LaneDetection&&) = default;
  virtual ~LaneDetection() = default;

  /*!
   * \brief setInitFlag
   *
   *  Sets Init-flag. After that LaneDetection starts again with initialisation.
   */
  void setInitFlag();

  /*!
   * \brief setStartBox
   *
   *  Sets LaneDetection into starbox mode
   * \param use_start_box_mode wheather to switch in startbox mode.
   */
  void setStartBox(const bool use_start_box_mode);

  /*!
   * \brief startModule
   *
   * starts (<=> activates) the module lane_detection
   */
  void startModule();

  /*!
   * \brief processImage
   *
   * extract the lines out of the given image.
   *
   * \param image the new source image
   */
  virtual LineVehiclePoints processImage(const cv::Mat& image);

  void setFeatureExtraction(FeatureExtraction* feature_extraction) {
    this->feature_extraction = feature_extraction;
  }

  void setMiddleLine(const VehiclePoints& points);

  static constexpr auto max_poly_degree_ =
      common::polynomial::PolynomialDegrees::Cubic + 1;

 protected:
  /*!
   * \brief extractFeatures
   *
   *  Performs feature extraction using the FeatureExtraction object.
   *
   * \param image the new source image
   * \param lane_model line points of previous detected line
   */
  LineVehiclePoints extractFeatures(const cv::Mat& image, const LaneModel& lane_model) const;

  /*!
   * \brief line_data_ detected lines of the last image
   */
  LaneModel lane_model_;

  /*!
   * \brief feature_extraction the feature extraction
   *
   * has to be protected to support fancy debug
   */
  FeatureExtraction* feature_extraction;

 private:
  /*!
   * \brief FITTING_IMAGE_POINTS_TRESHOLD
   *
   * This is the minimal number of points, a line must have for fitting.
   */
  static const ParameterString<int> FITTING_IMAGE_POINTS_TRESHOLD;
  /*!
   * \brief FITTING_FITTING_ERROR_THRESHOLD
   *
   * Threshold to filter fitted lines.
   */
  static const ParameterString<double> FITTING_FITTING_ERROR_THRESHOLD;
  /*!
   * \brief POLYNOMIAL_DEGREE
   *
   * the degree of the target polynoms
   */
  static const ParameterString<int> POLYNOMIAL_DEGREE;
  /*!
   * \brief POLYNOMIAL_SWITCHDOWN
   *
   * the amount the polynomial degree gets reduced on degree adaption
   */
  static const ParameterString<int> POLYNOMIAL_SWITCHDOWN;
  /*!
   * \brief LANE_SEPARATION
   *
   * seperation between two lines of the lane.
   */
  static const ParameterString<double> LANE_SEPARATION;
  /*!
   * \brief LANE_SEPARATION
   *
   * max width difference between lanes to consider a ped island.
   */
  static const ParameterString<double> LANE_WIDTH_DIFF;
  /*!
   * \brief POLYNOMIAL_EQUALITY_EPS
   *
   * epsilon for equality check of two polynoms
   */
  static const ParameterString<double> POLYNOMIAL_EQUALITY_EPS;
  /*!
   * \brief DEGREE_ADAPTION_ENABLE
   *
   * enbales degree adaption for polynomial fit.
   */
  static const ParameterString<bool> DEGREE_ADAPTION_ENABLE;
  /*!
   * \brief DEGREE_ADAPTION_MAX_A
   *
   * the polynomial degree gets smaller if the square component is smaller than
   *this threshold.
   */
  static const ParameterString<double> DEGREE_ADAPTION_MAX_A;
  /*!
   * \brief DEGREE_ADAPTION_MAX_ERROR
   *
   * the polynomial degree gets higher if the if the fitting error is gibber
   *than this threshold.
   */
  static const ParameterString<double> DEGREE_ADAPTION_MAX_ERROR;
  /*!
   * \brief MIDLINE_RECOGNITION_ENABLED
   *
   * enables middline recognition.
   */
  static const ParameterString<bool> MIDLINE_RECOGNITION_ENABLED;
  /*!
   * \brief MIDLINE_RECOGNITION_CLUSTERING_DISTANCE_TRESH
   *
   * the distance threshold that will be used for clustering during midline
   *recognition.
   */
  static const ParameterString<double> MIDLINE_RECOGNITION_CLUSTERING_DISTANCE_THRESH;
  /*!
   * \brief MIDLINE_RECOGNITION_INTRA_CLUSTER_DISTANCE_THRESH
   *
   * the maximal distance a cluster is allowed to have if its part of a middle
   *line.
   */
  static const ParameterString<double> MIDLINE_RECOGNITION_INTRA_CLUSTER_DISTANCE_THRESH;
  /*!
   * \brief MIDLINE_RECOGNITION_REGION_OF_INTEREST
   *
   * the maximal distance between a cluster and the vehicle coordinate system
   *origin in x-direction to be considered.
   */
  static const ParameterString<double> MIDLINE_RECOGNITION_REGION_OF_INTEREST;
  /*!
   * \brief MIDLINE_RECOGNITION_MIN_NUMBER_OF_CLUSTERS;
   *
   * the minimal number of clusters a line needs to have to be considered a
   *midline (in strict mode).
   */
  static const ParameterString<int> MIDLINE_RECOGNITION_MIN_NUMBER_OF_CLUSTERS;
  /*!
   * \brief MIDLINE_RECOGNITION_MIN_NUMBER_OF_POINTS_IN_CLUSTERS;
   *
   * the minimal number point in a cluster needs to have to be considered.
   */
  static const ParameterString<int> MIDLINE_RECOGNITION_MIN_NUMBER_OF_POINTS_IN_CLUSTER;
  /*!
   * \brief USE_POINTSWISE_NORMAL_SHIFT
   *
   * chooses pointwise normal shift during virtual line adding. Alternativly the
   *polynom-based normal shift is used.
   */
  static const ParameterString<bool> USE_POINTSWISE_NORMAL_SHIFT;
  /*!
   * \brief START_BOX_HANDLING_MAX_A
   *
   * the maximal square component of a polynom to be considered as "linear".
   */
  static const ParameterString<double> START_BOX_HANDLING_MAX_A;
  /*!
   * \brief START_BOX_HANDLING_MAX_B
   *
   * the linear component of a polynom to be considered as "linear".
   */
  static const ParameterString<double> START_BOX_HANDLING_MAX_B;
  /*!
   * \brief START_BOX_HANDLING_MAX_ERROR
   *
   * the maximal allowed error of a point to the fitted polynom.
   */
  static const ParameterString<double> START_BOX_HANDLING_MAX_ERROR;
  /*!
   * \brief START_BOX_HANDLING_STARTS_WITH
   *
   * enables starting with start box mode.
   */
  static const ParameterString<bool> START_BOX_HANDLING_STARTS_WITH;
  /*!
   * \brief USE_ALWAYS_INIT
   *
   * using init in every image.
   */
  static const ParameterString<bool> USE_ALWAYS_INIT;
  /*!
   * \brief VEHICLE_POINT_EQUALS_EPS
   *
   * epsilon for vehicle points equals
   */
  static const ParameterString<double> VEHICLE_POINT_EQUALS_EPS;
  /*!
   * \brief NAV_POINTS_TRESHOLD
   *
   * This is the minimal number of points, a line must have for publishing as
   *nav_points.
   */
  static const ParameterString<int> NAV_POINTS_TRESHOLD;
  /*!
   * \brief CONSISTENCY_CHECK_SAMPLING_POINTS
   *
   * The points (x-coordinates) where the consistency-checks are performed of.
   *
   */
  static const ParameterString<std::vector<double> > CONSISTENCY_CHECK_SAMPLING_POINTS;
  /*!
   * \brief POINT_FILTERING_ENABLED
   *
   * enables points filtering.
   */
  static const ParameterString<bool> POINT_FILTERING_ENABLED;
  /*!
   * \brief RELATIVE_CLUSTER_THRESHOLD
   *
   * threshold added to step_x for clustering lines
   */
  static const ParameterString<double> RELATIVE_CLUSTER_THRESHOLD;
  /*!
   * \brief CLUSTER_THRESHOLD_LEFT_RIGHT
   */
  static const ParameterString<double> CLUSTER_THRESHOLD_LEFT_RIGHT;
  /*!
   * \brief CLUSTER_MIN_SIZE
   *
   * minimal cluster size for valid clusters
   */
  static const ParameterString<int> CLUSTER_MIN_SIZE;
  /*!
   * \brief OUTLINE_LOCALIZATION_THRESHOLD
   *
   * threshold for decision when localizing dashed right middle line
   */
  static const ParameterString<double> OUTLINE_LOCALIZATION_THRESHOLD;
  /*!
   * \brief DISCRETIZATION_STEP
   *
   * discretization step for equally discretizing outline points and right
   *middle line points
   */
  static const ParameterString<double> DISCRETIZATION_STEP;
  /*!
   * \brief MINIMAL_CLUSTER_LENGTH
   *
   * minimal euclidian dist from nearest point of cluster to the farest,
   *according to
   * cup regulations. In addition a security threshold of 5 cm is added to be
   *realy sure
   */
  static const ParameterString<double> MINIMAL_CLUSTER_LENGTH;
  /*!
   * \brief POLYNOMIAL_MIN_X
   *
   * Minimal x-axis value of the polynomial. Used for evaluation of polynomial.
   */
  static const ParameterString<double> POLYNOMIAL_MIN_X;
  /*!
   * \brief POLYNOMIAL_MAX_X_SHORT
   *
   * Maximal x-axis value of the polynomial. Used for evaluation of polynomial.
   */
  static const ParameterString<double> POLYNOMIAL_MAX_X;
  /*!
   * \brief POLYNOMIAL_STEP
   *
   * Step size on x-axis. Used for evaluation of the polynomial.
   */
  static const ParameterString<double> POLYNOMIAL_STEP;

  static const ParameterString<double> POLYNOMIAL_CAUCHY_LOSS;


  /*!
   * \brief registerLineTrackingParameter
   *
   * registers all parameters needed by LaneDetection to the
   *parameter_interface.
   *
   * \param parameter_interface the parameter_interface
   */
  static inline void registerLaneDetectionParameter(ParameterInterface* parameter_interface);


  bool extractLines(const cv::Mat& image, const bool second_run, LineVehiclePoints* out_line_data);

  /*!
   * \brief tryCorrectingSingleLine
   *
   * takes an outer line (left or right) and checks if there is a middle line on
   * the other side. For example: usually the middle line is on the right side
   *of
   * the left line, but it occurs, that the right line is recognized as the left
   * line. In this case, the middle line is on the left side of the left line.
   * This function tries to recognize and correct this mistake.
   *
   * \param image the image.
   * \param linespec the line type.
   * \param shift distance to shoft the outer line
   * \param out_line_data the line data.
   */
  void tryCorrectingSingleLine(const cv::Mat& image,
                               const LineSpec linespec,
                               const double shift,
                               LineVehiclePoints* out_line_data);

  /*!
   * \brief filterVehiclePoints
   *
   * Performs filtering on the feature points from the feature-extraction. The
   *filtering is currently based on clustering. For more details see
   *advancedPointsFiltering.
   *
   * \param in_points the input VehiclePoints
   * \param out_line_points the output VehiclePoints
   * \param linespec secifies the current line
   */
  void filterVehiclePoints(VehiclePoints in_points,
                           const LineSpec linespec,
                           VehiclePoints* out_line_points);

  /*!
   * \brief fitPolynom
   *
   *  Fits the polynoms. First pointsfiltering is performed. Then the polynom
   *gets fitted to the filtered points.
   *  After fitting polynomial-degree-adaptions is considered and perfromed if
   *appropriated.
   *
   * \param lineSpec specifies the current line
   * \param line_data the line points.
   */
  inline void fitPolynom(const LineSpec lineSpec, const VehiclePoints& line_data);

  inline common::polynomial::DynamicPolynomial fitToPoints(
      const VehiclePoints& line_data, const common::polynomial::PolynomialDegree degree) const;

  /*!
   * \brief adaptPolyDegree
   *
   * Checks is absolute error and quadratic komponent of the polynom are in
   *bound and perfroms degree-adaption if it is appropiated.
   *
   * \param lineSpec specifies the current line
   * \param line the line to chec
   * \param points the points corresponding to the line
   * \param error the old fitting error. (Contains after refitting the new
   *fitting error)
   */
  inline void adaptPolyDegree(const LineSpec lineSpec,
                              common::DynamicPolynomial* line,
                              const VehiclePoints& points,
                              double* error);

  /*!
   * \brief switchDegree
   *
   * is a helper function to change the degree of a polynom and refit it.
   *
   * \param sw desicedes if the degree should be decremented (-1) or resetted to
   * standard (0)
   * \param line_degree the degree of the line.
   * \param line the resulting  refitted (if chosen) polynom
   * \param points the points corresponding to the line (needed for refit)
   * \param error the new fitting error
   */
  void switchDegree(const int sw,
                    int* line_degree,
                    common::DynamicPolynomial* line,
                    const VehiclePoints& points,
                    double* error);

  /*!
   * \brief checkConsistency
   *
   * Performs some consistency checks on the points and corrects
   *consistency-problems if possible.
   *
   * \param out_line_data the the points to check classified by lines
   */
  inline void checkConsistency(LineVehiclePoints* out_line_data);

  void checkPairWise(const LineSpec& lefter, const LineSpec& righter, LineVehiclePoints* out_line_data);

  void moveToMiddle(const LineSpec line_spec, LineVehiclePoints* out_line_data);

  void eraseCompletly(const LineSpec line_sec, LineVehiclePoints* out_line_data);

  bool inPedestrianIsland(const LineVehiclePoints& out_line_data) const;

  double estimateLaneWidth(const LineSpec& outer, const LineVehiclePoints& out_line_data) const;

  /*!
   * \brief addVirtualLines
   *
   * If a line drops away, this function creates a new "virtual" (not really
   *existing) polynom. This is needed to find the missing line again.
   * The other still existing lines will be discretesised. This points will be
   *shifted along the normal of the polynom in this point. To this
   * points the new polynom will be fitted.
   *
   * \param target specifies the Line to add a virtual line for
   * \param source1 the first source line to obtain the new virtual line from
   * \param source2 the secound source line to obtain the new virtual line from
   * \param shift1 the offset from the first source line to the target line
   * \param shift2 the offset from the source source line to the target line
   * \param line_existed for every line wheather it existed before
   *reconstruction.
   */
  inline void addVirtualLine(const LineSpec target,
                             const LineSpec source1,
                             const LineSpec source2,
                             const double shift1,
                             const double shift2,
                             const std::array<bool, LINESPEC_N>& line_existed);

  /*!
   * \brief advancedPointsFiltering
   *
   * performs filtering on the feature points extracted from the image. At first
   *clustering is performed. The resulting cluster are checked
   * against size and nearness before added to the output array of points.
   *
   * \param in the array of input points
   * \param out the array of output points
   * \param dist_thresh the widest distance between two neighbour points in a
   *cluster
   * \param count_thresh_ratio the ratio with part of the whole input point to
   *add a cluster
   *       to the output points array
   * \param count_thresh the number of points a cluster mus at least contain to
   *be taken to
   *        the output points if a points in the cluster is nearer than
   *near_field_dist
   * \param near_field_dist the distance a cluster is considered as near enough
   *to be not mistaken
   */
  void inline advancedPointsFiltering(const VehiclePoints& in,
                                      VehiclePoints* out,
                                      const double dist_thresh,
                                      const double count_thresh_ratio,
                                      const size_t count_thresh,
                                      const double near_field_dist);

  /*!
   * \brief mightBeMiddleLine EXPERIMENTAL
   *
   * decides, if a given point array represents a middle line or not.
   *
   * \param points the points to deside about
   * \return if the given point array represents a middle line or not.
   */
  bool mightBeMiddleLine(const VehiclePoints& points, const bool strict) const;

  /*!
   * \brief publishFeaturePoints
   *  Publishes FeaturePoints to topic
   *"/perception/feature_extraction/feature_points".
   *
   * \param line_points the points to publish
   */
  void publishFeaturePoints(LineVehiclePoints& line_points) const;

  /*!
   * \brief handleStartBox
   *
   * performes the operations which are needed for startbox mode.
   *
   * \param out_line_data the lines to perform startbox mode on.
   */
  void handleStartBox(LineVehiclePoints* out_line_data);

  /*!
   * \brief checkLineLinear
   *
   *
   * checks if the given line exists and is linear. If the line is not linear,
   *it get erased.
   * This function is only used in startbox mode!
   *
   * \param lineSpec the line to check
   * \param out_line_data all lines
   * \return whether the line exists and is linear
   */
  inline bool checkLineLinear(const LineSpec lineSpec, LineVehiclePoints* out_line_data);

  /*!
   * \brief isLinear
   *
   * returns whether the given line is linear. In this case a line is considered
   *as linear if
   *   - the absolute value of the square component is less than
   *START_BOX_HANDLING_MAX_A and
   *   - the absolute value of the linear component is less the
   *START_BOX_HANDLING_MAX_B
   *
   * This function is only used in startbox mod!
   *
   * \param line the line to check linearity of.
   * \return whether the given line is linear
   */
  bool isLinear(const common::DynamicPolynomial& line) const;

  /*!
   * \brief deleteLineWithFewPoints
   *
   * deletes line #line from #line_points, if the number of points in the line
   *are less than #FITTING_IMAGE_POINTS_TRESHOLD.
   *
   * \param line the line to check
   * \param line_points to remove from
   */
  void deleteLineWithFewPoints(const LineSpec& line, LineVehiclePoints* line_points) const;

  /*!
   * \brief tryNoPassingLine creates the second line, if present, and evaluates
   * whether it indicates no passing zone or not
   * \param image the image.
   * \param out_line_middle the middle line.
   * \param no_passing_line the no passing line..
   */
  bool tryNoPassingLine(const cv::Mat& image,
                        const VehiclePoints* out_line_middle,
                        VehiclePoints* no_passing_line);

  /*!
   * \brief filterTransformMiddlePoints transforms the input-ImagePoints to
   * VehiclePoints and performs cluster based filtering
   * on them, in order to delete misdetected points
   * \param double_line_points ImagePoints of left or right part of the
   * double-middle-line
   * \return transformed,filtered former ImagePoints
   */
  VehiclePoints filterTransformMiddlePoints(const VehiclePoints& double_line_points);

  /*!
   * \brief checkDoubleMiddleLines central interface to detection and
   * localization of double middle lines
   * \param image preprocessed image
   * \param out_line_data line data generated by the lane detection before
   * checking for double middle lines: of there's a dashed line on the right
   * part of the double middle line, this dashed line is integrated in
   * out_line_data after calling that function. If there's
   * no double middle line at all, out_line_data isn't modified
   */
  void checkDoubleMiddleLines(const cv::Mat& image, LineVehiclePoints* out_line_data);

  /*!
   * \brief deleteTooSmallClusters deletes clusters containing to few points
   * \param clusters the clusters.
   */
  void deleteTooSmallClusters(std::vector<VehiclePoints>* clusters);

  /*!
   * \brief localizeOutlinePoints is called when right part of double middle
   * line is dashed. This
   * function localizes the middle line correctly
   * \param right_line_middle right part of double middle line
   * \param out_line_middle middle line to be localized correctly
   */
  void localizeOutlinePoints(VehiclePoints* right_line_middle, VehiclePoints* out_line_middle);
  /*!
   * \brief deleteDoublePoints functions deletes points that re too close to
   * each other
   * \param line_points points to be filtered
   */
  void deleteDoublePoints(VehiclePoints* line_points);
  /*!
   * \brief isLefterDashed checks if left middle line is dashed by clustering
   * \param points left part of double midline
   */
  bool isDashed(const VehiclePoints& points);
  /*!
   * \brief largeClusters checks if at least one cluster is large enough to be a
   * solid line
   * \param clusters clusters to be checked
   * \return true, if at least one cluster is large enough to be a solid line
   */
  bool largeClusters(std::vector<VehiclePoints>& clusters);

  virtual void showProjectionPoints(const LineSpec&, const VehiclePoints&) {
    return;
  }

  /*!
   * \brief init
   *
   * if set, initialization is performed
   */
  bool init = true;

  /*!
   * \brief use_start_box_mode_
   *
   * whether startbox mode is activated or not.
   */
  bool use_start_box_mode_;

  /*!
   * \brief line_degree de current degrees of the polynoms
   */
  LineDegree line_degree = {};

  common::DiscretizationParams discretization_params;

  /*!
  * \brief parameters_ptr_ is needed for parameter access
  */
  const ParameterInterface* parameters_ptr_;

  std::array<VehiclePointFilter::Ptr, LINESPEC_N> vehicle_point_filter;
};

#endif  // LANE_DETECTION_H
