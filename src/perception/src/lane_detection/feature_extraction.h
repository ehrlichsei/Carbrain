#ifndef KITCAR_PERCEPTION_FEATURE_EXTRACTION_H
#define KITCAR_PERCEPTION_FEATURE_EXTRACTION_H
#include <common/macros.h>

THIRD_PARTY_HEADERS_BEGIN
#include <opencv2/core/core.hpp>
THIRD_PARTY_HEADERS_END

#include "common/parameter_interface.h"
#include "roi_birds_view_transformation.h"
#include "scan_line.h"
#include "vehicle_scan_line.h"
#include "line_vehicle_points.h"
#include "lane_model.h"
#include "../utils/ego_vehicle.h"

/**
 * Feature extraction of recorded images.
 *
 * This class performs the feature extraction in the recorded, colored
 * images and returns (potential) lane markings to the line tracking component.
 */
class FeatureExtraction {
 public:
  /**
   * @brief FeatureExtraction
   * The constructor - deprecated!
   *
   * \param parameters_ptr the parameters
   * \param view_transform the birds view transformation
   */
  FeatureExtraction(ParameterInterface* parameters_ptr,
                    ROIBirdsViewTransformation* view_transform,
                    EgoVehicle* ego_vehicle);
  FeatureExtraction(FeatureExtraction&&) = default;
  virtual ~FeatureExtraction() = default;

  /**
   * @brief extractLineMarkings
   *
   * extracts line markings assuming that the camera looks straight to a
   *straight part of the track.
   * This is jused for initialisation.
   *
   * \param image the source image
   * \return line_points the output line markings
   */
  virtual LineVehiclePoints extractLineMarkings(const cv::Mat& image) const;

  /**
   * @brief extractLineMarkings
   *
   * extracting line markings using the previous detected lines.
   *
   * \param image the source image
   * \param line_model the previous detected lines
   * \return line_points the output line markings
   */
  virtual LineVehiclePoints extractLineMarkings(const cv::Mat& image,
                                                const LaneModel& line_model) const;

  /**
   * @brief extractLinePointsByLine
   *
   * extracts the line points per line using previous detected lines.
   *
   * \param image the region of interest
   * \param line the previous detected lines
   * \return the output points
   */

  VehiclePoints extractLinePointsByLine(
      const cv::Mat& image,
      const common::DynamicPolynomial& line,
      const boost::optional<common::DynamicPolynomial>& ref_line = boost::none,
      const boost::optional<LineSpec> l_spec = boost::none) const;
  VehiclePoints extractLinePointsByLine(
      const cv::Mat& image,
      const boost::optional<common::DynamicPolynomial>& line,
      const boost::optional<common::DynamicPolynomial>& ref_line = boost::none,
      const boost::optional<LineSpec> l_spec = boost::none) const;


  void updateROI(const cv::Rect& roi) {
    this->roi = roi;
    updateLowerImageEdgeX();
  }

  /*!
   * \brief isDoubleLine checks if there's a double middle line and returns
   * its ine points.
   * \param lines the scanline.
   * \param left_line points of left line if existing
   * \param right_line points of right line if existing
   * \return true, if there are two lines
   */
  virtual bool isDoubleLine(const cv::Mat& image,
                            const ScanLines& lines,
                            VehiclePoints* left_line,
                            VehiclePoints* right_line);

  /*!
   * \brief doubleDetected returns true, if two lines are detected for current
   * iterator
   * \param it The LineIterator corresponding to current ScanLine
   * \param left_line left line to be filled
   * \param right_line right line to be filled
   * \return true, if two lines are detected
   */
  bool doubleDetected(cv::LineIterator& it, VehiclePoints* left_line, VehiclePoints* right_line);

  /*!
   * \brief createScanLines
   *
   * creates scan lines based on the given polynom.
   *
   * \param line the line to create the scan lines from
   * \return the created scan lines
   */
  virtual ScanLines createScanLines(const common::DynamicPolynomial& line) const;


 protected:
  struct CenterOfConnectedPointsParam {
    double max_width_feature_pair;
    double step;
    double max_abs_dot;
    double eps;
  };
  /**
   * @brief extractLinePoints
   *
   * extracts the line points per line using segmented ground points.
   *
   * \param image the source image (just regeion of interest)
   * \param ground_points the segmented ground points
   * \return image_points the output image points
   */
  VehiclePoints extractLinePoints(
      const cv::Mat& image,
      const VehiclePoints& ground_points,
      const boost::optional<common::DynamicPolynomial>& ref_line = boost::none,
      const boost::optional<LineSpec> l_spec = boost::none) const;

  /*!
   * \brief createScanLines
   *
   * created scan line based on the given ground points
   *
   * \param ground_points the ground points considered to a line at
   *initialisation
   * \return the created scan lines.
   */
  virtual ScanLines createScanLines(const VehiclePoints& ground_points) const;

  /**
   * @brief getCenterOfConnectedLinePoints
   *
   *  Findes the center point between connected points in x axis direction
   *
   * \param image the source image
   * \param scan_line the scan line.
   * \param center_points output center points
   *
   * \todo remove if-block to include only fully seen lane markings
   */
  void getCenterOfConnectedLinePoints(
      const cv::Mat& image,
      const ScanLine& scan_line,
      ImagePoints* center_points,
      const CenterOfConnectedPointsParam& params,
      const boost::optional<common::DynamicPolynomial>& ref_line = boost::none,
      const boost::optional<LineSpec> l_spec = boost::none) const;

  /**
   * @brief getSegmentedGroundPoints
   *
   * creates a segmentation of the points. Points are in a segment if the are
   *inbetween specific borders
   *
   * \param image the source image (region of interest)
   * \return ground_points the output segmented ground points
   */
  LineVehiclePoints getSegmentedGroundPoints(const cv::Mat& image) const;

  /*!
   * \brief extractImagePointsByScanLines
   *
   * extract image points usign the given scan lines.
   *
   * \param image the image to search in
   * \param scan_lines the scan lines to use
   * \return the extracted image points
   */
  virtual ImagePoints extractImagePointsByScanLines(
      const cv::Mat& image,
      const ScanLines& scan_lines,
      const boost::optional<common::DynamicPolynomial>& ref_line = boost::none,
      const boost::optional<LineSpec> l_spec = boost::none) const;

  cv::Rect roi;

  /**
   * @brief view_transform the bird view transformation
   */
  ROIBirdsViewTransformation* view_transform;

  const EgoVehicle* const ego_vehicle_;


 private:
  /*!
   * \brief SEGMENT_BOUNDARIES_OUTERLEFT
   *
   * Used for init only. Boundary of outer_left segment: (inf,outer_left].
   * Points in this segment will be ignored.
   */
  static const ParameterString<double> SEGMENT_BOUNDARIES_OUTER_LEFT;
  /*!
   * \brief SEGMENT_BOUNDARIES_LEFT
   *
   * Used for init only. Boundary of left segment: (outer_left,left].
   */
  static const ParameterString<double> SEGMENT_BOUNDARIES_LEFT;
  /*!
   * \brief SEGMENT_BOUNDARIES_MIDDLE
   *
   * Used for init only. Boundary of middle segment: (left,middle].
   */
  static const ParameterString<double> SEGMENT_BOUNDARIES_MIDDLE;
  /*!
   * \brief SEGMENT_BOUNDARIES_RIGHT
   *
   * Used for init only. Boundary of right segment: (middle,right].
   */
  static const ParameterString<double> SEGMENT_BOUNDARIES_RIGHT;

  static const ParameterString<double> SEGMENT_BOUNDARIES_NEAR;

  static const ParameterString<double> SEGMENT_BOUNDARIES_FAR;
  /*!
   * \brief GENERAL_SIGMA_FACTOR
   *
   * Used for init only. Boundary of right segment: (middle,right].
   */
  static const ParameterString<double> GENERAL_SIGMA_FACTOR;
  /*!
   * \brief GENERAL_SCAN_LINE_STEP_SIZE
   *
   * The step used for scan line generation during initialization.
   */
  static const ParameterString<double> GENERAL_SCAN_LINE_STEP_SIZE;
  /*!
   * \brief GENERAL_SCAN_LINE_HALF_WIDTH
   *
   * Half-width of a scanline.
   */
  static const ParameterString<double> GENERAL_SCAN_LINE_HALF_WIDTH;
  /*!
   * \brief GENERAL_HISTOGRAM_BIN_SIZE
   *
   * Size of one histogram bin.
   */
  static const ParameterString<double> GENERAL_HISTOGRAM_BIN_SIZE;

  static const ParameterString<double> GENERAL_MAX_WIDTH_FEATURE_PAIR;

  static const ParameterString<double> GENERAL_MAX_ABS_DOT_HYP_NORMAL_MODEL;

  static const ParameterString<double> GENERAL_EPS;

  /*!
   * \brief POLYNOMIAL_MIN_X
   *
   * Minimal x-axis value of the polynomial. Used for evaluation of
   *polynomial.
   */
  static const ParameterString<double> POLYNOMIAL_MIN_X;
  /*!
   * \brief POLYNOMIAL_MAX_X_SHORT
   *
   * Maximal x-axis value of the polynomial. Used for evaluation of
   *polynomial.
   */
  static const ParameterString<double> POLYNOMIAL_MAX_X;
  /*!
   * \brief POLYNOMIAL_STEP
   *
   * Step size on x-axis. Used for evaluation of the polynomial.
   */
  static const ParameterString<double> POLYNOMIAL_STEP;

  /*!
   * \brief registerFeatureExtractionParameter
   *
   * registeres all parameters needed by feature extraction to the give
   *parameter interface.
   *
   * \param parameters_ptr the parameter interface
   */
  static inline void registerFeatureExtractionParameter(ParameterInterface* parameters_ptr);

  void clampAndPush(ScanLine scan_line, ScanLines* scan_lines) const;

  void updateLowerImageEdgeX();

  bool isAboveLowerImageEdge(const VehicleScanLine& l) const;

  bool isOccludedByMask(const VehicleScanLine& l) const;

  /**
   * @brief params parameters needed for feature extraction
   */
  const ParameterInterface* parameters_ptr_;

  double lower_image_edge_x = 0.0;
};

#endif /* KITCAR_PERCEPTION_FEATURE_EXTRACTION_H */
