#ifndef CROSSWALKCLASSIFIER_H
#define CROSSWALKCLASSIFIER_H

#include "../classifier.h"
#include "common/parameter_interface.h"
THIRD_PARTY_HEADERS_BEGIN
#include <boost/optional.hpp>
THIRD_PARTY_HEADERS_END

#include "common/discretisize.h"

namespace road_object_detection {

using Direction = Eigen::Vector3d;
using CVPoints = std::vector<cv::Point>;

/**
 * @brief Classifier tracks
 */
class CrosswalkClassifier : public virtual Classifier {
 public:
  CrosswalkClassifier(const common::CameraTransformation *cam_transform,
                      ParameterInterface *parameter_interface);
  virtual ~CrosswalkClassifier() override = default;
  virtual RoadObjects classify(const Features &features) override;
  virtual size_t getClassifierId() const final override;

  static const std::string NAMESPACE;

 protected:
  // Functions
  virtual CVPoints linePoints(const Features &features, const int row_idx) const;

  virtual std::pair<common::DynamicPolynomial, common::DynamicPolynomial> extractBoundingPolynomials(
      const Features &features) const;

  VehiclePoint computeLocalizationPoint(const common::DynamicPolynomial &middle_lane_polynomial,
                                        const VehiclePoints &left_points,
                                        const double angle,
                                        const VehiclePoint &middle_point) const;
  VehiclePoint computeLanePoint(const VehiclePoints &lane_points,
                                const double angle,
                                const VehiclePoint &ref_point,
                                const VehiclePoint &offset) const;

  VehiclePoint computeBound(const Features &features,
                            const VehiclePoint &ref_point,
                            const double guessed_width) const;
  Direction getCrosswalkDirection(const Features &features,
                                  const std::vector<CVPoints> &crosswalk_lines) const;

  VehiclePoints convexHull(const VehiclePose &cw_pose) const;
  bool isTooSmall(const Features &features) const;

  bool notWideEnough(const Features &features) const;

  bool assumptPedestrianIsland(const Features &features) const;

  virtual std::vector<CVPoints> onlyPointsOnLane(const std::vector<CVPoints> &crosswalk_lines,
                                                 const Features &features) const;
  /*!
   * \brief getDistVariance calculates the mean variance of the distances of the
   * points the vector is conatining
   * \param points the points.
   * \return the variance of U.
   */
  //  double getUVariance(const CVPoints &points) const;

  std::vector<double> estimateLaneWidth(const common::DynamicPolynomial &lane_polynom,
                                        const common::DiscretizationParams &params,
                                        const VehiclePoints &outer_lane_points) const;

  void createCrosswalk(RoadObjects &crosswalk,
                       const Features &features,
                       const std::vector<CVPoints> &crosswalk_lines,
                       const double score) const;

  // Attributes/Parameters
  const common::CameraTransformation *const camera_transform;

  const ParameterInterface *const parameter_interface_;

  static const ParameterString<int> WHITE_PIXEL;
  static const ParameterString<int> BLACK_PIXEL;
  static const ParameterString<int> LINE_OFFSET;
  static const ParameterString<int> BOTTOM_LINE_ROW;
  static const ParameterString<int> NR_LINES;
  static const ParameterString<int> CANNY_THRESH_LOW;
  static const ParameterString<int> CANNY_THRESH_HIGH;
  static const ParameterString<int> CANNY_MASK_SIZE;
  static const ParameterString<int> MIN_SIZE_POINT_VECTORS;
  static const ParameterString<double> GUESSED_LANE_WIDTH;
  static const ParameterString<double> STEP_LOC_POINT;
  static const ParameterString<int> OFFSET_RIGHT_LEFT_BOUND;
  static const ParameterString<double> MAX_VARIANCE;
  static const ParameterString<double> MIN_SCORE;
  static const ParameterString<double> MIN_PROJECTION_ON_LANE;
  static const ParameterString<double> CROSSWALK_LENGTH;
  static const ParameterString<double> DISC_X_STEP;
  static const ParameterString<double> MAX_LANE_WIDTH;
  static const ParameterString<double> MIN_LANE_WIDTH;
  static const ParameterString<double> MIN_CLUSTER_SPREAD_NORMAL;

  //  CVPoints points_l1 = {};
  //  CVPoints points_l2 = {};
  //  ImagePoint right_bound_image, left_bound_image;
  //  cv::Mat bottomLine;
  //  cv::Mat topLine;
  //  std::vector<double> dots = {};
  VehiclePoint closest_point, farest_point;
};

}  // namespace road_object_detection

#endif  // CROSSWALKCLASSIFIER_H
