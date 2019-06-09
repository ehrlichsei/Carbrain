#ifndef START_LINE_CLASSIFIER_H
#define START_LINE_CLASSIFIER_H

#include "../classifier.h"
#include "perception_types.h"

namespace road_object_detection {

using CVPoints = std::vector<cv::Point>;
using Direction3d = Eigen::Vector3d;

class StartLineClassifier : public virtual Classifier {
 public:
  StartLineClassifier(const common::CameraTransformation *camera_transformation,
                      ParameterInterface *parameter_interface);

  virtual ~StartLineClassifier() override = default;

  virtual RoadObjects classify(const Features &features) override;

  virtual size_t getClassifierId() const final override;

  static const std::string NAMESPACE;

 protected:
  const common::CameraTransformation *camera_transform_;
  ParameterInterface *parameters_ptr_;
  double meanStepSize(const Eigen::Vector3d &pc, const Features &features) const;
  static const ParameterString<double> LINE_SHIFT;
  static const ParameterString<int> REFERENCE_FUNCTION_LENGTH;
  static const ParameterString<double> STEP_DETECTION_THLD;

 protected:
  virtual bool isLine(const Features &features, Eigen::Vector3d &direction) const;

  // virtual CVPoints stepPoints(const Features &features,
  //                          const VehiclePoint &start_dir,
  //                        const VehiclePoint &end_dir) const;

  // virtual cv::Rect bvROI(const Features &features) const;

  bool hasFeaturePointsOnBothLanes(const Features &features) const;

  VehiclePose setHullPointsAndPose(VehiclePoints &hull_points,
                                   const Features &features,
                                   const Eigen::Vector3d &direction) const;

  //  bool isFeaturePartOfImage(const ImagePoints& feature_points);

  std::unique_ptr<StartLine> toStartLine(const double score,
                                         const Features &features,
                                         const Eigen::Vector3d &direction = Eigen::Vector3d::Identity());

  static const ParameterString<double> MIN_SCORE_RATIO;
  static const ParameterString<double> MAX_ANGLE_DELTA;
  static const ParameterString<int> HARRIS_CORNER_THRESHOLD;
  static const ParameterString<double> MEAN_STEP_SIZE;
  static const ParameterString<int> HARRIS_BLOCK_SIZE;
  static const ParameterString<int> SOBEL_WINDOW;
  static const ParameterString<int> HARRIS_K;
  static const ParameterString<double> MINIMAL_HARRIS_MAX;
  static const ParameterString<int> MIN_NR_POINTS;
  static const ParameterString<double> FIXED_SCORE_START_LINE;
  static const ParameterString<bool> STEP_DET_ABS_FLAG;
  static const ParameterString<double> LANE_WIDTH;
  static const ParameterString<double> UNIDENTIFIED_SCORE_THRESHOLD;
  static const ParameterString<double> HALF_STARTLINE_LENGTH;
};


}  // namespace road_object_detection

#endif  // START_LINE_CLASSIFIER_H
