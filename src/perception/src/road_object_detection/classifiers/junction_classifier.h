#ifndef JUNCTION_CLASSIFIER_H
#define JUNCTION_CLASSIFIER_H

#include "../classifier.h"
#include "common/parameter_interface.h"
#include "perception_types.h"
#include "scan_line.h"

namespace road_object_detection {

/**
 * classifier using PCA for Line classification
 */
class JunctionClassifier : public virtual Classifier {
 public:
  JunctionClassifier(ParameterInterface* parameter_interface,
                     const common::CameraTransformation* const camera_transform);

  virtual ~JunctionClassifier() override = default;

  virtual RoadObjects classify(const Features& features) override;
  virtual size_t getClassifierId() const final override;

 protected:
  bool DEBUG_MODE = false;
  static const std::string NAMESPACE;
  static constexpr std::array<Junction::JunctionType, 4> allJunctionTypes() {
    return {{Junction::stopline_left,
             Junction::stopline_right,
             Junction::givewayline_left,
             Junction::givewayline_right}};
  }
  struct PCAData {
    // Eigen Vals get initialized with large numbers to prevent dividing by zero
    // if pca got no points
    Eigen::Vector2d eigen_vals{std::numeric_limits<double>::infinity(),
                               std::numeric_limits<double>::infinity()};
    ImagePointExact mean;
    double angle = 0.0;
    Eigen::Matrix2d eigen_vecs;
  };

  using Scoring = std::array<double, 4>;

  struct Trapeze {
    double min;
    double max;
    double fall_dist;
  };

  struct Params {
    int min_point_dist;
    int min_point_nr;
    int vertical_offset;
    int offset_eigenline;
    int lenght_eigenline;
    int lenght_eigenline_offset;
    int lenght_step_ref_func;
    int threshold_step_eigenline;
    int filter_size_median;
    int adaptive_thresh_blocksize;
    int adaptive_thresh_const;

    double is_stop_line_left;
    double is_give_way_line_left;
    double is_stop_line_right;
    double is_give_way_line_right;

    float min_x_dist;
    Trapeze angle_diff;
    Trapeze mirror_side_no_median;

    // stopline
    Trapeze stop_line_left_mean1;
    Trapeze stop_line_left_mean0;

    Trapeze stop_line_left_middlelane_pca;
    Trapeze stop_line_right_middlelane_pca;

    Trapeze stop_line_outer_line_steps;
    Trapeze stop_line_middle_line_steps;

    Trapeze stop_line_pos_neg_ratio;

    Trapeze stop_line_right_mean1;
    Trapeze stop_line_right_mean0;

    Trapeze stop_middle_line_no_median;

    // giveaway
    Trapeze give_way_line_left_mean1;
    Trapeze give_way_line_left_mean0;

    Trapeze give_way_line_left_middlelane_pca;
    Trapeze give_way_line_right_middlelane_pca;

    Trapeze give_way_line_outer_line_steps;
    Trapeze give_way_line_middle_line_steps;

    Trapeze give_way_line_pos_neg_ratio;

    Trapeze give_way_line_right_mean1;
    Trapeze give_way_line_right_mean0;

    Trapeze give_way_middle_line_no_median;
  };

  struct CandidateData {
    cv::Rect window;
    Scoring score = {};
    PCAData pca_data;
    ImagePoints pos_steps_window;
    ImagePoints neg_steps_window;
    ImagePoints steps_window;
    double angle_diff = 0;
    double dist_middlelane_pca_mean = 0;
    double pos_neg_ratio = 0.0;
    int nr_horizontal_eigen_line_plus_points = 0;
    int nr_horizontal_eigen_line_points = 0;
    int nr_horizontal_eigen_line_minus_points = 0;
    int nr_horizontal_eigen_line_points_no_median = 0;
    int nr_vertical_eigen_line_points_no_median = 0;
    ScanLine horizontal_eigen_line_plus;
    ScanLine horizontal_eigen_line;
    ScanLine horizontal_eigen_line_minus;
    ScanLine horizontal_eigen_line_no_median;
    ScanLine vertical_eigen_line_plus;
    ScanLine vertical_eigen_line_minus;
  };

  enum class Alignment { HORIZONTAL, VERTICAL };

  PCAData getOrientation(const ImagePoints& pts) const;
  double getAngleDiff(const Features& features,
                      const common::DynamicPolynomial& middle_lane,
                      const PCAData& pca_data) const;
  double getDistMiddleLanePcaMean(const Features& features,
                                  const common::DynamicPolynomial& middle_lane,
                                  const PCAData& pca_result) const;
  Scoring getScore(const CandidateData& candidate_data, const Params& params) const;
  double trapeze(Trapeze params, double pos) const;
  ImagePoints getStepPointsNoDoubling(int step_dist,
                                      int step_function_lenght,
                                      int step_function_offset,
                                      int min_point_dist,
                                      const cv::Mat& img,
                                      int threshold,
                                      bool direction) const;

  ImagePoints getLineStepPointsNoDoubling(const ScanLine& line,
                                          int step_function_lenght,
                                          int min_point_dist,
                                          const cv::Mat& img,
                                          int threshold) const;

  cv::Point CvPointOrthogonalOffset(const cv::Point& point,
                                    int offset,
                                    const JunctionClassifier::Alignment) const;

  ImagePoints GetStepsOnMeanEigenLine(const cv::Mat& birdsview,
                                      const ScanLine& line,
                                      const int& step_function_lenght,
                                      const int& min_point_dist,
                                      const int& threshold) const;

  ScanLine GetEigenLine(const PCAData& pca_result,
                        const int& offset,
                        const double& eigen_lenght,
                        const JunctionClassifier::Alignment alignment) const;

  RoadObjects GetRoadObjects(const Features& features,
                             const VehiclePose& line_pose,
                             const JunctionClassifier::Scoring& score) const;

  CandidateData classifyCandidate(const Features& features,
                                  const cv::Rect& feature_window,
                                  const ImagePoints& pos_steps_box,
                                  const ImagePoints& neg_steps_box,
                                  const Params& params,
                                  const int& loop_counter,
                                  const std::string& unique_id) const;

  VehiclePose getLinePose(const Features& features, const PCAData& pca_result) const;

 protected:
  using FourCandidates = std::array<CandidateData, 4>;

  virtual FourCandidates findFourCandidates(const Features& features,
                                            const ImagePoints& pos_steps_box,
                                            const ImagePoints& neg_steps_box) const;

  const common::CameraTransformation* const camera_transformation_;

 private:
  struct TrapezeParameterStrings {

    const ParameterString<double> MIN;
    const ParameterString<double> MAX;
    const ParameterString<double> FALL_DIST;

    static TrapezeParameterStrings Create(const std::string& name) {
      return {ParameterString<double>{name + "_min"},
              ParameterString<double>{name + "_max"},
              ParameterString<double>{name + "_fall_dist"}};
    }
  };

  Params readParameters() const;

  cv::Rect computeBordersRect(const Features& features) const;

  VehiclePoints computeBaseHullPolygon(const Features& features,
                                       const VehiclePose& line_pose,
                                       const Junction::JunctionType type) const;

  RoadObjects createRoadObjects(const FourCandidates& winning_candidates,
                                const Features& features) const;

  static void registrate(ParameterInterface* parameter_interface,
                         const TrapezeParameterStrings& p);

  static Trapeze getParams(const ParameterInterface* const parameter_interface,
                           const TrapezeParameterStrings& p);

  static const ParameterString<int> STEP_THRESHOLD;
  static const ParameterString<int> STEP_DIST;
  static const ParameterString<int> STEP_FUNCTION_LENGHT;
  static const ParameterString<int> STEP_FUNCTION_OFFSET;
  static const ParameterString<int> MIN_POINT_DIST;
  static const ParameterString<int> RECT_ADD_SPACE_X;
  static const ParameterString<int> RECT_ADD_SPACE_Y;
  static const ParameterString<int> RECT_MIN_X_SIZE;
  static const ParameterString<int> RECT_MIN_Y_SIZE;
  static const ParameterString<double> DEFAULT_IS_GIVE_WAY_LINE_LEFT;
  static const ParameterString<double> DEFAULT_IS_GIVE_WAY_LINE_RIGHT;
  static const ParameterString<double> DEFAULT_IS_STOP_LINE_LEFT;
  static const ParameterString<double> DEFAULT_IS_STOP_LINE_RIGHT;

  static const ParameterString<int> ADAPTIVE_THRESH_BLOCKSIZE;
  static const ParameterString<int> ADAPTIVE_THRESH_CONST;

  static const TrapezeParameterStrings ANGLE_DIFF;

  static const TrapezeParameterStrings GIVE_WAY_LINE_LEFT_MEAN1;
  static const TrapezeParameterStrings GIVE_WAY_LINE_LEFT_MEAN0;

  static const TrapezeParameterStrings GIVE_WAY_LINE_LEFT_DIST_MIDDLELANE_PCA;

  static const TrapezeParameterStrings GIVE_WAY_LINE_OUTER_LINE_STEPS;

  static const TrapezeParameterStrings GIVE_WAY_LINE_MIDDLE_LINE_STEPS;

  static const TrapezeParameterStrings GIVE_WAY_LINE_POS_NEG_RATIO;

  static const TrapezeParameterStrings GIVE_WAY_LINE_RIGHT_MEAN1;
  static const TrapezeParameterStrings GIVE_WAY_LINE_RIGHT_MEAN0;

  static const TrapezeParameterStrings GIVE_WAY_LINE_RIGHT_DIST_MIDDLELANE_PCA;

  static const TrapezeParameterStrings STOP_LINE_LEFT_MEAN1;
  static const TrapezeParameterStrings STOP_LINE_LEFT_MEAN0;

  static const TrapezeParameterStrings STOP_LINE_LEFT_DIST_MIDDLELANE_PCA;

  static const TrapezeParameterStrings STOP_LINE_OUTER_LINE_STEPS;

  static const TrapezeParameterStrings STOP_LINE_MIDDLE_LINE_STEPS;

  static const TrapezeParameterStrings STOP_LINE_POS_NEG_RATIO;

  static const TrapezeParameterStrings STOP_LINE_RIGHT_MEAN1;
  static const TrapezeParameterStrings STOP_LINE_RIGHT_MEAN0;

  static const TrapezeParameterStrings STOP_LINE_RIGHT_DIST_MIDDLELANE_PCA;

  static const ParameterString<int> OFFSET_EIGENLINE;
  static const ParameterString<int> LENGHT_EIGENLINE;
  static const ParameterString<int> LENGHT_EIGENLINE_OFFSET;
  static const ParameterString<int> LENGHT_STEP_REF_FUNC;
  static const ParameterString<int> THRESHOLD_STEP_EIGENLINE;

  static const TrapezeParameterStrings MIRROR_SIDE_NO_MEDIAN;

  static const TrapezeParameterStrings GIVE_WAY_MIDDLE_LINE_NO_MEDIAN;

  static const TrapezeParameterStrings STOP_MIDDLE_LINE_NO_MEDIAN;

  static const ParameterString<int> FEATURE_WINDOW_HEIGHT;
  static const ParameterString<int> MIN_POINT_NR;

  static const ParameterString<int> FILTER_SIZE_MEDIAN;
  static const ParameterString<int> VERTICAL_OFFSET;
  static const ParameterString<int> MIN_X_DIST;

  const ParameterInterface* const parameter_interface_;
};

}  // namespace road_object_detection

#endif  // JUNCTION_CLASSIFIER_H
