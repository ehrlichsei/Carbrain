#ifndef NO_PASSING_ZONE_CLASSIFIER_H
#define NO_PASSING_ZONE_CLASSIFIER_H

#include "../classifier.h"
#include "vehicle_scan_line.h"


THIRD_PARTY_HEADERS_BEGIN

THIRD_PARTY_HEADERS_END

namespace road_object_detection {

class NoPassingZoneClassifier : public virtual Classifier {
 public:
  NoPassingZoneClassifier(const common::CameraTransformation* const camera_transformation,
                          ParameterInterface* const parameter_interface);

  NoPassingZoneClassifier() = delete;

  virtual ~NoPassingZoneClassifier() override = default;

  virtual RoadObjects classify(const Features& features) override;

  virtual size_t getClassifierId() const final override;

  static const std::string NAMESPACE;

 protected:
  const common::CameraTransformation* const camera_transformation_;



  const ParameterInterface* const parameters_ptr_;

  static const ParameterString<double> SCAN_RIGHT;
  static const ParameterString<double> SCAN_LEFT;
  static const ParameterString<int> STEP_THRESHOLD;
  static const ParameterString<int> STEP_LENGTH;
  static const ParameterString<double> MAX_LINE_LINE_DEV;
  static const ParameterString<double> MAX_LINE_SPACE_DEV;
  static const ParameterString<double> LINE_WIDTH;
  static const ParameterString<double> MAX_LINE_WIDTH_DEV;
  static const ParameterString<double> MIN_LENGTH_NP_ZONE;
  static const ParameterString<int> TWO_SOLID_LINES_QUAD_DOUBLE_RELATION;


  struct Params {
    double scan_right;
    double scan_left;
    int step_threshold;
    std::size_t step_length;
    double max_line_line_dev;
    double max_line_space_dev;
    double line_width;
    double max_line_width_dev;
    double min_length_np_zone;
    int two_solid_lines_quad_double_relation;
  };



  Params readParameters() const;

  std::vector<VehiclePoints> getSteps(const Features& features, const Params& params) const;

  std::vector<VehiclePoints> selectMultiSteps(const std::vector<VehiclePoints>& step_clstrs_gnd,
                                              const unsigned int steps) const;

  std::vector<VehiclePoints> filterLineWidth(const std::vector<VehiclePoints>& step_clstrs_gnd,
                                             const double line_width,
                                             const double max_deviation) const;

  float getAvgDistToLine(const VehicleScanLine& _line,
                         const std::vector<VehiclePoints>& step_clstrs,
                         const int stepnumber) const;

  std::vector<VehiclePoints> filterRelativeDeviation(const std::vector<VehiclePoints>& step_clstrs_gnd,
                                                     const double max_line2line_deviation,
                                                     const double max_line2space_deviation) const;


  VehiclePoints selectNthSteps(const std::vector<VehiclePoints>& step_clstrs,
                               const std::size_t stepnumber) const;
  bool solidLineIsRight(const std::vector<VehiclePoints>& double_step_clstrs,
                        const std::vector<VehiclePoints>& quad_step_clstrs,
                        const Features& features,
                        const Params& params) const;

  void np_debug(const std::string str) const;
};


/***  const ParameterString<double> MAX_LINE_LINE_DEV;
  const ParameterString<double>  STEP_THRESHOLD;
  const ParameterString<double>  STEP_LENGTH;
  const ParameterString<double>  MAX_LINE_LINE_DEV;
  const ParameterString<double>  MAX_LINE_SPACE_DEV;
  const ParameterString<double>  MIN_LINE_WIDTH;
  const ParameterString<double>  MIN_LENGTH_NP_ZONE; **/



}  // namespace road_object_detection

#endif  // NO_PASSING_ZONE_CLASSIFIER_H
