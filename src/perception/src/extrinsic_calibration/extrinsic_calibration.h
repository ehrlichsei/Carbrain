#ifndef EXTRINSIC_CALIBRATION_H
#define EXTRINSIC_CALIBRATION_H
#include <common/macros.h>

THIRD_PARTY_HEADERS_BEGIN
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
THIRD_PARTY_HEADERS_END

#include "common/parameter_interface.h"
#include "common/parameter_handler.h"
#include "common/camera_transformation_parameters.h"

/*!
 * \brief Publishes extrinsic parameters
 */
class ExtrinsicCalibration {
 public:
  /*!
  * \brief ExtrinsicCalibration is the consstructor. A ros indipendent
  * functionality containing
  * class needs a pointer to a ParameterInterface (in fact a ParameterHandler)
  * to get access to parameters.
  * \param parameters the ParameterInterface
  */
  ExtrinsicCalibration(ParameterInterface *parameters);



  void printExtrinsicParams(const cv::Mat &SourceImage, cv::Mat &debug);

 private:
  void registerParams(ParameterInterface *parameters);
  std::vector<cv::Point3f> create3DChessboardCorners(cv::Size boardSize);
  void fillIntrinsicMatrix(cv::Mat *cameraMatrix);

  double smallest_mean_reprojection_error = 10000.0;
  cv::Mat best_rvec;
  cv::Mat best_tvec;
  /*!
  * \brief parameters_ptr_ is needed for parameter access
  */
  ParameterInterface *parameters_ptr_;

  static const ParameterString<double> CHESSBOARD_DISTANCE_X;
  static const ParameterString<double> CHESSBOARD_DISTANCE_Y;
  static const ParameterString<int> CHESSBOARD_PATTERN_WIDTH;
  static const ParameterString<int> CHESSBOARD_PATTERN_HEIGHT;
  static const ParameterString<double> CHESSBOARD_EDGE_LENGTH;
};


#endif  // EXTRINSIC_CALIBRATION_H
