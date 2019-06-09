#include "extrinsic_calibration.h"

#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <ros/console.h>
#include "common/camera_transformation_parameters.h"

#include <iostream>
#include <string>

using namespace std;
using namespace ros;
using namespace common;

const ParameterString<double> ExtrinsicCalibration::CHESSBOARD_DISTANCE_X(
    "chessboard/distance_x");
const ParameterString<double> ExtrinsicCalibration::CHESSBOARD_DISTANCE_Y(
    "chessboard/distance_y");
const ParameterString<int> ExtrinsicCalibration::CHESSBOARD_PATTERN_HEIGHT(
    "chessboard/patternSize_height");
const ParameterString<int> ExtrinsicCalibration::CHESSBOARD_PATTERN_WIDTH(
    "chessboard/patternSize_width");
const ParameterString<double> ExtrinsicCalibration::CHESSBOARD_EDGE_LENGTH(
    "chessboard/edge_Length");



ExtrinsicCalibration::ExtrinsicCalibration(ParameterInterface *parameters)
    : parameters_ptr_(parameters) {
  registerParams(parameters);
}


void ExtrinsicCalibration::registerParams(ParameterInterface *parameters) {
  // camera_params
  parameters->registerParam(CAMERA_FOCAL_LENGTH_X);
  parameters->registerParam(CAMERA_FOCAL_LENGTH_Y);
  parameters->registerParam(CAMERA_OPTICAL_CENTER_X);
  parameters->registerParam(CAMERA_OPTICAL_CENTER_Y);
  // chessboard_params
  parameters->registerParam(CHESSBOARD_DISTANCE_X);
  parameters->registerParam(CHESSBOARD_DISTANCE_Y);
  parameters->registerParam(CHESSBOARD_PATTERN_HEIGHT);
  parameters->registerParam(CHESSBOARD_PATTERN_WIDTH);
  parameters->registerParam(CHESSBOARD_EDGE_LENGTH);
}

vector<cv::Point3f> ExtrinsicCalibration::create3DChessboardCorners(cv::Size boardSize) {
  // This function creates the 3D points of your chessboard in its own
  // coordinate system||Coords-Origin is in the left lower corner of the chess
  // that are transformed into the Coords-System of the rear-wheel

  vector<cv::Point3f> corners;
  // float fchessboardOrigin_y = 0.321f * 1000.0f;
  // float fchessboardOrigin_x = 0.5675f * 1000.0f;


  for (int i = boardSize.height - 1; i >= 0; i--) {
    for (int j = 0; j < boardSize.width; j++) {
      corners.emplace_back(
          float(i * parameters_ptr_->getParam(CHESSBOARD_EDGE_LENGTH) +
                parameters_ptr_->getParam(CHESSBOARD_DISTANCE_X)),
          float(-j * parameters_ptr_->getParam(CHESSBOARD_EDGE_LENGTH) +
                parameters_ptr_->getParam((CHESSBOARD_DISTANCE_Y))),
          0.0);
    }
  }

  return corners;
}


// intrinsischeKamera-Matrix füllen
// Focal length is given in [mm] and we want to have extrinsic params in [m]
// Center Point is given in pixels
void ExtrinsicCalibration::fillIntrinsicMatrix(cv::Mat *cameraMatrix) {


  cameraMatrix->at<double>(0, 0) = parameters_ptr_->getParam(CAMERA_FOCAL_LENGTH_X);
  cameraMatrix->at<double>(0, 1) = 0;
  cameraMatrix->at<double>(0, 2) = parameters_ptr_->getParam(CAMERA_OPTICAL_CENTER_X);
  cameraMatrix->at<double>(1, 0) = 0;
  cameraMatrix->at<double>(1, 1) = parameters_ptr_->getParam(CAMERA_FOCAL_LENGTH_Y);
  cameraMatrix->at<double>(1, 2) = parameters_ptr_->getParam(CAMERA_OPTICAL_CENTER_Y);
  cameraMatrix->at<double>(2, 0) = 0;
  cameraMatrix->at<double>(2, 1) = 0;
  cameraMatrix->at<double>(2, 2) = 1;
}

void ExtrinsicCalibration::printExtrinsicParams(const cv::Mat &SourceImage, cv::Mat &debug) {
  // Input Variables
  // const float chessboardEdgeLength =
  //    0.06f * 1000.0f;   // Kantenlange eines Schachbrettfelds
  cv::Size patternSize;  // interior number of corners of the Chessboard
  patternSize.width = parameters_ptr_->getParam(CHESSBOARD_PATTERN_WIDTH);
  patternSize.height = parameters_ptr_->getParam(CHESSBOARD_PATTERN_HEIGHT);

  cv::Mat image_debug = debug;
  cv::Mat cameraMatrix(
      3,
      3,
      cv::DataType<double>::type);  // KameraMatrix with intrinsic Parameters
  cv::Mat distortionCoefficients =
      cv::Mat::zeros(8, 1, CV_64F);  // Distortion Parameter (Hier alle = 0)

  // Empty Output Variables, that get filled
  vector<cv::Point2f> corners2D;  // this will be filled by the detected
                                  // corners
  cv::Mat rvec(3, 1, cv::DataType<double>::type);  // this will be filled by
                                                   // solvePnP (rotationvector)
  cv::Mat tvec(3, 1, cv::DataType<double>::type);  // this will be filled by
  // solvePnP (translationvector)

  // Pointers to empty Variables, that get filled
  cv::Mat *pcameraMatrix = &cameraMatrix;

  // Filling of empty-Variables
  fillIntrinsicMatrix(pcameraMatrix);
  vector<cv::Point3f> corners3D =
      ExtrinsicCalibration::create3DChessboardCorners(patternSize);

  bool found = cv::findChessboardCorners(SourceImage, patternSize, corners2D);
  // Processing: 1. Finding Points on Chessboard; 2. Drawing the found corners
  // on the debug_image
  if (found) {
    if (corners2D.front().x > corners2D.back().x) {
      std::reverse(corners2D.begin(), corners2D.end());
    }

    cv::solvePnP(corners3D, corners2D, cameraMatrix, distortionCoefficients, rvec, tvec);
    vector<cv::Point2f> reprojectedPoints;
    cv::projectPoints(corners3D, rvec, tvec, cameraMatrix, distortionCoefficients, reprojectedPoints);
    cv::Mat substraction;
    cv::subtract(reprojectedPoints, corners2D, substraction);
    double total_reprojection_error = cv::norm(substraction);
    double mean_reprojection_error =
        total_reprojection_error / (substraction.size().height * substraction.size().width);


    // Formatting eulerian rotation vector to rotation-Matrix
    /*
    std::cout << "rvec: " << rvec << endl;
    std::cout << "tvec: " << tvec << endl;
    std::cout << "rotMatrix: " << rotMatrix << endl;
    */

    // Store best calibration parameters, eulerian angles not included
    if (mean_reprojection_error < smallest_mean_reprojection_error) {
      cv::Mat rotMatrix;
      smallest_mean_reprojection_error = mean_reprojection_error;
      best_rvec = rvec;
      best_tvec = tvec;
      cv::Rodrigues(best_rvec, rotMatrix);
      ROS_INFO_STREAM("eulerian-rot-vector: " << best_rvec);
      ROS_INFO_STREAM("translation-vector: " << best_tvec);
      ROS_INFO_STREAM("rot-matrix: " << rotMatrix);
      ROS_INFO("mean reprojection error: %lf", smallest_mean_reprojection_error);
      // yaml-output
      ROS_INFO_STREAM("Extrinsic_Parameters yaml-format:"
                      << endl
                      << "r11: " << rotMatrix.at<double>(0, 0) << endl
                      << "r12: " << rotMatrix.at<double>(0, 1) << endl
                      << "r13: " << rotMatrix.at<double>(0, 2) << endl
                      << "r21: " << rotMatrix.at<double>(1, 0) << endl
                      << "r22: " << rotMatrix.at<double>(1, 1) << endl
                      << "r23: " << rotMatrix.at<double>(1, 2) << endl
                      << "r31: " << rotMatrix.at<double>(2, 0) << endl
                      << "r32: " << rotMatrix.at<double>(2, 1) << endl
                      << "r33: " << rotMatrix.at<double>(2, 2) << endl
                      << "t1:  " << best_tvec.at<double>(0, 0) << endl
                      << "t2:  " << best_tvec.at<double>(1, 0) << endl
                      << "t3:  " << best_tvec.at<double>(2, 0) << endl);
    }
  }
  cv::drawChessboardCorners(image_debug, patternSize, corners2D, found);
}
