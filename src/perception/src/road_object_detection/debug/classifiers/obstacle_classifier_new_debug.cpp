#include "obstacle_classifier_new_debug.h"
#include <opencv_eigen_conversions.h>
#include <opencv_utils.h>

THIRD_PARTY_HEADERS_BEGIN
#include <opencv2/opencv.hpp>
#include <sstream>
#include <string>
THIRD_PARTY_HEADERS_END



#include <common/eigen_utils.h>
#include <common/pca_eigen.h>
#include <opencv_eigen_conversions.h>
#include <perception_types.h>
#include <numeric>
//#include <common/polynomial.h>
//#include <common/polynomialfit.h>
#include <common/angle_conversions.h>
#include <opencv_utils.h>

#include "../../../utils/foot_finder.h"
#include "../../../utils/linear_clustering.h"
#include "../../../utils/step_detection.h"



#include <common/polynomial.h>
#include <common/polynomialfit.h>


THIRD_PARTY_HEADERS_BEGIN
#include <cv.h>
#include <Eigen/Dense>
#include <boost/range/algorithm_ext.hpp>
#include <cmath>
#include "boost/range/algorithm.hpp"

THIRD_PARTY_HEADERS_END

namespace road_object_detection {


/******************************************************************************
 *****ObstacleCLassifierNew***************************************************
 *****************************************************************************/

ObstacleClassifierNewDebug::ObstacleClassifierNewDebug(const common::CameraTransformation *const camera_transformation,
                                                       ParameterInterface *const parameters_ptr,
                                                       DebugImages *debug_images)
    : ObstacleClassifierNew(camera_transformation, parameters_ptr),
      ClassifierDebug(debug_images) {}



RoadObjects ObstacleClassifierNewDebug::classify(const Features &features) {
  ROS_DEBUG("start classiífy obstacle");
  cluster_id = features.cluster.id;
  RoadObjects ret = ObstacleClassifierNew::classify(features);

  for (const auto &o : ret) {
    ImagePoints ips;
    camera_transformation_->transformVehicleToImage(o->base_hull_polygon_in_vehicle, &ips);
    cv::polylines(
        *debug_images->getCameraImage(), toCV(ips), true, cv::Scalar(0, 255, 255), 1);
  }

  ROS_DEBUG("finish classiífy obstacle");
  return ret;
}



VehiclePoint ObstacleClassifierNewDebug::getLaneDirectionVector(
    const common::DynamicPolynomial &middle_lane_polynomial,
    const VehiclePoint &ground_point) const {
  const VehiclePoint v = ObstacleClassifierNew::getLaneDirectionVector(
      middle_lane_polynomial, ground_point);
  //    const ImagePoint ip1 =
  //    camera_transformation_->transformVehicleToImage(ground_point);
  //    const ImagePoint ip2 =
  //    camera_transformation_->transformVehicleToImage(ground_point+0.1*v);

  //    cv::arrowedLine(*debug_images->getCameraImage(),
  //                    toCV(ip1),
  //                    toCV(ip2),
  //                    cv::Scalar(50, 200, 200),
  //                    2 /*thickness*/);

  return v;
}



ImagePoints ObstacleClassifierNewDebug::getVolumeRoi(const ImagePoints &area_roi,
                                                     const cv::Size &img_size) const {

  const ImagePoints volume_roi = ObstacleClassifierNew::getVolumeRoi(area_roi, img_size);

  const auto cv_points = toCV(volume_roi);

  cv::polylines(
      *debug_images->getCameraImage(), cv_points, true, cv::Scalar(255, 50, 200), 1);

  return volume_roi;
}



ScanLines ObstacleClassifierNewDebug::generateScanLines(const cv::Rect &rect,
                                                        const double edge_angle,
                                                        double &out_scan_line_spacing,
                                                        ImagePointExact &out_scan_direction) const {
  const ScanLines scan_lines = ObstacleClassifierNew::generateScanLines(
      rect, edge_angle, out_scan_line_spacing, out_scan_direction);

  // cv::rectangle(*debug_images->getCameraImage(), rect, cv::Scalar(0, 0, 100),
  // 1);

  //    for (const ScanLine &scan_line : scan_lines) {
  //      cv::arrowedLine(*debug_images->getCameraImage(),
  //                      toCV(scan_line.start),
  //                      toCV(scan_line.end),
  //                      cv::Scalar(50, 200, 200),
  //                      1 /*thickness*/);
  //    }
  return scan_lines;
}



ImagePoints ObstacleClassifierNewDebug::getFeaturePoints(const cv::Mat &image_complete,
                                                         cv::InputArray &roi_defining_points,
                                                         const ScanLines &scan_lines,
                                                         const DetectionMode &mode) const {

  const ImagePoints feature_points = ObstacleClassifierNew::getFeaturePoints(
      image_complete, roi_defining_points, scan_lines, mode);

  for (const ImagePoint &ip : feature_points) {
    cv::circle(*debug_images->getCameraImage(),
               toCV(ip),
               1, /*radius*/
               cv::Scalar(255, 20, 20),
               1 /*thickness*/);
  }
  return feature_points;
}


// void ObstacleClassifierNewDebug::erasePointsOnLaneLines(
//    ImagePoints &in_out_ips,
//    const boost::optional<PolynomialWithXRange> poly_left,
//    const boost::optional<PolynomialWithXRange> poly_middle,
//    const boost::optional<PolynomialWithXRange> poly_right,
//    const double rem_distance_m_thld) const {
//  ObstacleClassifierNew::erasePointsOnLaneLines(
//      in_out_ips, poly_left, poly_middle, poly_right, rem_distance_m_thld);

//  // draw area within which points are deleteded
//  if (poly_left) {
//    const std::size_t N = 10;
//    std::vector<cv::Point> poly(2 * N);
//    const double dx = poly_left->x_range.width() / N;
//    for (std::size_t i = 0; i < N; i++) {
//      const double x = poly_left->x_range.start + dx;
//      const VehiclePoint p(x, poly_left->polynomial(x), 0);
//      const VehiclePoint p1 =
//          p -
//          ObstacleClassifierNew::getLaneDirectionVector(poly_left->polynomial,
//          x);
//      const VehiclePoint p2 =
//          p +
//          ObstacleClassifierNew::getLaneDirectionVector(poly_left->polynomial,
//          x);
//      poly[i] = toCV(camera_transformation_->transformVehicleToImage(p1));7
//      poly[i + N] = toCV(camera_transformation_->transformVehicleToImage(p2));
//    }

//    cv::polylines(
//        *debug_images->getCameraImage(), toInputArray(poly), true,
//        cv::Scalar(20, 255, 255));
//  }
//}



void ObstacleClassifierNewDebug::getFrontShapeFeaturePoints(const Features &features,
                                                            const ImagePoints &area_roi,
                                                            ImagePoints &fp_bottom_out,
                                                            ImagePoints &fp_vertical_left_out,
                                                            ImagePoints &fp_vertical_right_out) const {

  ObstacleClassifierNew::getFrontShapeFeaturePoints(
      features, area_roi, fp_bottom_out, fp_vertical_left_out, fp_vertical_right_out);



  for (const ImagePoint &ip : fp_vertical_left_out) {
    cv::circle(*debug_images->getCameraImage(),
               toCV(ip),
               1, /*radius*/
               cv::Scalar(0, 0, 255),
               1 /*thickness*/);
  }

  for (const ImagePoint &ip : fp_vertical_right_out) {
    cv::circle(*debug_images->getCameraImage(),
               toCV(ip),
               1, /*radius*/
               cv::Scalar(200, 150, 255),
               1 /*thickness*/);
  }
  for (const ImagePoint &ip : fp_bottom_out) {
    cv::circle(*debug_images->getCameraImage(),
               toCV(ip),
               1, /*radius*/
               cv::Scalar(0, 255, 0),
               1 /*thickness*/);
  }
}



boost::optional<ObstacleFrontModel> ObstacleClassifierNewDebug::fitObstacleFrontModel(
    const ImagePoints &bottom_pts,
    const ImagePoints &vertical_left_pts,
    const ImagePoints &vertical_right_pts,
    const Features &features) const {


  boost::optional<ObstacleFrontModel> model = ObstacleClassifierNew::fitObstacleFrontModel(
      bottom_pts, vertical_left_pts, vertical_right_pts, features);

  if (model) {
    const ObstacleFrontModel &m = model.get();

    m.rosDebugAll();
    ROS_DEBUG("ground line visible : %d",
              m.isSecondGroundLineClearlyVisible(
                  model_params_.gndline2_visible_angle_thld, camera_transformation_));

    cv::polylines(
        *debug_images->getCameraImage(), m.asConvexPolygonCV(0), true, cv::Scalar(0, 0, 255), 1);

    if (m.getModelType() == ObstacleFrontModel::ModelType::L_2ND_GND_LINE) {
      cv::polylines(*debug_images->getCameraImage(),
                    m.asConvexPolygonCV(1),
                    true,
                    cv::Scalar(0, 0, 255),
                    1);
    }
  }
  //    std::stringstream ss;
  //    ss.precision(2);
  //    ss << "o = " << m.getMostRecentScore();
  //    cv::putText(*debug_images->getCameraImage(),
  //                ss.str(),
  //                toCV(m.getCenter(0)),
  //                CV_FONT_HERSHEY_PLAIN,
  //                1.0,
  //                cv::Scalar(0, 255, 0));

  else {
    ROS_DEBUG("No Model found.");
  }

  return model;
}

boost::optional<VehiclePoint> ObstacleClassifierNewDebug::estimateObstacleEndPoint(
    const Line3d &model_ground_line, const Features &features) const {

  const boost::optional<VehiclePoint> end_point =
      ObstacleClassifierNew::estimateObstacleEndPoint(model_ground_line, features);
  //  if (end_point) {
  //    cv::circle(*debug_images->getCameraImage(),
  //               toCV(camera_transformation_->transformVehicleToImage(end_point.get())),
  //               5, /*radius*/
  //               cv::Scalar(0, 0, 255),
  //               2 /*thickness*/);
  //  }
  return end_point;
}


ScanLine ObstacleClassifierNewDebug::generateBaseHullEstmateScanLine(const Line3d &model_ground_line,
                                                                     double z0) const {
  const ScanLine scan_line =
      ObstacleClassifierNew::generateBaseHullEstmateScanLine(model_ground_line, z0);


  //  cv::line(*debug_images->getCameraImage(),
  //           toCV(scan_line.start),
  //           toCV(scan_line.end),
  //           cv::Scalar(155, 0, 255),
  //           1);

  return scan_line;
}



ObstacleVertices ObstacleClassifierNewDebug::estimateBaseHullPolygon(
    const ObstacleFrontModel &model, const Features &features) const {
  const ObstacleVertices vertices =
      ObstacleClassifierNew::estimateBaseHullPolygon(model, features);

  if (vertices.vertices_detection_state.size() != vertices.vertices.size()) {
    ROS_ERROR(
        "Obstacle vertices and vertex detecttion state vector must have same "
        "size.");
    return vertices;
  }

  ImagePoints ips;
  camera_transformation_->transformVehicleToImage(vertices.vertices, &ips);
  cv::polylines(
      *debug_images->getCameraImage(), toCV(ips), true, cv::Scalar(0, 255, 255), 1);
  // mark detected vertices fat
  for (std::size_t i = 0; i < ips.size(); i++)
    if (vertices.vertices_detection_state[i] == Obstacle::DetectionState::VERTEX_DETECTED) {
      cv::circle(*debug_images->getCameraImage(),
                 toCV(ips[i]),
                 1, /*radius*/
                 cv::Scalar(255, 50, 50),
                 7 /*thickness*/);
    } else if (vertices.vertices_detection_state[i] == Obstacle::DetectionState::VERTEX_ESTIMATED) {
      cv::circle(*debug_images->getCameraImage(),
                 toCV(ips[i]),
                 1, /*radius*/
                 cv::Scalar(0, 0, 255),
                 7 /*thickness*/);
    }
  return vertices;
}
}  // namespace road_object_detection
