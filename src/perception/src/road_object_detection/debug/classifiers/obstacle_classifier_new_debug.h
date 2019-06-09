#ifndef OBSTACLE_CLASSIFIER_NEW_DEBUG_H
#define OBSTACLE_CLASSIFIER_NEW_DEBUG_H

#include "../classifier_debug.h"
#include "../../classifiers/obstacle_classifier_new.h"

namespace road_object_detection {

class ObstacleClassifierNewDebug : public virtual ObstacleClassifierNew,
                                   public virtual ClassifierDebug {

 public:
  ObstacleClassifierNewDebug(const common::CameraTransformation* const camera_transformation,
                             ParameterInterface* const parameters_ptr,
                             DebugImages* debug_images);

  RoadObjects classify(const Features& features) override;


 private:
  VehiclePoint getLaneDirectionVector(const common::DynamicPolynomial& middle_lane_polynomial,
                                      const VehiclePoint& ground_point) const override;


  ImagePoints getVolumeRoi(const ImagePoints& area_roi, const cv::Size& img_size) const override;



  ScanLines generateScanLines(const cv::Rect& rect,
                              const double edge_angle,
                              double& out_scan_line_spacing,
                              ImagePointExact& out_scan_direction) const override;


  ImagePoints getFeaturePoints(const cv::Mat& image_complete,
                               cv::InputArray& roi_defining_points,
                               const ScanLines& scan_lines,
                               const DetectionMode& mode) const override;



  //  void erasePointsOnLaneLines(ImagePoints& in_out_ips,
  //                              const boost::optional<PolynomialWithXRange>
  //                              poly_left,
  //                              const boost::optional<PolynomialWithXRange>
  //                              poly_middle,
  //                              const boost::optional<PolynomialWithXRange>
  //                              poly_right,
  //                              const double rem_distance_m_thld) const
  //                              override;


  void getFrontShapeFeaturePoints(const Features& features,
                                  const ImagePoints& area_roi,
                                  ImagePoints& fp_bottom_out,
                                  ImagePoints& fp_vertical_left_out,
                                  ImagePoints& fp_vertical_right_out) const override;



  boost::optional<ObstacleFrontModel> fitObstacleFrontModel(const ImagePoints& bottom_pts,
                                                            const ImagePoints& vertical_left_pts,
                                                            const ImagePoints& vertical_right_pts,
                                                            const Features& features) const override;


  boost::optional<VehiclePoint> estimateObstacleEndPoint(const Line3d& model_ground_line,
                                                         const Features& features) const override;


  ScanLine generateBaseHullEstmateScanLine(const Line3d& model_ground_line,
                                           double z0) const override;

  ObstacleVertices estimateBaseHullPolygon(const ObstacleFrontModel& model,
                                           const Features& features) const override;


  unsigned int cluster_id = 0;

  mutable cv::Point debug_canny_patch_position;
};

}  // namespace road_object_detection

#endif  // OBSTACLE_CLASSIFIER_NEW_DEBUG_H
