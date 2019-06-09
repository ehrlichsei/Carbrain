#ifndef ROAD_WATCHER_DEBUG_H
#define ROAD_WATCHER_DEBUG_H

#include "../road_watcher.h"

class RoadWatcherDebug : public RoadWatcher {
 public:
  RoadWatcherDebug(RoadWatcher&& road_watcher, cv::Mat* debug_image_ptr);

  /*!
   * \brief creates polynomial for the middle lane; creates and applies scan
   * line pattern finding feature points; clusters these feature points using
   * simplified DBScan
   * \param img_gray gray level image
   * \param points the lane points detected by lane_detection
   * \param clusters the found feature point clusters
   * \param middle_polynomial the polynomial representing the middle lane;
   * created by shifting, fusing & fitting the input lane points
   */
  virtual void scanRoadAndFindClusters(const cv::Mat& img_gray,
                                       const LineVehiclePoints& points,
                                       std::vector<FeaturePointCluster>& clusters,
                                       common::DynamicPolynomial& middle_polynomial) const override;

  //! \brief uses cv::LineIterator to check along scan lines for high gray level
  //! gradients by correlating with a reference function (e.g. tanh) and
  //! thresholding the correlation value.
  virtual ImagePoints apply1dGradientDetector(const cv::Mat& img,
                                              const ScanLines& scan_lines) const override;

  /*!
   * \brief createScanLineGrid creates scan pattern on left and right lane;
   * points will be sampled from the fitted polynomials with consistent distance
   * in vehicle x coordinates.
   */
  virtual ScanLines createScanLineGrid(const LineVehiclePoints& points,
                                       const common::DynamicPolynomial& middle_polynomial) const override;

  virtual VehiclePoints allPointToMiddle(LineVehiclePoints points) const override;

 private:
  mutable cv::Mat* img_debug_;

  void visualize(const ScanLines& scan_lines) const;

  void visualize(const ImagePoints& feature_points,
                 const cv::Scalar& color = CV_RGB(0, 255, 255)) const;

  void visualize(const std::vector<FeaturePointCluster>& clusters) const;

  void drawBoundingRectAroundFeaturePoints(const common::Vector2iVector& feature_points) const;
};

#endif  // ROAD_WATCHER_DEBUG_H
