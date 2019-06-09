#ifndef OPENCV_EIGEN_CONVERSIONS
#define OPENCV_EIGEN_CONVERSIONS

#include <opencv2/core/core.hpp>
#include <eigen3/Eigen/Dense>
#include <boost/range/algorithm/transform.hpp>
#include "common/lift.h"
#include "perception_types.h"

inline ImagePoint toEigen(const cv::Point& point) {
  return ImagePoint(point.x, point.y);
}

inline ImagePointExact toEigen(const cv::Point2f& point) {
  return ImagePointExact(point.x, point.y);
}

inline ImagePoint cvPointToImagePoint(const cv::Point& point) {
  return toEigen(point);
}

inline ImagePointExact cvPoint2fToImagePointExact(const cv::Point2f& point) {
  return toEigen(point);
}

inline ImagePointExact cvPointToImagePointExact(const cv::Point& point) {
  return ImagePointExact(point.x, point.y);
}

inline cv::Point toCV(const ImagePoint& point) {
  return cv::Point(point(0), point(1));
}

inline cv::Point2f toCV(const ImagePointExact& point) {
  return cv::Point2f(static_cast<float>(point(0)), static_cast<float>(point(1)));
}

inline cv::Point imagePointToCvPoint(const ImagePoint& vector) {
  return toCV(vector);
}

inline cv::Point2f imagePointExactToCvPoint2f(const ImagePointExact& vector) {
  return toCV(vector);
}

inline cv::Point imagePointExactToCvPoint(const ImagePointExact& vector) {
  return {static_cast<int>(std::round(vector.x())),
          static_cast<int>(std::round(vector.y()))};
}

namespace detail {
template <class PointType>
using deriveEigenType = decltype(toEigen(std::declval<PointType>()));

template <class PointType>
using deriveCvType = decltype(toCV(std::declval<PointType>()));
}

template <class PointType>
auto toEigen(const std::vector<PointType>& points) {
  common::EigenAlignedVector<detail::deriveEigenType<PointType>> target_vector;
  target_vector.reserve(points.size());
  boost::transform(points, std::back_inserter(target_vector), LIFT(toEigen));
  return target_vector;
}

inline ImagePoints cvPointToImagePoint(const std::vector<cv::Point>& points) {
  return toEigen(points);
}

inline ImagePointsExact cvPoint2fToImagePointExact(const std::vector<cv::Point2f>& points) {
  return toEigen(points);
}

inline ImagePointsExact cvPointToImagePointExact(const std::vector<cv::Point>& points) {
  ImagePointsExact image_points;
  image_points.reserve(points.size());
  boost::transform(points, std::back_inserter(image_points), LIFT(cvPointToImagePointExact));
  return image_points;
}

template <class PointType>
auto toCV(const common::EigenAlignedVector<PointType>& points) {
  std::vector<detail::deriveCvType<PointType>> target_vector;
  target_vector.reserve(points.size());
  boost::transform(points, std::back_inserter(target_vector), LIFT(toCV));
  return target_vector;
}

inline std::vector<cv::Point> imagePointToCvPoint(const ImagePoints& vectors) {
  return toCV(vectors);
}

inline std::vector<cv::Point2f> imagePointExactToCvPoint2f(const ImagePointsExact& vectors) {
  return toCV(vectors);
}

inline std::vector<cv::Point> imagePointExactToCvPoint(const ImagePointsExact& vectors) {
  std::vector<cv::Point> cvpoints;
  cvpoints.reserve(vectors.size());
  boost::transform(vectors, std::back_inserter(cvpoints), LIFT(imagePointExactToCvPoint));
  return cvpoints;
}

#endif  // OPENCV_EIGEN_CONVERSIONS
