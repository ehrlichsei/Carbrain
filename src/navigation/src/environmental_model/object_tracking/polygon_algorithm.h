#ifndef POLYGON_ALGORITHM_H
#define POLYGON_ALGORITHM_H

#include "common/macros.h"

THIRD_PARTY_HEADERS_BEGIN
#include <Eigen/Core>
#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/polygon.hpp>
THIRD_PARTY_HEADERS_END
#include "common/math.h"
#include "common/types.h"

namespace environmental_model {
using Point = boost::geometry::model::d2::point_xy<float>;
using Points = std::vector<Point>;

using Polygon = boost::geometry::model::polygon<Point, false, false>;
using MultiPolygon = boost::geometry::model::multi_polygon<Polygon>;

bool calcCentroid(const Polygon& p, Eigen::Vector3d& centroid);
Polygon createPolygon(const common::EigenAlignedVector<Eigen::Vector3d>& hull_points);
std::pair<double, double> distanceForClustering(const Polygon& a, const Polygon& b);
std::pair<double, double> distanceForMatching(const Polygon& a, const Polygon& b);
std::vector<Eigen::Vector3d> toEigen3d(const Polygon& poly);
Polygon calculateConvexHullPolygon(const Polygon& a, const Polygon& b);
class PolygonAlgorithm {
 public:
  PolygonAlgorithm() = default;
  bool testPolygonSidesForSeperatingAxis(
      const common::EigenAlignedVector<Eigen::Vector3d>& axis_polygon,
      const common::EigenAlignedVector<Eigen::Vector3d>& point_polygon);
  bool checkConvexPolygonForIntersection(common::EigenAlignedVector<Eigen::Vector3d> first_polygon,
                                         common::EigenAlignedVector<Eigen::Vector3d> second_polygon);
};

} // namespace environmental_model

#endif  // POLYGON_ALGORITHM_H
