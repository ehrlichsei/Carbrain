#include "polygon_algorithm.h"
THIRD_PARTY_HEADERS_BEGIN
#include <ros/console.h>
#include <boost/range/algorithm.hpp>
#include <limits>
THIRD_PARTY_HEADERS_END

namespace environmental_model {

Polygon createPolygon(const common::EigenAlignedVector<Eigen::Vector3d>& hull_points) {
  if (hull_points.empty()) {
    return {};
  }

  Points points;
  points.reserve(hull_points.size() + 1);
  boost::transform(hull_points,
                   std::back_inserter(points),
                   [&](const auto& hull_point) {
                     return Point(hull_point.x(), hull_point.y());
                   });
  points.emplace_back(hull_points.front().x(), hull_points.front().y());

  Polygon poly;
  boost::geometry::assign_points(poly, points);
  boost::geometry::correct(poly);
  return poly;
}

// collision detection algorithm using the Seperating Axis Theorem:
// https://stackoverflow.com/questions/753140/how-do-i-determine-if-two-convex-polygons-intersect?rq=1
bool PolygonAlgorithm::checkConvexPolygonForIntersection(
    common::EigenAlignedVector<Eigen::Vector3d> first_polygon,
    common::EigenAlignedVector<Eigen::Vector3d> second_polygon) {
  if (first_polygon.empty() || second_polygon.empty()) {
    return false;
  }

  first_polygon.push_back(first_polygon.front());
  second_polygon.push_back(second_polygon.front());

  return !(testPolygonSidesForSeperatingAxis(first_polygon, second_polygon) ||
           testPolygonSidesForSeperatingAxis(second_polygon, first_polygon));
}


bool PolygonAlgorithm::testPolygonSidesForSeperatingAxis(
    const common::EigenAlignedVector<Eigen::Vector3d>& axis_polygon,
    const common::EigenAlignedVector<Eigen::Vector3d>& point_polygon) {
  if (axis_polygon.size() < 4) {
    return false;
  }

  // calculate the orientation of the polygon (clockwise vs. anti-clockwise)
  // see https://en.wikipedia.org/wiki/Curve_orientation
  double orientation_sign = 0.0;
  const auto end_it = axis_polygon.end() - 2;
  for (auto it = axis_polygon.begin(); it != end_it; it++) {
    // clang-format off
     Eigen::Matrix3d orientation_matrix;
     orientation_matrix << 1, it->x(),     it->y(),
                           1, (it+1)->x(), (it+1)->y(),
                           1, (it+2)->x(), (it+2)->y();
    // clang-format on
    const double epsilon = 0.001;
    const double orientation_det = orientation_matrix.determinant();
    if (std::fabs(orientation_det) < epsilon) {
      if (it == end_it - 1) {
        ROS_ERROR("can't determine orientation of base area polygon");
        return false;
      }
    } else {
      orientation_sign = common::math::sgn(orientation_det);
      break;
    }
  }

  return (axis_polygon.end() !=
          std::adjacent_find(axis_polygon.begin(),
                             axis_polygon.end(),
                             [point_polygon, orientation_sign](
                                 const Eigen::Vector3d& a, const Eigen::Vector3d& b) {
                               const Eigen::Vector3d direction = b - a;
                               Eigen::Vector3d normal;
                               normal(0) = direction(1);
                               normal(1) = -direction(0);
                               normal *= orientation_sign;
                               const float d = normal.dot(b);
                               return std::all_of(point_polygon.begin(),
                                                  point_polygon.end() - 1,
                                                  [normal, d](const Eigen::Vector3d& c) {
                                                    return (normal.dot(c) - d >= 0.0);
                                                  });
                             }));
}

std::pair<double, double> distanceForClustering(const Polygon& a, const Polygon& b) {
  return distanceForMatching(a, b);
  /*if (boost::geometry::num_points(a) == 0 || boost::geometry::num_points(b) ==
  0) {
    ROS_WARN("empty polygon in distanceForClustering");
    return std::numeric_limits<double>::max();
  }
  std::deque<Polygon> i;
  Point a_centroid, b_centroid;
  boost::geometry::centroid(a, a_centroid);
  boost::geometry::centroid(b, b_centroid);
  const double distance_centroids = boost::geometry::distance(a_centroid,
  b_centroid);
  return distance_centroids;*/
}

std::pair<double, double> distanceForMatching(const Polygon& a, const Polygon& b) {
  if (boost::geometry::num_points(a) == 0 || boost::geometry::num_points(b) == 0) {
    ROS_WARN("empty polygon in distanceForMatching");
    return std::make_pair(std::numeric_limits<double>::max(),
                          std::numeric_limits<double>::max());
  }
  std::deque<Polygon> i;
  Point a_centroid{0, 0}, b_centroid{0, 0};
  boost::geometry::centroid(a, a_centroid);
  boost::geometry::centroid(b, b_centroid);
  auto distance = [](const auto& poly, const auto& centroid_other_poly) {
    if (boost::geometry::within(centroid_other_poly, poly)) {
      return 0.0;
    }
    return boost::geometry::distance(centroid_other_poly, poly);
  };
  // const double distance_centroids = boost::geometry::distance(a_centroid,
  // b_centroid);
  // return distance_centroids;
  return std::make_pair(std::min(distance(a, b_centroid), distance(b, a_centroid)),
                        boost::geometry::distance(a_centroid, b_centroid));
}

Polygon calculateConvexHullPolygon(const Polygon& a, const Polygon& b) {
  if (boost::geometry::num_points(a) < 3) {
    return Polygon(b);
  }
  if (boost::geometry::num_points(b) < 3) {
    return Polygon(a);
  }
  if (boost::geometry::intersects(a)) {
    ROS_ERROR_THROTTLE(0.4, "polygon self intersection");
    return Polygon(b);
  }
  if (boost::geometry::intersects(b)) {
    ROS_ERROR_THROTTLE(0.4, "polygon self intersection");
    return Polygon(a);
  }
  std::deque<Polygon> polygons;
  MultiPolygon mp;
  boost::geometry::union_(a, b, mp);

  Polygon hull_polygon;
  boost::geometry::convex_hull(mp, hull_polygon);
  return hull_polygon;
}

std::vector<Eigen::Vector3d> toEigen3d(const Polygon& poly) {
  std::vector<Point> points(boost::geometry::exterior_ring(poly));
  std::vector<Eigen::Vector3d> result;
  result.reserve(points.size());
  for (const auto& point : points) {
    result.emplace_back(point.x(), point.y(), 0);
  }
  return result;
}

bool calcCentroid(const Polygon& p, Eigen::Vector3d& centroid) {
  if (boost::geometry::num_points(p) == 0) {
    ROS_WARN_THROTTLE(1, "empty polygon in getCentroid");
    return false;
  }
  Point centroid_point{0, 0};
  boost::geometry::centroid(p, centroid_point);
  centroid = Eigen::Vector3d(centroid_point.x(), centroid_point.y(), 0);
  return true;
}

} // namespace environmental_model
