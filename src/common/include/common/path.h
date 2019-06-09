#ifndef PATH_H
#define PATH_H
#include "common/macros.h"

#include "common/adaptors.h"
#include "common/best_score.h"
#include "common/eigen_functors.h"
#include "common/math.h"
#include "common/types.h"
#include "common/unique_erase.h"
#include "polynomial.h"
#include "polynomialfit.h"

THIRD_PARTY_HEADERS_BEGIN
#include <ros/console.h>
#include <Eigen/Geometry>
#include <boost/range/adaptor/transformed.hpp>
#include <cmath>
THIRD_PARTY_HEADERS_END

namespace common {

struct ArcLengthParameterizedPathPoint {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  ArcLengthParameterizedPathPoint(Eigen::Vector2d position, double arc_length)
      : position(position), arc_length(arc_length) {}

  Eigen::Vector2d position = Eigen::Vector2d::Zero();
  double arc_length = 0.0;
};

template <unsigned int POLYNOMIAL_DEGREE>
struct LocalPathApproximation {
  LocalPathApproximation(Polynomial<POLYNOMIAL_DEGREE, double> polynomial, double approximation_x)
      : polynomial(polynomial), approximation_x(approximation_x) {}

  Polynomial<POLYNOMIAL_DEGREE, double> polynomial;
  double approximation_x;
};

template <unsigned int MAX_REGRESSION_POLYNOMIAL_DEGREE = 2u, unsigned int DEFAULT_REGRESSION_POLYNOMIAL_DEGREE = 2u>
class Path {
  static_assert(MAX_REGRESSION_POLYNOMIAL_DEGREE >= DEFAULT_REGRESSION_POLYNOMIAL_DEGREE,
                "DEFAULT_REGRESSION_POLYNOMIAL_DEGREE can be at most "
                "MAX_REGRESSION_POLYNOMIAL_DEGREE");
  static_assert(
      std::numeric_limits<double>::is_iec559,
      "requires IEEE 754 floating point behaviour for error handling");

 public:
  using PathPoints = common::EigenAlignedVector<Eigen::Vector2d>;
  using ArcLengthParameterizedPathPoints =
      common::EigenAlignedVector<ArcLengthParameterizedPathPoint>;

  class RawPath {
   public:
    RawPath(PathPoints ps) : path_(normalizePath(ps)) {}

    RawPath subtract(const double arc_length_offset) const {
      ArcLengthParameterizedPathPoints alps = path_;

      for (auto& alp : alps) {
        alp.arc_length -= arc_length_offset;
      }
      return RawPath(std::move(alps));
    }

    const ArcLengthParameterizedPathPoints& path() const & { return path_; }
    ArcLengthParameterizedPathPoints&& path() && { return std::move(path_); }

   private:
    ArcLengthParameterizedPathPoints path_;

    RawPath(ArcLengthParameterizedPathPoints&& alps) : path_(std::move(alps)) {}

    static ArcLengthParameterizedPathPoints normalizePath(PathPoints ps) {
      std::sort(ps.begin(), ps.end(), common::less_x());
      common::unique_erase(ps,
                           [](const auto& a, const auto& b) {
                             return std::fabs((b.x() - a.x())) < 0.001;
                           });
      if (ps.size() < MAX_REGRESSION_POLYNOMIAL_DEGREE + 1u) {
        throw std::runtime_error("at least " +
                                 std::to_string(MAX_REGRESSION_POLYNOMIAL_DEGREE + 1u) +
                                 " points required to generate path");
      }

      ArcLengthParameterizedPathPoints alps;
      alps.reserve(ps.size());
      alps.emplace_back(ps.front(), 0.0);
      for (auto it = ps.begin() + 1; it != ps.end(); it++) {
        alps.emplace_back(*it, alps.back().arc_length + (*it - *(it - 1)).norm());
      }
      return alps;
    }
  };

  Path(const RawPath& raw_path,
       const Eigen::Vector2d& vehicle_position,
       PathPoints::difference_type number_of_points_for_local_fit)
      : points([&raw_path, &vehicle_position, number_of_points_for_local_fit]() {
          if (number_of_points_for_local_fit % 2 == 0) {
            throw std::runtime_error(
                "number_of_points_for_local_fit has to be odd");
          }
          if (number_of_points_for_local_fit < MAX_REGRESSION_POLYNOMIAL_DEGREE + 1u) {
            throw std::runtime_error(
                "number_of_points_for_local_fit has to be at least "
                "MAX_REGRESSION_POLYNOMIAL_DEGREE + 1");
          }
          // arc_length=0 should correspond to the vehicle position foot point
          // on the path (positive arc_lengths are in front of the vehicle
          // (lookahead not considered))
          const double arc_length_offset = calculateVehicleFootArcLength(
              vehicle_position, raw_path.path(), number_of_points_for_local_fit);
          return raw_path.subtract(arc_length_offset).path();
        }()),
        number_of_points_for_local_fit(number_of_points_for_local_fit) {}

  Path(PathPoints ps, const Eigen::Vector2d& vehicle_position, PathPoints::difference_type number_of_points_for_local_fit)
      : Path(RawPath(std::move(ps)), vehicle_position, number_of_points_for_local_fit) {}

  ArcLengthParameterizedPathPoints points;

  static double tricubeWeightFunction(double distance) {
    assert((distance >= -1. && distance <= 1.) &&
           "distance in weight function in path not in range -1..1");
    const double epsilon = 0.01;  // to avoid numerical garbage in corner cases
    return std::max(0., std::pow(1 - std::pow(std::fabs(distance), 3), 3)) + epsilon;
  }

  template <common::PolynomialDegree DEGREE = DEFAULT_REGRESSION_POLYNOMIAL_DEGREE>
  static Polynomial<DEGREE, double> getLocalPolynomialFit(
      ArcLengthParameterizedPathPoints::const_iterator center_path_point,
      const ArcLengthParameterizedPathPoint& center_point,
      const ArcLengthParameterizedPathPoints& points,
      PathPoints::difference_type number_of_points_for_local_fit) {
    static_assert(DEGREE <= MAX_REGRESSION_POLYNOMIAL_DEGREE,
                  "you can't get a regression polynomial degree above "
                  "MAX_REGRESSION_POLYNOMIAL_DEGREE");

    // center_point is the point to approximate, center_point is the iterator to
    // the path point closest to this point

    auto begin_regression_range = points.begin();
    auto end_regression_range = points.end();
    if (std::distance(points.begin(), center_path_point) < number_of_points_for_local_fit / 2) {
      begin_regression_range = points.begin();
      end_regression_range =
          begin_regression_range +
          std::min(number_of_points_for_local_fit,
                   std::distance(begin_regression_range, points.end()));
    } else if (std::distance(center_path_point, points.end()) <
               number_of_points_for_local_fit / 2 + 1) {
      end_regression_range = points.end();
      begin_regression_range =
          end_regression_range -
          std::min(number_of_points_for_local_fit,
                   std::distance(points.begin(), end_regression_range));
    } else {
      begin_regression_range = center_path_point - number_of_points_for_local_fit / 2;
      end_regression_range = center_path_point + number_of_points_for_local_fit / 2 + 1;
    }

    std::vector<double> weights(std::distance(begin_regression_range, end_regression_range));
    const double max_distance = std::max(
        std::fabs(center_point.position.x() - begin_regression_range->position.x()),
        std::fabs((end_regression_range - 1)->position.x() - center_point.position.x()));
    // scale distance to window 0..1, wider window to avoid the outer points
    // having zero weight (tricube weight function has weight zero for scaled
    // distance 1)
    const double window = max_distance + max_distance / number_of_points_for_local_fit;
    std::transform(begin_regression_range,
                   end_regression_range,
                   weights.begin(),
                   [&center_point, window](const auto& p) {
                     return Path::tricubeWeightFunction(
                         (p.position.x() - center_point.position.x()) / window);
                   });

    return fitToPointsWeighted<DEGREE>(
        boost::make_iterator_range(begin_regression_range, end_regression_range) |
            common::members(&ArcLengthParameterizedPathPoint::position),
        weights);
  }

  template <common::PolynomialDegree DEGREE = DEFAULT_REGRESSION_POLYNOMIAL_DEGREE>
  Polynomial<DEGREE, double> getLocalPolynomialFit(
      ArcLengthParameterizedPathPoints::const_iterator center_path_point,
      const ArcLengthParameterizedPathPoint& center_point) const {
    return getLocalPolynomialFit<DEGREE>(
        center_path_point, center_point, points, number_of_points_for_local_fit);
  }

  template <common::PolynomialDegree DEGREE = DEFAULT_REGRESSION_POLYNOMIAL_DEGREE>
  static LocalPathApproximation<DEGREE> getLocalPathApproximation(
      double arc_length,
      const ArcLengthParameterizedPathPoints& points,
      PathPoints::difference_type number_of_points_for_local_fit) {
    auto it = std::adjacent_find(points.begin(),
                                 points.end(),
                                 [arc_length](const auto& a, const auto& b) {
                                   return a.arc_length <= arc_length &&
                                          arc_length <= b.arc_length;
                                 });

    double approximation_x = 0.0;
    Eigen::Vector2d interpolated_point;
    if (it == points.end()) {
      if (points.front().arc_length > arc_length) {
        it = points.begin();
      } else {
        it = std::prev(points.end());
      }
      // extrapolation at end points
      approximation_x = it->position.x() + (arc_length - it->arc_length);
      interpolated_point = it->position;
    } else {
      // linear interpolation
      const Eigen::Vector2d delta = (it + 1)->position - it->position;

      const double delta_s = arc_length - it->arc_length;
      interpolated_point =
          (delta_s / ((it + 1)->arc_length - it->arc_length)) * delta + it->position;

      const double delta_x_interpolation =
          delta_s / (std::sqrt(1.0 + squared(delta.y() / delta.x())));
      approximation_x = it->position.x() + delta_x_interpolation;
    }

    // is *it or *(it+1) closer to arc_length?
    if (std::distance(it, points.end()) > 1 &&
        std::fabs((it + 1)->arc_length - arc_length) < std::fabs(it->arc_length - arc_length)) {
      it++;
    }

    return LocalPathApproximation<DEGREE>(
        getLocalPolynomialFit<DEGREE>(
            it, ArcLengthParameterizedPathPoint(interpolated_point, arc_length), points, number_of_points_for_local_fit),
        approximation_x);
  }

  template <common::PolynomialDegree DEGREE = DEFAULT_REGRESSION_POLYNOMIAL_DEGREE>
  LocalPathApproximation<DEGREE> getLocalPathApproximation(double arc_length) const {
    return getLocalPathApproximation<DEGREE>(arc_length, points, number_of_points_for_local_fit);
  }

  template <common::PolynomialDegree DEGREE = DEFAULT_REGRESSION_POLYNOMIAL_DEGREE>
  double getY(double arc_length) const {
    return operator()<DEGREE>(arc_length).y();
  }

  template <common::PolynomialDegree DEGREE = DEFAULT_REGRESSION_POLYNOMIAL_DEGREE>
  Eigen::Vector2d operator()(double arc_length) const {
    const LocalPathApproximation<DEGREE> local_path =
        getLocalPathApproximation<DEGREE>(arc_length);
    return {local_path.approximation_x, local_path.polynomial(local_path.approximation_x)};
  }

  template <typename T, typename std::enable_if<std::is_integral<T>::value, int>::type = 0>
  ArcLengthParameterizedPathPoint& operator[](T i) {
    return points.at(i);
  }

  template <typename T, typename std::enable_if<std::is_integral<T>::value, int>::type = 0>
  const ArcLengthParameterizedPathPoint& operator[](T i) const {
    return points.at(i);
  }

  ArcLengthParameterizedPathPoint& back() { return points.back(); }

  const ArcLengthParameterizedPathPoint& back() const { return points.back(); }

  ArcLengthParameterizedPathPoint& front() { return points.front(); }

  const ArcLengthParameterizedPathPoint& front() const {
    return points.front();
  }

  template <common::PolynomialDegree DEGREE = DEFAULT_REGRESSION_POLYNOMIAL_DEGREE>
  double firstDerivative(double arc_length) const {
    const LocalPathApproximation<DEGREE> local_path =
        getLocalPathApproximation<DEGREE>(arc_length);
    const auto derivative_polynomial = local_path.polynomial.calculateFirstDerivative();
    return derivative_polynomial(local_path.approximation_x);
  }

  template <common::PolynomialDegree DEGREE = DEFAULT_REGRESSION_POLYNOMIAL_DEGREE>
  double secondDerivative(double arc_length) const {
    const LocalPathApproximation<DEGREE> local_path =
        getLocalPathApproximation<DEGREE>(arc_length);
    const auto derivative_polynomial = local_path.polynomial.calculateSecondDerivative();
    return derivative_polynomial(local_path.approximation_x);
  }

  template <common::PolynomialDegree DEGREE = DEFAULT_REGRESSION_POLYNOMIAL_DEGREE>
  double curvature(double arc_length) const {
    const LocalPathApproximation<DEGREE> local_path =
        getLocalPathApproximation<DEGREE>(arc_length);
    return local_path.polynomial.calculateCurvature(local_path.approximation_x);
  }

  bool isArcLengthInRange(double arc_length) const {
    return points.front().arc_length <= arc_length && arc_length <= points.back().arc_length;
  }


 private:
  static double findRootCardano(const CubicPolynomial& root_polynomial) {
    // source: https://math.vanderbilt.edu/schectex/courses/cubic/
    const double p = -root_polynomial.getCoefficients()[2] /
                     (3.0 * root_polynomial.getCoefficients()[3]);
    const double q =
        std::pow(p, 3.0) +
        (root_polynomial.getCoefficients()[2] * root_polynomial.getCoefficients()[1] -
         3.0 * root_polynomial.getCoefficients()[3] *
             (root_polynomial.getCoefficients()[0])) /
            (6.0 * std::pow(root_polynomial.getCoefficients()[3], 2.0));
    const double s = std::pow(root_polynomial.getCoefficients()[1] /
                                      (3.0 * root_polynomial.getCoefficients()[3]) -
                                  std::pow(p, 2.0),
                              3.0);  // = (r-p^2)^3

    return std::cbrt(q + std::sqrt(std::pow(q, 2.0) + s)) +
           std::cbrt(q - std::sqrt(std::pow(q, 2.0) + s)) + p;
  }

  static double findRootStraight(const CubicPolynomial& root_polynomial) {
    if (std::abs(root_polynomial.getCoefficients()[1]) < 1e-5) {
      return 0.0;
    }
    return -root_polynomial.getCoefficients()[0] / root_polynomial.getCoefficients()[1];
  }

  static double calculateVehicleFootArcLength(const Eigen::Vector2d& point,
                                              const ArcLengthParameterizedPathPoints& points,
                                              PathPoints::difference_type number_of_points_for_local_fit) {
    using namespace common;

    // find closest point on path
    const auto closest_point = common::min_score(
        points, [point](const auto& p) { return (p.position - point).norm(); });

    // approximate path at closest_point with polynomial and find root as better
    // approximation for foot point
    const QuadraticPolynomial regression_polynomial = getLocalPolynomialFit<2>(
        closest_point, *closest_point, points, number_of_points_for_local_fit);

    // the closest point (x,f(x)) on a polynomial of second degree f(x) to a
    // given point (x0,y0) is implicitly given by (x-x0) + f'(x)(f(x)-y0) = 0
    // (the root of a third degree polynomial)
    const CubicPolynomial root_polynomial =
        (1.0_x - ConstantPolynomial(point.x())) +
        regression_polynomial.derivate() *
            (regression_polynomial - ConstantPolynomial(point.y()));

    const double root = std::abs(root_polynomial.getCoefficients()[3]) > 1e-5
                            ? findRootCardano(root_polynomial)
                            : findRootStraight(root_polynomial);
    if (!std::isfinite(root)) {
      ROS_WARN_THROTTLE(4, "root not finite");
      return closest_point->arc_length;
    }

    const Eigen::Vector2d polynomial_regression_foot_point(
        root, regression_polynomial(root));

    // extrapolation if root isn't on path
    if (root < points.front().position.x()) {
      return points.front().arc_length -
             (points.front().position - polynomial_regression_foot_point).norm();
    } else if (root > points.back().position.x()) {
      return points.back().arc_length +
             (points.back().position - polynomial_regression_foot_point).norm();
    }


    // find closest point to previously calculated polynomial regression foot
    // point that is on the path polyline to be able to calculate the arc length
    // along the polyline segments
    constexpr ArcLengthParameterizedPathPoints::difference_type sampling_range = 10;
    const auto start_it = std::distance(points.begin(), closest_point) < sampling_range
                              ? points.begin()
                              : closest_point - sampling_range;
    const auto end_it = std::distance(closest_point, points.end()) < sampling_range
                            ? points.end()
                            : closest_point + sampling_range;

    // two possible cases
    // case 1: closest point is on a line segment
    double min_distance_to_line_segment = std::numeric_limits<double>::infinity();
    double arc_length_line_segment = 0.0;
    for (auto it = start_it; it != end_it - 1; it++) {
      const Eigen::Vector2d line_segment_vector = (it + 1)->position - it->position;
      const Eigen::Vector2d vector_to_point = polynomial_regression_foot_point - it->position;
      const double projection =
          line_segment_vector.dot(vector_to_point) / line_segment_vector.norm();
      const bool point_is_on_line_segment =
          0.0 < projection && projection < line_segment_vector.norm();

      if (point_is_on_line_segment) {
        const Eigen::Vector2d n = ((it + 1)->position - it->position).unitOrthogonal();
        const double d = n.dot(it->position);
        const double distance_to_line =
            std::fabs(n.dot(polynomial_regression_foot_point) - d);

        if (distance_to_line < min_distance_to_line_segment) {
          min_distance_to_line_segment = distance_to_line;
          arc_length_line_segment = it->arc_length + projection;
        }
      }
    }

    // case 2: closest point is one of the path points
    auto min_distance_point_iterator = common::min_score(
        start_it,
        end_it,
        [&polynomial_regression_foot_point](const auto& a) {
          return (a.position - polynomial_regression_foot_point).norm();
        });

    if ((min_distance_point_iterator->position - polynomial_regression_foot_point)
            .norm() < min_distance_to_line_segment) {
      return min_distance_point_iterator->arc_length;
    } else {
      return arc_length_line_segment;
    }
  }

  PathPoints::difference_type number_of_points_for_local_fit;
};
}

#endif  // PATH_H
