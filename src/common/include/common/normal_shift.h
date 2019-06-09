#ifndef COMMON_NORMALSHIFT_H
#define COMMON_NORMALSHIFT_H

#include "common/discretisize.h"
#include "common/eigen_utils.h"
#include "common/polynomial.h"
#include "common/polynomial_utils.h"
#include "common/types.h"

namespace common {
/*!
 * \namespace common::normal_shift
 * \brief normal_shift contains functionality to perform a normal_shift.
 */

namespace normal_shift {
/*!
 * \brief normalShift implements a point wise normal shift.
 *
 * 'point wise' means, that the shift direction is obtained based on the vector
 * between two of the input points.
 *
 * \param in_points the points to be shifted_point.
 * \param distances vector containing the difference for the i_th point to be
 * shifted.
 * \param out_points the destination to put the shifted points in.
 * \param min_point_dist the minimal distance between two points to consider the
 * to do the shift. \note distances must have the same size than in_points
 */
template <class VectorXdVector>
inline void normalShift(const VectorXdVector& in_points,
                        const std::vector<double>& distances,
                        VectorXdVector* out_points,
                        const double min_point_dist = 0.0) {
  assert(in_points.size() == distances.size() &&
         "Size of points to be shifted has to be the same than the related "
         "distances!");

  out_points->reserve(out_points->size() + in_points.size());
  using V = typename VectorXdVector::value_type;
  using S = typename V::Scalar;
  for (std::size_t i = 1; i < in_points.size(); i++) {
    const Eigen::Matrix<S, 2, 1> diff = to2D(in_points[i]) - to2D(in_points[i - 1]);
    if (diff.norm() > min_point_dist) {
      out_points->push_back(in_points[i] + to<V>(diff.unitOrthogonal() * distances[i]));
    }
  }
}



/*!
 * \brief normalShift implements a point wise normal shift.
 *
 * 'point wise' means, that the shift direction is obtained based on the vector
 * between two of the input points.
 *
 * \param in_points the points to be shifted_point.
 * \param dist the distance to shift the points.
 * \param out_points the destination to put the shifted points in.
 * \param min_point_dist the minimal distance between two points to consider the
 * to do the shift.
 */
template <class VectorXdVector>
inline void normalShift(const VectorXdVector& in_points,
                        const double dist,
                        VectorXdVector* out_points,
                        const double min_point_dist = 0.0) {

  out_points->reserve(out_points->size() + in_points.size());
  using V = typename VectorXdVector::value_type;
  using S = typename V::Scalar;
  for (std::size_t i = 1; i < in_points.size(); i++) {
    const Eigen::Matrix<S, 2, 1> diff = to2D(in_points[i]) - to2D(in_points[i - 1]);
    if (diff.norm() > min_point_dist) {
      out_points->push_back(in_points[i] + to<V>(diff.unitOrthogonal() * dist));
    }
  }
}

/*!
 * \brief normalShift implements a function based normalShift.
 *
 * 'function based' means, that the shift direction is obtained based on the
 * first derivative of the function.
 *
 * \param polynom the function to shift from.
 * \param dist the distance to shift the points.
 * \param params the DiscretizationParams for discretisize the polynom.
 * \param points the destination to put the shifted points in.
 * \pre all DiscretizationParams needs to be finite.
 * \pre if params.min != param.max then params.max >= params.min and step > 0.
 * \post points.size() == (params.max - params.min) / params.step.
 *
 * \b Note: default constructed DiscretizationParams will lead to a noop.
 */
template <class VectorXdVector, PolynomialDegree DEGREE, typename T>
inline void normalShift(const Polynomial<DEGREE, T>& polynom,
                        const T dist,
                        const DiscretizationParams params,
                        VectorXdVector* points) {

  assert(std::isfinite(params.step) && std::isfinite(params.min) &&
         std::isfinite(params.max));

  if (params.min == params.max) {
    return;
  }

  assert(params.max >= params.min);
  assert(params.step > 0);
  points->reserve(points->size() +
                  static_cast<std::size_t>((params.max - params.min) / params.step));

  using V = typename VectorXdVector::value_type;
  const auto derived_polynomial = polynom.derivate();
  for (auto x = params.min; x < params.max; x += params.step) {
    const Eigen::Matrix<T, 2, 1> diff{1.0, derived_polynomial.evaluate(x)};
    points->push_back(to<V>(common::point(polynom, x) + diff.unitOrthogonal() * dist));
  }
}

/*!
 * \brief normalShift implements a function based normalShift.
 *
 * 'function based' means, that the shift direction is obtained based on the
 * first derivative of the function.
 *
 * \param polynom the function to shift from.
 * \param distances vector containing the difference for the i_th point to be.
 * \param params the DiscretizationParams for discretisize the polynom.
 * \param points the destination to put the shifted points in.
 * \pre all DiscretizationParams needs to be finite.
 * \pre if params.min != param.max then params.max >= params.min and step > 0.
 *
 * \b Note: default constructed DiscretizationParams will lead to a noop.
 */
template <class VectorXdVector, PolynomialDegree DEGREE, typename T>
inline void normalShift(const Polynomial<DEGREE, T>& polynom,
                        const std::vector<T>& distances,
                        const DiscretizationParams params,
                        VectorXdVector* points) {

  assert(std::isfinite(params.step) && std::isfinite(params.min) &&
         std::isfinite(params.max));

  if (params.min == params.max) {
    return;
  }

  assert(params.max >= params.min);
  assert(params.step > 0);
  points->reserve(points->size() +
                  static_cast<std::size_t>((params.max - params.min) / params.step));

  using V = typename VectorXdVector::value_type;
  const auto derived_polynomial = polynom.derivate();
  int i = 0;
  for (auto x = params.min; x < params.max; x += params.step, i++) {
    const Eigen::Matrix<T, 2, 1> diff{1.0, derived_polynomial.evaluate(x)};
    points->push_back(
        to<V>(common::point(polynom, x) + diff.unitOrthogonal() * distances[i]));
  }
}
}  // namespace normal_shift
using namespace normal_shift;
}  // namespace common

#endif  // COMMON_NORMALSHIFT_H
