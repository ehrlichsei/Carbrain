#ifndef FOOT_FINDER_H
#define FOOT_FINDER_H
#include <common/macros.h>

THIRD_PARTY_HEADERS_BEGIN
#include <Eigen/Dense>
THIRD_PARTY_HEADERS_END

#include <common/polynomial.h>
#include <common/eigen_utils.h>

namespace utils {

/*!
 * \brief findLotfusspunkt
 *  finds the closest point on the trajectory polynom to point
 * \param polynomial the polynomial to find the perpendicular foot on.
 * \param point the point
 */
double findLotfusspunktX(const common::DynamicPolynomial& polynomial,
                         const Eigen::Vector2d& point,
                         const double epsilon = 0.01,
                         const unsigned int max_iterations = 20);

template <class VectorExpression>
auto findLotfusspunkt(const common::DynamicPolynomial& polynomial,
                      const VectorExpression& point,
                      const double epsilon = 0.01,
                      const unsigned int max_iterations = 20) {
  auto p = common::toVector(point);
  p.x() = findLotfusspunktX(polynomial, to2D(p), epsilon, max_iterations);
  p.y() = polynomial.evaluate(p.x());
  return p;
}

/*!
 * \brief findRoot
 *  finds a root of the given polynomial using newton's method stopping if a
 *maximal number of iterations is reached or x value changes less than
 *epsilon.
 *
 * \param polynom the polynom to find the root of
 * \param epsilon the
 * \param max_iterations maximal numbers of iterations
 * \param x_start the starting point
 * \return resulting x value
 */
double findRoot(const common::DynamicPolynomial& polynom,
                const double epsilon,
                const unsigned int max_iterations,
                const double x_start);

}  // namespace utils

#endif  // FOOT_FINDER_H
