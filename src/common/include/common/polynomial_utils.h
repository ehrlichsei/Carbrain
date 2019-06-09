#ifndef POLYNOMIAL_UTILS
#define POLYNOMIAL_UTILS

#include <common/macros.h>

THIRD_PARTY_HEADERS_BEGIN
#include <Eigen/Core>
#include <Eigen/Geometry>
THIRD_PARTY_HEADERS_END

#include <common/polynomial.h>

namespace common {
namespace polynomial {
//!
//! \brief normalAngle calculates the angle from polynomial base frame
//! (e.g. vehicle) to the tangent vector
//! \param polynomial the polynomial to get the tangent angle on.
//! \param x the x vale to get the tangent angle on polynomial at.
//! \return angle in radians
//!
template <PolynomialDegree DEGREE, typename T>
inline T tangentAngle(const Polynomial<DEGREE, T>& polynomial, T x) {
  return std::atan(polynomial.derivate().evaluate(x));
}

//!
//! \brief tangent calculates the tangent vector on the polynomial at x.
//! \param polynomial the polynomial to get the tangent of.
//! \param x the x value to geht the tangent of.
//! \return tangent vector with unit length
//!
template <PolynomialDegree DEGREE, typename T>
inline Eigen::Matrix<T, 2, 1> tangent(const Polynomial<DEGREE, T>& polynomial, T x) {
  const T alpha = tangentAngle(polynomial, x);
  return Eigen::Rotation2D<T>(alpha) * Eigen::Matrix<T, 2, 1>::UnitX();
}


//!
//! \brief normalAngle calculates the angle from polynomial base frame
//! (e.g. vehicle) to the normal vector
//! \param polynomial the polynomial to get the normal angle of.
//! \param x the x-value to get the normal angle of.
//! \return angle in radians
//!
template <PolynomialDegree DEGREE, typename T>
inline T normalAngle(const Polynomial<DEGREE, T>& polynomial, T x) {
  return tangentAngle(polynomial, x) + M_PI_2;
}

//!
//! \brief normal calculates the normal vector on the polynomial at x.
//! \param polynomial the polynomial.
//! \param x the x value to get the normal of.
//! \return the normal vector.
//!
template <PolynomialDegree DEGREE, typename T>
inline Eigen::Matrix<T, 2, 1> normal(const Polynomial<DEGREE, T>& polynomial, T x) {
  return tangent(polynomial, x).unitOrthogonal();
}

//!
//! \brief point returns the point on the polynomial at x.
//! \param polynomial the polynomial.
//! \param x the x-value.
//! \return the point on the polynomial at x.
//!
template <PolynomialDegree DEGREE, typename T>
inline Eigen::Matrix<T, 2, 1> point(const Polynomial<DEGREE, T>& polynomial, T x) {
  return {x, polynomial(x)};
}

}  // namespace polynomial
using namespace polynomial;
}  // namespace common

#endif  // POLYNOMIAL_UTILS
