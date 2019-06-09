#pragma once

#include <type_traits>
#include <cmath>

namespace common {
/*!
 * \namespace common::math
 * \brief math contains some useful mathematical utilities which are not part of
 * the standard-library or boost.
 */
namespace math {
template <typename T>
/*!
 * \brief sgn returns the sign of a given value.
 * \param val value to get the sign of.
 * \return -1 if val < 0, 1 if val > 0, 0 if val == 0.
 */
int sgn(T val) {
  static_assert(std::is_arithmetic<T>(), "");
  return (T(0) < val) - (val < T(0));
}

/*!
 * \brief squared squares the given value.
 * \param x the value to square.
 * \return the square of x.
 */
template <typename T>
auto squared(const T& x) {
  return x * x;
}

}  // namespace math;

namespace math_detail {

template <typename T1, typename T2>
auto positiveModulo(T1 x, T2 y, std::true_type, std::false_type) {
  static_assert(std::is_signed<T1>::value && std::is_signed<T2>::value,
                "you passed an unsigned value to positiveModulo. "
                "positiveModulo only works with signed integer or floating "
                "point parameters");
  // see
  // https://stackoverflow.com/questions/14997165/fastest-way-to-get-a-positive-modulo-in-c-c
  return (x % y + y) % y;
}

template <typename T1, typename T2>
auto positiveModulo(T1 x, T2 y, std::false_type, std::true_type) {
  // see https://stackoverflow.com/questions/11980292/how-to-wrap-around-a-range
  return x - std::floor(x / y) * y;
}

}  // namespace math_detail

namespace math {

/*!
 * \brief positiveModulo performs a modulo operation which is defined for
 * both integer and floating point types. The definition is not the same as the
 * definition of the %operator, the result is the same for positive integer
 * operands,
 * but different for negative operands (% is not defined for floating point
 * operands) (positiveModulo result has same sign as the divisor, % result has
 * same sign as dividend).
 * see https://en.wikipedia.org/wiki/Modulo_operation
 * \param x the dividend.
 * \param y the divisor.
 * \return x modulo y
 */
template <typename T1, typename T2>
auto positiveModulo(T1 x, T2 y) {
  return math_detail::positiveModulo(
      x,
      y,
      typename std::integral_constant < bool,
      std::is_integral<T1>::value&& std::is_integral<T2>::value > (),
      typename std::integral_constant < bool,
      std::is_floating_point<T1>::value || std::is_floating_point<T2>::value > ());
}

/*!
 * \brief wraps the given angle into the range [-Pi, Pi).
 * \param angle the angle to wrap.
 * \return the wraped angle.
 */
template <typename T>
T wrapAngleMinusPiToPi(T angle) {
  static_assert(
      std::is_floating_point<T>::value,
      "you can only wrap floating point angles (float, double, long double)");
  return positiveModulo(angle + M_PI, 2 * M_PI) - M_PI;
}

/*!
 * \brief wraps the given angle into the range [0, 2*Pi).
 * \param angle the angle to wrap.
 * \return the wraped angle.
 */
template <typename T>
T wrapAngleZeroToTwoPi(T angle) {
  static_assert(
      std::is_floating_point<T>::value,
      "you can only wrap floating point angles (float, double, long double)");
  return positiveModulo(angle, 2 * M_PI);
}

}  // namespace math

using namespace math;

}  // namespace common
