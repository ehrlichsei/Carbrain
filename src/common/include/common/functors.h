#ifndef COMMON_FUNCTORS_H
#define COMMON_FUNCTORS_H

#include <functional>

namespace common {

namespace functors_detail {
template <typename T>
struct x_value_getter;

// the fallback implementation
template <typename T>
struct vector_trait {
  const auto& x(const T& p) const { return p.x; }
  const auto& y(const T& p) const { return p.y; }
  auto& x(T& p) const { return p.x; }
  auto& y(T& p) const { return p.y; }
  auto x(T&& p) const { return p.x; }
  auto y(T&& p) const { return p.y; }
};
}  // namespace functors_detail

/*!
 * \namespace common::functors
 * \brief functors contains functors which are helpful in combination with
 * STL/boost algorithms.
 */
namespace functors {
/*!
 * \brief plus_constant returns  a function, which adds a constant.
 * \param c the constant to add.
 * \return a function which returns the sum of c and its argument.
 */
template <typename T>
auto plus_constant(const T& c) {
  return [c](const auto& x) { return x + c; };
}

/*!
 * \brief get_x_value returns a const reference to the x-value of the given
 * type, which is likely to be a vector.
 * \param p the object ot get its x-value from.
 * \return const reference to the x-value of p.
 */
template <typename T>
decltype(auto) get_x_value(T&& p) {
  return functors_detail::vector_trait<std::remove_const_t<std::remove_reference_t<T>>>().x(
      std::forward<T>(p));
}

/*!
 * \brief get_y_value returns a const reference to the y-value of the given
 * type, which is likely to be a vector.
 * \param p the object ot get its y-value from.
 * \return const reference to the y-value of p.
 */
template <typename T>
decltype(auto) get_y_value(T&& p) {
  return functors_detail::vector_trait<std::remove_const_t<std::remove_reference_t<T>>>().y(
      std::forward<T>(p));
}

/*!
 * \brief x_value returns a function which returns the x-value of its argument.
 */
struct x_value {
  template <class T>
  decltype(auto) operator()(T&& p) const {
    return get_x_value(std::forward<T>(p));
  }
};

/*!
 * \brief y_value returns a function which returns the y-value of its argument.
 */
struct y_value {
  template <class T>
  decltype(auto) operator()(T&& p) const {
    return get_y_value(std::forward<T>(p));
  }
};

/*!
 * \brief addordinTo creates a functor which compares it's two arguments
 * according
 * to the given scoring function.
 * \param s the scoring funcktion.
 * \param c the comparison function.
 * \return a functor which compares it's two arguments accordin to the given
 * scoring function.
 */
template <class ScoringFct, class Comp = std::less<>>
inline auto accordingTo(ScoringFct&& s, Comp&& c = std::less<>()) {
  return [ s = std::forward<ScoringFct>(s), c = std::forward<Comp>(c) ](
      const auto& a, const auto& b) {
    return c(s(a), s(b));
  };
}

/*!
 * \brief less_x returns a function wich compares the x-values of two given
 * objects.
 */
inline auto less_x() { return accordingTo(x_value{}); }

/*!
 * \brief less_y returns a function wich compares the y-values of two given
 * objects.
 */
inline auto less_y() { return accordingTo(y_value{}); }

}  // namespace functors;
using namespace functors;
}  // namespace common;

#endif  // COMMON_FUNCTORS_H
