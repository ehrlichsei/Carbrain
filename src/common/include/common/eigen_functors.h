#ifndef COMMON_EIGEN_FUNCTORS_H
#define COMMON_EIGEN_FUNCTORS_H
#include <common/macros.h>

#include <functional>
THIRD_PARTY_HEADERS_BEGIN
#include <Eigen/Dense>
THIRD_PARTY_HEADERS_END

#include "common/functors.h"

namespace common {
namespace functors {
/*!
 * \brief areClose returns a functions which returns whether its arguments are
 * closer togehter (according to the euclidean distance) as d.
 */
inline auto areClose(const double d) {
  return [d](auto& a, auto& b) { return (a - b).norm() < d; };
}

/*!
 * \brief distanceTo returns a function which returns the euclidean distance
 * between its argument and v.
 */
template <typename Vector>
auto distanceTo(const Vector& v) {
  return [v](const auto& p) { return (v - p).norm(); };
}
}  // namespace functors

namespace functors_detail {
template <typename T, int Size>
struct vector_trait<Eigen::Matrix<T, Size, 1>> {
  const auto& x(const Eigen::Matrix<T, Size, 1>& p) const { return p.x(); }
  const auto& y(const Eigen::Matrix<T, Size, 1>& p) const { return p.y(); }
  auto& x(Eigen::Matrix<T, Size, 1>& p) const { return p.x(); }
  auto& y(Eigen::Matrix<T, Size, 1>& p) const { return p.y(); }
  auto x(Eigen::Matrix<T, Size, 1>&& p) const { return p.x(); }
  auto y(Eigen::Matrix<T, Size, 1>&& p) const { return p.y(); }
};

template <typename T>
struct eigen_transform {
  T t;
  template <class S>
  auto operator()(const S& s) const {
    return t * s;
  }
};
}  // namespace functors_detail

namespace functors {
/*!
 * \brief eigen_transformer creates a functor wich apply the given trasformation
 * to its argument.
 * \param t the transformation.
 * \return a function which applies t to its argument and returns the result.
 */
template <class T>
auto eigen_transformer(const T& t) {
  return functors_detail::eigen_transform<T>{t};
}
}  // namespace functors
using namespace functors;
}  // namespace common

#endif  // COMMON_EIGEN_FUNCTORS_H
