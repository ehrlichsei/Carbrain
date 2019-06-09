#ifndef COMMON_ADAPTORS_H
#define COMMON_ADAPTORS_H
#include <functional>
#include <boost/range/adaptor/transformed.hpp>
#include "common/functors.h"
#include "common/type_traits.h"

namespace common {
// clang-format off
/*!
 * \namespace common::range
 * \brief range contains extensions for <a href="https://www.boost.org/doc/libs/1_66_0/libs/range/doc/html/index.html">boost.range</a>.
 */
// clang-format on
namespace range {

/*!
 * \brief make_transform_holder creates a boost::range_detail::transform_holder.
 * This makes creating new transformative range adaptors easier.
 */
template <typename T>
auto make_transform_holder(T&& transform) {
  return boost::range_detail::transform_holder<T>{std::forward<T>(transform)};
}

/*!
 * \brief members creats an range-adaptor which allpys the given
 * member-function.
 * \code
 * std::vector<std::vector<double>> xs;
 * auto n = boost::accumulate(xs | members(&std::vector<double>::size, 0);
 * \endcode
 */
template <class M, class T>
inline auto members(M T::*f) {
  return boost::adaptors::transformed(std::mem_fn(f));
}

namespace {
/*!
 * \brief x_values is a range adaptor which makes the x-values of the given
 * input-range accessable as a range.
 */
const auto x_values = make_transform_holder(x_value{});
/*!
 * \brief y_values is a range adaptor which makes the y-values of the given
 * input-range accessable as a range.
 */
const auto y_values = make_transform_holder(y_value{});
}

namespace range_detail {
template <typename T>
struct static_caster {
  template <typename S>
  auto operator()(const S& x) const {
    return static_cast<T>(x);
  }
};
}  // namespace range_detail

/*!
 * \brief static_casted is a range adaptor which casts the elements of the
 * input-range to the given type.
 */
template <typename T>
const auto static_casted = make_transform_holder(range_detail::static_caster<T>{});

}  // namespace range;
using namespace range;
}  // namespace common

#endif  // COMMON_ADAPTORS_H
