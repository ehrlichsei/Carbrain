#ifndef COMMON_BASIC_STATISTICS_H
#define COMMON_BASIC_STATISTICS_H

#include <iterator>
#include <numeric>
#include <type_traits>
#include <boost/range/adaptor/transformed.hpp>

#include "common/type_traits.h"

namespace common {
namespace basic_statistics_detail {

template <typename T>
struct zero {
  static constexpr T get() {
    static_assert(
        std::is_arithmetic<T>::value && !std::is_same<T, bool>::value,
        "basic_statistics needs a specialization of the struct 'zero', if "
        "you are using non-arithmetical types. Did you forget to "
        "include basic_statistics_eigen.h?");
    return T(0);
  }
};

template <class X, class IT>
using iterator_working_type = type_or<X, typename std::iterator_traits<IT>::value_type>;

}  // namespace basic_statistics_detail

/*!
 * \namespace common::basic_statistics
 * \brief basic_statistic functionallity like sum, mean, squared sum and
 * variance.
 *
 * All of these functions are designed to work with the builtin
 * integral types. To work with other types like Eigen::Vector or cv::Point the
 * zero-trait needs to be specialized.
 *
 * \b Attention: currently variance can only be used with integral types.
 */
namespace basic_statistics {

// clang-format off
/*!
 * \brief sum summs up the elements in the range [begin, last).
 *
 * sum() follows the <a href="https://en.wikipedia.org/wiki/Empty_sum">emtpy sum convention</a>.
 *
 * \param begin an iterator which points to the first element of the range.
 * \param last an iterator which points after the last element.
 * \tparam X the type which shall used during summation. If not passed, the
 *         value types of the range is used.
 * \return the sum of the elements in the range.
 */
// clang-format on
template <typename X = void, typename IT>
basic_statistics_detail::iterator_working_type<X, IT> sum(IT begin, IT last) {
  using T = type_or<X, typename std::iterator_traits<IT>::value_type>;
  return std::accumulate(begin, last, basic_statistics_detail::zero<T>::get());
}

// clang-format off
/*!
 * \brief sum summs up the eelemnts in range.
 * \param range the range to sum up.
 *
 * sum() follows the <a href="https://en.wikipedia.org/wiki/Empty_sum">emtpy sum convention</a>.
 *
 * \tparam X the type which shall used during summation. If not passed, the
 *         value types of the range is used.
 * \return the sum of the elements in the range.
 */
// clang-format on
template <typename X = void, typename Range>
type_or<X, range_value_t<Range>> sum(const Range& range) {
  STATIC_ASSERT_RANGE(Range);
  using T = type_or<X, range_value_t<Range>>;
  return sum<T>(range.begin(), range.end());
}

/*!
 * \brief mean the mean of the range [begin, last).
 * \param begin an iterator which points to the first element of the range.
 * \param last an iterator which points after the last element.
 * \tparam X the type which shall used during mean calculation. If not passed,
 *         the value types of the range is used.
 * \pre the range need to contain at least one element.
 * \return the mean of the elements in the range.
 */
template <typename X = void, typename IT>
basic_statistics_detail::iterator_working_type<X, IT> mean(IT begin, IT last) {
  using T = basic_statistics_detail::iterator_working_type<X, IT>;
  const auto size = std::distance(begin, last);
  assert(size >= 1 && "mean is not defined for less than 1 element");
  return sum<T>(begin, last) / size;
}
/*!
 * \brief mean the mean of range.
 * \param range the range.
 * \tparam X the type which shall used during mean calculation. If not passed,
 *         the value types of the range is used.
 * \pre range need to contain at least one element.
 * \return the mean of the elements in the range.
 */
template <typename X = void, typename Range>
type_or<X, range_value_t<Range>> mean(const Range& range) {
  STATIC_ASSERT_RANGE(Range);
  using T = type_or<X, range_value_t<Range>>;
  return mean<T>(range.begin(), range.end());
}

/*!
 * \brief squaredSum the sum of the squares of the range [begin, last).
 * \param begin an iterator which points to the first element of the range.
 * \param last an iterator which points after the last element.
 * \tparam X the type which shall used during the squared summation. If not
 *         passed, the value types of the range is used.
 * \return the squared sum of the elements in the range.
 */
template <typename X = void, typename IT>
basic_statistics_detail::iterator_working_type<X, IT> squaredSum(IT begin, IT last) {
  using T = basic_statistics_detail::iterator_working_type<X, IT>;
  return sum<T>(boost::adaptors::transform(boost::make_iterator_range(begin, last),
                                           [](const auto& a) { return a * a; }));
}

/*!
 * \brief squaredSum the sum of the squares of range.
 * \param range the range.
 * \tparam X the type which shall used during the squared summation. If not
 *         passed, the value types of the range is used.
 * \return the squared sum of the elements in the range.
 */
template <typename X = void, typename Range>
type_or<X, range_value_t<Range>> squaredSum(const Range& range) {
  STATIC_ASSERT_RANGE(Range);
  using T = type_or<X, range_value_t<Range>>;
  return squaredSum<T>(range.begin(), range.end());
}

/*!
 * \brief variance the sum of the variance of the range [begin, last).
 * This function implements the
 * <a href="https://en.wikipedia.org/wiki/Algebraic_formula_for_the_variance">
 * algebraic formula for the variance.</a>.
 * \param begin an iterator which points to the first element of the range.
 * \param last an iterator which points after the last element.
 * \tparam X the type which shall used during the variance calculation . If not
 *         passed, the value types of the range is used.
 * \pre the range needs to contain at least two elements.
 * \return the variance of the elements in the range.
 */
template <typename X = void, typename IT, typename IT2>
basic_statistics_detail::iterator_working_type<X, IT> variance(
    IT begin, IT2 last, basic_statistics_detail::iterator_working_type<X, IT> mean) {
  using T = basic_statistics_detail::iterator_working_type<X, IT>;
  const auto n = std::distance(begin, last);
  assert(n >= 2 && "variance is not defined for less than 2 elements");
  return (squaredSum<T>(begin, last) - n * mean * mean) / (n - 1);
}

/*!
 * \brief variance the sum of the variance of range.
 * This function implements the
 * <a href="https://en.wikipedia.org/wiki/Algebraic_formula_for_the_variance">
 * algebraic formula for the variance.</a>.
 * \param range the range.
 * \tparam X the type which shall used during the variance calculation . If not
 *         passed, the value types of the range is used.
 * \pre range needs to contain at least two elements.
 * \return the variance of the elements in the range.
 */
template <typename X = void, typename Range>
type_or<X, range_value_t<Range>> variance(const Range& range,
                                          type_or<X, range_value_t<Range>> mean) {
  STATIC_ASSERT_RANGE(Range);
  using T = type_or<X, range_value_t<Range>>;
  return variance<T>(range.begin(), range.end(), mean);
}

}  // namespace basic_statistics

using namespace basic_statistics;

}  // namespace common

#endif  // COMMON_BASIC_STATISTICS_H
