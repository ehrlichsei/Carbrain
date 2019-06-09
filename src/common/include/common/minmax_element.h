#ifndef COMMON_MINMAX_ELEMENT_H
#define COMMON_MINMAX_ELEMENT_H

#include <algorithm>
#include <iterator>

namespace common {
namespace algorithm {
// clang-format off
/*!
 * \brief minmax_element returns a pair of iterators in range which point to
 * minimal and the maximal element.
 *
 * This is just a thin range-warpper for
 * <a href="https://en.cppreference.com/w/cpp/algorithm/minmax_element">std::minmax_element</a>.
 *
 * \param range the range to find the minimum/maximum of.
 * \return the same as \code {boost::min_element(range), boost::max_element(range)} \endcode
 */
// clang-format on
template <class Range>
inline auto minmax_element(const Range& range) {
  return std::minmax_element(std::begin(range), std::end(range));
}

// clang-format off
/*!
 * \brief minmax_element returns a pair of iterators in range which point to
 * minimal and the maximal element.
 *
 * This is just a thin range-warpper for
 * <a href="https://en.cppreference.com/w/cpp/algorithm/minmax_element">std::minmax_element</a>.
 *
 * \param range the range to find the minimum/maximum of.
 * \return the same as \code {boost::min_element(range), boost::max_element(range)} \endcode
 */
// clang-format on
template <class Range>
inline auto minmax_element(Range& range) {
  return std::minmax_element(std::begin(range), std::end(range));
}

// clang-format off
/*!
 * \brief minmax_element returns a pair of iterators in range which point to
 * minimal and the maximal element according to a given predicate.
 *
 * This is just a thin range-warpper for 
 * <a href="https://en.cppreference.com/w/cpp/algorithm/minmax_element">std::minmax_element</a>.
 *
 * \param range the range to find the minimum/maximum of.
 * \param p the predicate to use for comparison.
 * \return the same as \code {boost::min_element(range, p), boost::max_element(range, p)} \endcode
 */
// clang-format on
template <class Range, class Pred>
inline auto minmax_element(const Range& range, Pred p) {
  return std::minmax_element(std::begin(range), std::end(range), p);
}

// clang-format off
/*!
 * \brief minmax_element returns a pair of iterators in range which point to
 * minimal and the maximal element according to a given predicate.
 *
 * This is just a thin range-wrapper for 
 * <a href="https://en.cppreference.com/w/cpp/algorithm/minmax_element">std::minmax_element</a>.
 *
 * \param range the range to find the minimum/maximum of.
 * \param p the predicate to use for comparison.
 * \return the same as \code {boost::min_element(range, p), boost::max_element(range, p)} \endcode
 */
// clang-format on
template <class Range, class Pred>
inline auto minmax_element(Range& range, Pred p) {
  return std::minmax_element(std::begin(range), std::end(range), p);
}

}  // namespace algorithm;
using namespace algorithm;
}  // namespace common;

#endif  // COMMON_MINMAX_ELEMENT_H
