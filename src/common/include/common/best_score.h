#ifndef COMMON_BEST_SCORE_H
#define COMMON_BEST_SCORE_H
#include <iterator>
#include <functional>
#include "common/type_traits.h"

namespace common {
// clang-format off
/*!
 * \namespace common::algorithm
 * \brief algorithm contains algorithm which are not part of other libraries.
 *   - <a href="http://en.cppreference.com/w/cpp/algorithm">The Standard Algorithm Library</a>
 *   - <a href="http://en.cppreference.com/w/cpp/numeric"> The Standard Numeric Library</a>
 *   - <a href="https://www.boost.org/doc/libs/1_67_0/libs/algorithm/doc/html/index.html">Boost.Algorithm</a>
 *   - <a href="https://www.boost.org/doc/libs/1_67_0/libs/range/doc/html/index.html">Boost.Range</a>
 */
// clang-format on
namespace algorithm {
/*!
 * \brief best_score finds the element in [first, end) which scores best
 * according to a comparison function and a scoring function.
 *
 * This function minimizes the invokations of the scoring function without
 * allocation additional memory.
 *
 * \param first the iterator to the first element in a range.
 * \param end the iterator to the end of a range.
 * \param scoringFkt the scoring function.
 * \param comp the predicate to compare the scores.
 * \return an iterator to the best scoring element in [first, end) according to
 * comp and scoringFkt.
*/
template <typename IT, typename ScoringFkt, typename Comp>
inline IT best_score(IT first, IT end, ScoringFkt scoringFkt, Comp comp) {
  if (first == end) {
    return end;
  }
  auto best_score = scoringFkt(*first);
  IT best = first;
  ++first;

  for (; first != end; ++first) {
    auto current_score = scoringFkt(*first);
    if (comp(current_score, best_score)) {
      best_score = current_score;
      best = first;
    }
  }
  return best;
}

/*!
 * \brief min_score finds the element in [first, end) with minimal score
 * according to a scoring function.
 *
 * This function minimizes the invokations of the scoring functions without
 * allocation additional memory.
 *
 * \param first the iterator to the first element in a range.
 * \param end the iterator to the end of a range.
 * \param scoringFkt the scoring function.
 * \return an iterator to the minimal scoring element in [first, end) according
 * to comp and scoringFkt.
*/
template <typename IT, typename ScoringFkt>
inline IT min_score(IT first, IT end, ScoringFkt scoringFkt) {
  return best_score(first, end, scoringFkt, std::less<decltype(scoringFkt(*first))>());
}

/*!
 * \brief min_score finds the element in range with minimal score according to a
 * scoring function.
 *
 * This function minimizes the invokations of the scoring functions without
 * allocation additional memory.
 *
 * \param range the range.
 * \param scoringFkt the scoring function.
 * \return an iterator to the minimal scoring element in range according to comp
 * and scoringFkt.
 */
template <typename Range, typename ScoringFkt>
inline auto min_score(const Range& range, ScoringFkt scoringFkt) {
  STATIC_ASSERT_RANGE(Range);
  return min_score(std::begin(range), std::end(range), scoringFkt);
}

/*!
 * \brief min_score finds the element in range with minimal score according to a
 * scoring function.
 *
 *  This function minimizes the invokations of the scoring functions without
 * allocation additional memory.
 *
 * \param range the range.
 * \param scoringFkt the scoring function.
 * \return an iterator to the minimal scoring element in range according to comp
 * and scoringFkt.
*/
template <typename Range, typename ScoringFkt>
inline auto min_score(Range& range, ScoringFkt scoringFkt) {
  STATIC_ASSERT_RANGE(Range);
  return min_score(std::begin(range), std::end(range), scoringFkt);
}

/*!
 * \brief max_score finds the element in [first, end) with maximal score
 * according to a scoring function.
 *
 * This function minimizes the invokations of the scoring functions without
 * allocation additional memory.
 *
 * \param first the iterator to the first element in a range.
 * \param end the iterator to the end of a range.
 * \param scoringFkt the scoring function.
 * \return an iterator to the maximal scoring element in [first, end) according
 * to comp and scoringFkt.
*/
template <typename IT, typename ScoringFkt>
inline IT max_score(IT first, IT end, ScoringFkt scoringFkt) {
  return best_score(first, end, scoringFkt, std::greater<decltype(scoringFkt(*first))>());
}

/*!
 * \brief max_score finds the element in range with maximal score according to a
 * scoring function.
 *
 * This function minimizes the invokations of the scoring functions without
 * allocation additional memory. max_score() will give the same result as
 * \code
 * auto it = boost::max_element(range |
 *   boost::adaptors::transformed(scoringFkt)).base();
 * \endcode
 * Node the scoring function will be called twice as much here!
 *
 * \param range the range.
 * \param scoringFkt the scoring function.
 * \return an iterator to the maximal scoring element in range according to comp
 * and scoringFkt.
*/
template <typename Range, typename ScoringFkt>
inline auto max_score(const Range& range, ScoringFkt scoringFkt) {
  STATIC_ASSERT_RANGE(Range);
  return max_score(std::begin(range), std::end(range), scoringFkt);
}

/*!
 * \brief max_score finds the element in range with maximal score according to a
 * scoring function.
 *
 * This function minimizes the invokations of the scoring functions without
 * allocation additional memory.
 *
 * \param range the range.
 * \param scoringFkt the scoring function.
 * \return an iterator to the maximal scoring element in range according to comp
 * and scoringFkt.
*/
template <typename Range, typename ScoringFkt>
inline auto max_score(Range& range, ScoringFkt scoringFkt) {
  STATIC_ASSERT_RANGE(Range);
  return max_score(std::begin(range), std::end(range), scoringFkt);
}

/*!
 * \brief minmax_score finds the two elements in [first, end) with
 * maximal/minimal score according to a scoring function.
 *
 * This function minimizes the invokations of the scoring functions without
 * allocation additional memory.
 *
 * The result is the same as
 * \code {min_score(first, end, scoringFkt), max_score(first, end, scoringFkt)};
 * \endcode
 * but this function traverses the range only once.
 *
 * \param first the iterator to the first element in a range.
 * \param end the iterator to the end of a range.
 * \param scoringFkt the scoring function.
 * \return a pair of iterators to the minimal/maximal scoring element in [first,
 * end) according scoringFkt.
*/
template <typename IT, typename ScoringFkt>
inline std::pair<IT, IT> minmax_score(IT first, IT end, ScoringFkt scoringFkt) {
  if (first == end) {
    return {first, first};
  }
  auto max_score = scoringFkt(*first);
  auto min_score = max_score;
  IT max = first;
  IT min = max;
  ++first;

  for (; first != end; ++first) {
    auto current_score = scoringFkt(*first);
    if (current_score > max_score) {
      max_score = current_score;
      max = first;
    }
    if (current_score < min_score) {
      min_score = current_score;
      min = first;
    }
  }
  return {min, max};
}

/*!
 * \brief minmax_score finds the two elements in range with
 * maximal/minimal score according to a scoring function.
 *
 * This function minimizes the invokations of the scoring functions without
 * allocation additional memory.
 *
 * The result is the same as
 * \code {min_score(range, scoringFkt), max_score(range, scoringFkt)}; \endcode
 * but this function traverses the range only once.
 *
 * \param range the range.
 * \param scoringFkt the scoring function.
 * \return a pair of iterators to the minimal/maximal scoring element in range
 * according scoringFkt.
*/
template <typename Range, typename ScoringFkt>
inline auto minmax_score(const Range& range, ScoringFkt scoringFkt) {
  STATIC_ASSERT_RANGE(Range);
  return minmax_score(std::begin(range), std::end(range), scoringFkt);
}

}  // namespace algorithm;
using namespace algorithm;
}  // namespace common;

#endif  // COMMON_BEST_SCORE_H
