#ifndef COMMON_INDIRECT_SORT_H
#define COMMON_INDIRECT_SORT_H

#include <vector>
#include <iterator>
#include <functional>
#include <boost/range/algorithm/sort.hpp>
#include <boost/range/algorithm/transform.hpp>
#include <boost/range/algorithm_ext/iota.hpp>
#include "common/functors.h"

namespace common {
namespace algorithm {
/*!
 * \brief index list, defining how to sort range
 * \param range unsorted range
 * \param score_func , score score_func applied on range and stored in score
 * \param pred compare function applied to score
 * \return index list
 */
template <typename Range, typename ScoreFunc, typename Pred>
std::vector<size_t> indirect_sort(const Range& range,
                                  ScoreFunc score_func,
                                  std::vector<double>& score,
                                  Pred pred) {
  score.clear();
  score.reserve(range.size());

  boost::transform(range, std::back_inserter(score), score_func);

  std::vector<size_t> indices(score.size());
  boost::iota(indices, 0);  // 0,1,2,...

  // sort index list
  boost::sort(indices,
              accordingTo([&score](const auto& idx) { return score[idx]; }, pred));

  return indices;
}

template <typename Range, typename ScoreFunc>
std::vector<size_t> indirect_sort(const Range& range,
                                  ScoreFunc score_func,
                                  std::vector<double>& score) {
  return indirect_sort(range, score_func, score, std::less<double>());
}

}  // namespace algorithm;
using namespace algorithm;
}  // namespace common;

#endif  // COMMON_INDIRECT_SORT_H
