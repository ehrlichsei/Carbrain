#ifndef COMMON_UNIQUE_ERASE_H
#define COMMON_UNIQUE_ERASE_H

#include <algorithm>
#include "common/type_traits.h"

namespace common {
namespace algorithm {

/*!
 * \brief unique_erase eliminates all but the first element from every
 * consecutive group of elements from the range, which are equivalent according to
 * the given predicate.
 *
 * \param range the range
 * \param p the predicate
 */
template <typename Range, typename Pred>
void unique_erase(Range& range, Pred p) {
  STATIC_ASSERT_RANGE(Range);
  range.erase(std::unique(range.begin(), range.end(), p), range.end());
}

/*!
 * \brief unique_erase eliminates all but the first element from every
 * consecutive group of equivalent elements from the range.
 *
 * \param range the range
 */
template <typename Range>
void unique_erase(Range& range) {
  STATIC_ASSERT_RANGE(Range);
  range.erase(std::unique(range.begin(), range.end()), range.end());
}

}  // namespace algorithm;
using namespace algorithm;
}  // namespace common;

#endif  // COMMON_UNIQUE_ERASE_H
