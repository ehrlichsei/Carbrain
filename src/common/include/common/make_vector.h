#ifndef COMMON_MAKE_VECTOR_H
#define COMMON_MAKE_VECTOR_H
#include <common/type_traits.h>
#include <common/types.h>

namespace common {
namespace range {

/*!
 * \brief make_vector creates a vector from a given range.
 * \param range the range to put it's elements into the vector.
 * \return a std::vector containing the elements of range.
 */
template <typename T = void, class Range>
inline auto make_vector(const Range &range) {
  STATIC_ASSERT_RANGE(Range);
  using TT = common::type_or<T, common::range_value_t<Range>>;
  return std::vector<TT>(std::begin(range), std::end(range));
}


/*!
 * \brief make_eigen_vector creates a vector from a given range.
 * In contrast to make_vector make_eigen_vector creates a std::vector
 * with an Eigen::alligned_allocator.
 * \param range the range to put it's elements into the vector.
 * \return a std::vector containing the elements of range.
 */
template <typename T = void, class Range>
inline auto make_eigen_vector(const Range &range) {
  STATIC_ASSERT_RANGE(Range);
  using TT = common::type_or<T, common::range_value_t<Range>>;
  return common::EigenAlignedVector<TT>(std::begin(range), std::end(range));
}

}  // namespace range
using namespace range;
}  // namespace common
#endif  // COMMON_MAKE_VECTOR_H
