#ifndef COMMON_MOVE_RANGE_H
#define COMMON_MOVE_RANGE_H
#include <boost/range/iterator_range.hpp>

namespace common {
namespace range {
// clang-format off
/*!
 * \brief move_range create a moving range around the given range. This is
 * usefull for moving noncopyable types like std::unique_ptr<T>.
 *
 * \b Attention: there are some ranges like
 * <a href="https://www.boost.org/doc/libs/1_66_0/libs/range/doc/html/range/reference/adaptors/reference/uniqued.html">boost::range::uniqued</a>,
 * which can not be move from. If you try moving from them, this will lead to
 * strange runtime errors. For some details look at <a href="https://github.com/boostorg/range/issues/63">this issue on github</a>.
 *
 * \param range the range to move from.
 * \return the moving range.
 */
// clang-format on
template <typename Range>
auto move_range(Range &&range) {
  return boost::make_iterator_range(std::make_move_iterator(std::begin(range)),
                                    std::make_move_iterator(std::end(range)));
}

}  // namespace range;
using namespace range;
}  // namespace common;

#endif  // COMMON_MOVE_RANGE_H
