#ifndef COMMON_JOIN_H
#define COMMON_JOIN_H

#include <boost/range/join.hpp>

namespace common {
namespace range {

// clang-format off
/*!
 * \brief join is an warpper around
 * <a href="https://www.boost.org/doc/libs/1_66_0/libs/range/doc/html/range/reference/utilities/join.html">boost::join</a>.
 * The basic advantage is, that join() can deal with an arbitrary amount of parameters.
 */
// clang-format on
template <class C>
auto join(C &&c) {
  return boost::make_iterator_range(c);
}

// clang-format off
/*!
 * \brief join is an warpper around
 * <a href="https://www.boost.org/doc/libs/1_66_0/libs/range/doc/html/range/reference/utilities/join.html">boost::join</a>.
 * The basic advantage is, that join() can deal with an arbitrary amount of parameters.
 */
// clang-format on
template <class C, class... Args>
auto join(C &&c, Args &&... args) {
  return boost::join(boost::make_iterator_range(c), join(std::forward<Args>(args)...));
}

}  // namespace range;
using namespace range;
}  // namespace common

#endif  // COMMON_JOIN_H
