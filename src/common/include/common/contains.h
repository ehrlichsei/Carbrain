#ifndef CONTAINS_H
#define CONTAINS_H

#include <iterator>
#include <algorithm>

namespace common {

namespace algorithm_detail {
template <class C, typename T>
inline bool contains(const C& container, const T& element, std::false_type) {
  return std::find(std::cbegin(container), std::cend(container), element) !=
         std::cend(container);
}

template <class C, typename T>
inline bool contains(const C& container, const T& element, std::true_type) {
  return container.find(element) != container.end();
}

template <typename C, typename Arg>
struct has_find {
  template <typename T>
  static constexpr auto check(
      T*) -> typename std::is_same<decltype(std::declval<T>().find(std::declval<Arg>())),
                                   decltype(std::declval<T>().end())>::type;  // attempt to call it and see if the return type is correct

  template <typename>
  static constexpr std::false_type check(...);

  typedef decltype(check<C>(nullptr)) type;
};
}  // namespace algorithm_detail;

namespace algorithm {

/*!
 * \brief contains returns whether the given container contains the given
 * element.
 *
 * contains() uses per default std::find, but if the contaier provides a member
 * function which is called find and returns an iterator, this function is used
 * instead.
 *
 * \param container the container to look in.
 * \param element the element to look for.
 * \return whether container contains element.
 */
template <class C, typename T>
inline bool contains(const C& container, const T& element) {
  return algorithm_detail::contains(
      container, element, typename algorithm_detail::has_find<C, const T&>::type());
}

}  // namespace algorithm;
using namespace algorithm;
}  // namespace common;

#endif  // CONTAINS_H
