#ifndef TYPE_TRAITS_H
#define TYPE_TRAITS_H
#include <type_traits>
#include <iterator>

namespace common {
/*!
 * \namespace common::type_traits
 * \brief type_traits contains custom useful type_traits.
 * Type traits are utilities to get informations about types
 * at compile time.
 */
namespace type_traits {
// replace this by std::void_t in C++17
template <class... Ts>
using void_t = void;

template <class T, class = void>
struct has_begin : std::false_type {};

template <class T>
struct has_begin<T, void_t<decltype(std::begin(std::declval<T&>()))>> : std::true_type {
};

template <class T, class = void>
struct has_end : std::false_type {};

template <class T>
struct has_end<T, void_t<decltype(std::end(std::declval<T&>()))>> : std::true_type {};

/*!
 * \brief is_range returns wheather a given type is a range (e.g. c-array,
 * std::vector, adaptors...).
 * \tparam T the type to test.
 * \return whether T is a range.
 */
template <class T>
constexpr bool is_range() {
  return has_end<T>() && has_begin<T>();
}

#define STATIC_ASSERT_RANGE(T) \
  static_assert(               \
      ::common::is_range<T>(), \
      "T needs to be a range (needs to have begin- and end-iterator")

/*!
 * \brief range_traits is a type trait class whcih provides a uniform interface
 * to properties of ranges. It is heavly inspired by std::iterator_traits.
 */
template <typename Range>
struct range_traits {
  /*!
   * \brief iterator_type the type of the (begin-) iterator of range.
   */
  using iterator_type = decltype(std::begin(std::declval<Range&>()));
  /*!
   * \brief value_type the type of the values carried by the range.
   */
  using value_type = typename std::iterator_traits<iterator_type>::value_type;
  /*!
   * \brief reference the return type of the dereference operation.
   */
  using reference = typename std::iterator_traits<iterator_type>::reference;
};

/*!
 * \brief range_iterator_t shortcut to the iterator_type of Range.
 */
template <typename Range>
using range_iterator_t = typename range_traits<Range>::iterator_type;

/*!
 * \brief range_value_t shortcut to the value_type of Range.
 */
template <typename Range>
using range_value_t = typename range_traits<Range>::value_type;

/*!
 * \brief range_reference_t shortcut to the reference type of Range.
 */
template <typename Range>
using range_reference_t = typename range_traits<Range>::reference;

/*!
 * \brief type_or evaluates to a default value if T is not given.
 *
 * type_or evaluates to T is T is not void and otherwise to Default.
 */
template <typename T, typename Default>
using type_or = std::conditional_t<std::is_same<T, void>::value, Default, T>;

}  // namespace type_traits;
using namespace type_traits;
}  // namepsace common;

#endif  // TYPE_TRAITS_H
