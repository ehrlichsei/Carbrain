#pragma once

#include <set>
#include <sstream>
#include <iterator>

namespace common {
namespace algorithm {

/*!
 * \brief toString converts any given range into a std::string.
 *
 * \param begin the begin of the range.
 * \param end the end of the range.
 * \param d the delimited between the elements of the range.
 * \return a string representation of the range.
 */

template <class IT>
std::string toString(IT begin, IT end, const char* d = " ") {
  using value_type = std::remove_reference_t<decltype(*begin)>;

  if (begin == end) {
    return {};
  }

  std::ostringstream oss;
  std::copy(begin, std::prev(end), std::ostream_iterator<value_type>(oss, d));

  oss << *std::prev(end);

  return oss.str();
}

/*!
 * \brief toString converts any given range into a std::string.
 *
 * \param c the range.
 * \param d the delimited between the elements of the range.
 * \return a string representation of the range.
 */
template <class T>
std::string toString(const T& c, const char* d = " ") {
  return toString(std::begin(c), std::end(c), d);
}

/*!
 * \brief toSet converts any container into a set.
 *
 * \param container the container.
 * \return a std::set containing the elements of container.
 */
template <class T>
auto toSet(const T& container) {
  using value_type = std::remove_reference_t<decltype(*std::declval<T>().begin())>;
  return std::set<value_type>(container.begin(), container.end());
}

}  // namespace algorithm;
using namespace algorithm;
}  // namespace common;
