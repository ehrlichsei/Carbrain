#pragma once

#include <boost/multi_index_container.hpp>
#include <boost/multi_index/ordered_index.hpp>
#include <boost/multi_index/sequenced_index.hpp>
#include <boost/multi_index/member.hpp>

namespace common {

namespace types_detail {
template <typename T1, typename T2>
struct mutable_pair {
  typedef T1 first_type;
  typedef T2 second_type;

  mutable_pair() : first(T1()), second(T2()) {}
  mutable_pair(const T1& f, const T2& s) : first(f), second(s) {}
  mutable_pair(const std::pair<T1, T2>& p) : first(p.first), second(p.second) {}

  T1 first;
  mutable T2 second;
};

}  // common types_detail

namespace types {
using boost::multi_index::indexed_by;
using boost::multi_index::ordered_unique;
using boost::multi_index::member;
using boost::multi_index::sequenced;
using boost::multi_index::tag;

/*!
 * \brief fifo the tag to acceess the elemenets of fifo_map in the order of
 * insertion.
 */
struct fifo {};

/*!
 * \brief fifo_map is an associative container (aka a map/dictionary) which
 * preserves the order of the insertion of the elements.
 */
template <typename KEY, typename VALUE>
using fifo_map = boost::multi_index_container<
    types_detail::mutable_pair<KEY, VALUE>,
    indexed_by<
        // sort by KEY
        ordered_unique<member<types_detail::mutable_pair<KEY, VALUE>, KEY, &types_detail::mutable_pair<KEY, VALUE>::first>>,
        // sort fifo
        sequenced<tag<fifo>>>>;

}  // namespace types
using types::fifo_map;
using types::fifo;
}  // namespace common
