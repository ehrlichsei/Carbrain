#ifndef SIMULTANEOUS_SORT_H
#define SIMULTANEOUS_SORT_H

#include <boost/range/algorithm/copy.hpp>
#include <boost/range/algorithm/sort.hpp>
#include <boost/range/combine.hpp>
#include <boost/tuple/tuple.hpp>
#include <iterator>
#include <vector>

namespace common {
namespace algorithm {

/*!
 * \brief simultaneous_sort
 * sorts range and applies the same permutation to all secondary_ranges,
 * retaining any correspondence between an element at index i in range and the
 * elements in the secondary_ranges at the same position i.
 * e.g. if vector o stores some arbitrary objects, vector id the corresponding
 * ids and vector s the scores (with id[i] being the id for object o[i] and s[i]
 * its score), simultaneous_sort(s, std::greater(), o, id) sorts the objects,
 * scores and ids in descending order of the scores, while retaining the relative
 * order between the containers.
 * \param range boost random access range to sort
 * \param comp comparison predicative for sorting, (e.g. std::less() for sorting
 * in ascending order)
 * \param secondary_ranges other boost random access ranges, to which the same
 * sorting permutation is applied, that is also applied to range (no comparison
 * is done on the secondary_ranges, the elements don't have to be comparable)
 * \pre range and secondary_ranges have to have the same size
 */
template <typename PrimaryRange, typename Compare, typename... SecondaryRanges>
void simultaneous_sort(PrimaryRange& range, Compare comp, SecondaryRanges&... secondary_ranges) {
  const auto tuple_range = boost::combine(range, secondary_ranges...);
  std::vector<boost::tuple<typename boost::range_value<PrimaryRange>::type, typename boost::range_value<SecondaryRanges>::type...>> zipped(
      tuple_range.begin(), tuple_range.end());
  boost::sort(zipped, [&comp](const auto& t0, const auto& t1) {
    return comp(boost::get<0>(t0), boost::get<0>(t1));
  });
  boost::copy(zipped, boost::begin(tuple_range));
}

}  // namespace algorithm
using namespace algorithm;
}  // namespace common

#endif  // SIMULTANEOUS_SORT_H
