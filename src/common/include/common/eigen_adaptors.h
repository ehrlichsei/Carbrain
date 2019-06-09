#ifndef COMMON_EIGEN_ADAPTORS_H
#define COMMON_EIGEN_ADAPTORS_H
#include <boost/range/adaptor/transformed.hpp>
#include "common/type_traits.h"
#include "common/eigen_utils.h"
#include "common/eigen_functors.h"
#include "common/adaptors.h"

namespace common {
namespace range {

namespace range_detail {
template <typename T>
struct eigen_caster {
  template <typename S>
  auto operator()(const S& s) const {
    return toVector(s.template cast<T>());
  }
};
}  // namespace range_detail

/*!
 * \brief eigen_casted is a range adaptor which applies the cast<T>()-function
 * to the elements of the input-range.
 */
template <typename T>
const auto eigen_casted = make_transform_holder(range_detail::eigen_caster<T>{});

/*!
 * \brief eigen_transformed creates a range adaptor wich applies the given
 * Eigen-transformation to the given range.
 */
template <typename T>
auto eigen_transformed(const T& t) {
  return make_transform_holder(eigen_transformer(t));
}

}  // namespace range;
using namespace range;
}  // namespace common

#endif  // COMMON_EIGEN_ADAPTORS_H
