#ifndef COMMON_DISCRETISIZEPOLYNOM_H
#define COMMON_DISCRETISIZEPOLYNOM_H

#include "common/types.h"

namespace common {
/*!
 * \namespace common::discretization
 * \brief discretization contains functionality to discretisize a given
 * 1D-function(-object).
 */
namespace discretization {
/*!
 * \brief The DiscretizationParams struct contains the parameters for
 * discretization.
 *
 * The discrectisation will take place in the range [min, max) with step size
 * step.
 *
 * The default constructed struct results in a noop (in discretisize()).
 *
 * \pre All values have to be finite.
 * \pre min != max implies min > max and step > 0.
 */
struct DiscretizationParams {
  double min = 0.0;
  double max = 0.0;
  double step = 0.0;
};

/*!
 * \brief discretisize discretisize a given function (a callable object).
 * \param function the function to discretisize.
 * \param params the parameters to use for the discretization.
 * \param points the vector where the discretization points will be put in.
 * \pre all DiscretizationParams needs to be finite.
 * \pre if params.min != param.max then params.max >= params.min and step > 0.
 * \post points.size() == (params.max - params.min) / params.step.
 *
 * \b Note: default constructed DiscretizationParams will lead to a noop.
 */
template <class VectorXdVector, class Function>
inline void discretisize(const Function& function,
                         const DiscretizationParams& params,
                         VectorXdVector* points) {
  assert(std::isfinite(params.step) && std::isfinite(params.min) &&
         std::isfinite(params.max));

  if (params.min == params.max) {
    return;
  }

  assert(params.max >= params.min);
  assert(params.step > 0);
  points->reserve(static_cast<std::size_t>((params.max - params.min) / params.step));

  using EigenVector = typename VectorXdVector::value_type;
  for (double x = params.min; x < params.max; x += params.step) {
    EigenVector point = EigenVector::Zero();
    point.x() = x;
    point.y() = function(x);
    points->push_back(point);
  }
}

/*!
 * \brief discretisize discretisize a given function (a callable object).
 * \param function the function to discretisize.
 * \param params the parameters to use for the discretization.
 * \return the vector containing the discretization points.
 *
 * \pre all DiscretizationParams needs to be finite.
 * \pre if params.min != param.max then params.max >= params.min and step > 0.
 * \post ret.size() == (params.max - params.min) / params.step.
 *
 * \b Note: default constructed DiscretizationParams will lead to a noop.
 */
template <class Vector, class Function>
auto discretisize(const Function& function, const DiscretizationParams& params) {
  std::vector<Vector> ps;
  discretisize(function, params, &ps);
  return ps;
}

}  // namespace discretization;
using namespace discretization;
}  // namespace common

#endif  // COMMON_DISCRETISIZEPOLYNOM_H
