#ifndef KITCAR_PRODUCT_TYPE_CALCULATION_H
#define KITCAR_PRODUCT_TYPE_CALCULATION_H

#include <type_traits>

#include "common/polynomial_degree.h"

namespace common {
namespace polynomial_detail {

template <PolynomialDegree FIRST_FACTOR_MAX_DEGREE, PolynomialDegree SECOND_FACTOR_MAX_DEGREE, typename T>
struct ProductTypeCalculation {
  typedef std::conditional_t<FIRST_FACTOR_MAX_DEGREE == PolynomialDegrees::MinusInfinity ||
                                 SECOND_FACTOR_MAX_DEGREE == PolynomialDegrees::MinusInfinity,
                             Polynomial<PolynomialDegrees::MinusInfinity, T>,
                             Polynomial<FIRST_FACTOR_MAX_DEGREE + SECOND_FACTOR_MAX_DEGREE, T> > type;
};

template <typename T>
struct ProductTypeCalculation<PolynomialDegrees::Dynamic, PolynomialDegrees::Dynamic, T> {
  typedef Polynomial<PolynomialDegrees::Dynamic, T> type;
};

}  // polynomial_detail
}  // namespace common

#endif  // KITCAR_PRODUCT_TYPE_CALCULATION_H
