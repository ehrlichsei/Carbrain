#ifndef KITCAR_SUM_TYPE_CALCULATION_H
#define KITCAR_SUM_TYPE_CALCULATION_H

#include <type_traits>

#include "common/polynomial_degree.h"

namespace common {
namespace polynomial_detail {

template <PolynomialDegree FIRST_FACTOR_MAX_DEGREE, PolynomialDegree SECOND_FACTOR_MAX_DEGREE, typename T>
struct SumTypeCalculation {
  typedef std::conditional_t<FIRST_FACTOR_MAX_DEGREE >= SECOND_FACTOR_MAX_DEGREE,
                             Polynomial<FIRST_FACTOR_MAX_DEGREE, T>,
                             Polynomial<SECOND_FACTOR_MAX_DEGREE, T>> type;
};

template <PolynomialDegree FIRST_FACTOR_MAX_DEGREE, typename T>
struct SumTypeCalculation<FIRST_FACTOR_MAX_DEGREE, PolynomialDegrees::MinusInfinity, T> {
  typedef Polynomial<FIRST_FACTOR_MAX_DEGREE, T> type;
};

template <PolynomialDegree SECOND_FACTOR_MAX_DEGREE, typename T>
struct SumTypeCalculation<PolynomialDegrees::MinusInfinity, SECOND_FACTOR_MAX_DEGREE, T> {
  typedef Polynomial<SECOND_FACTOR_MAX_DEGREE, T> type;
};

template <typename T>
struct SumTypeCalculation<PolynomialDegrees::MinusInfinity, PolynomialDegrees::MinusInfinity, T> {
  typedef Polynomial<PolynomialDegrees::MinusInfinity, T> type;
};

template <typename T>
struct SumTypeCalculation<PolynomialDegrees::Dynamic, PolynomialDegrees::Dynamic, T> {
  typedef Polynomial<PolynomialDegrees::Dynamic, T> type;
};

}  // namesapce polynomial_detail
}  // namespace common

#endif  // KITCAR_SUM_TYPE_CALCULATION_H
