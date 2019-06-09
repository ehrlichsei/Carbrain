#ifndef KITCAR_DERIVATIVE_TYPE_CALCULATION_H
#define KITCAR_DERIVATIVE_TYPE_CALCULATION_H

#include "common/polynomial_detail/infinitissimal_count.h"
#include "common/polynomial_degree.h"

namespace common {
namespace polynomial_detail {

template <PolynomialDegree MAX_DEGREE, InfinitesimalCount COUNT, typename T>
struct DerivativeTypeCalculation {
  typedef typename DerivativeTypeCalculation<MAX_DEGREE - 1, COUNT - 1, T>::type type;
};

template <PolynomialDegree MAX_DEGREE, typename T>
struct DerivativeTypeCalculation<MAX_DEGREE, 0, T> {
  typedef Polynomial<MAX_DEGREE, T> type;
};

template <InfinitesimalCount COUNT, typename T>
struct DerivativeTypeCalculation<PolynomialDegrees::MinusInfinity, COUNT, T> {
  typedef Polynomial<PolynomialDegrees::MinusInfinity, T> type;
};

template <InfinitesimalCount COUNT, typename T>
struct DerivativeTypeCalculation<PolynomialDegrees::Dynamic, COUNT, T> {
  typedef Polynomial<PolynomialDegrees::Dynamic, T> type;
};

template <typename T>
struct DerivativeTypeCalculation<PolynomialDegrees::MinusInfinity, 0, T> {
  typedef Polynomial<PolynomialDegrees::MinusInfinity, T> type;
};

template <typename T>
struct DerivativeTypeCalculation<PolynomialDegrees::Dynamic, 0, T> {
  typedef Polynomial<PolynomialDegrees::Dynamic, T> type;
};
} // namespace polynomial_detail
} // namespace common

#endif  // KITCAR_DERIVATIVE_TYPE_CALCULATION_H
