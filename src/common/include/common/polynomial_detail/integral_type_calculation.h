#ifndef KITCAR_INTEGRATIVE_TYPE_CALCULATION_H
#define KITCAR_INTEGRATIVE_TYPE_CALCULATION_H
#include "common/polynomial_detail/infinitissimal_count.h"
#include "common/polynomial_degree.h"

namespace common {
namespace polynomial_detail {

template <PolynomialDegree MAX_DEGREE, InfinitesimalCount COUNT, typename T>
struct IntegralTypeCalculation {
  typedef typename IntegralTypeCalculation<MAX_DEGREE + 1, COUNT - 1, T>::type type;
};

template <PolynomialDegree MAX_DEGREE, typename T>
struct IntegralTypeCalculation<MAX_DEGREE, 0, T> {
  typedef Polynomial<MAX_DEGREE, T> type;
};

template <InfinitesimalCount COUNT, typename T>
struct IntegralTypeCalculation<PolynomialDegrees::MinusInfinity, COUNT, T> {
  typedef Polynomial<PolynomialDegrees::MinusInfinity, T> type;
};

template <InfinitesimalCount COUNT, typename T>
struct IntegralTypeCalculation<PolynomialDegrees::Dynamic, COUNT, T> {
  typedef Polynomial<PolynomialDegrees::Dynamic, T> type;
};

template <typename T>
struct IntegralTypeCalculation<PolynomialDegrees::MinusInfinity, 0, T> {
  typedef Polynomial<PolynomialDegrees::MinusInfinity, T> type;
};

template <typename T>
struct IntegralTypeCalculation<PolynomialDegrees::Dynamic, 0, T> {
  typedef Polynomial<PolynomialDegrees::Dynamic, T> type;
};

} // polynomial_detail
} // common namespace

#endif  // KITCAR_INTEGRATIVE_TYPE_CALCULATION_H
