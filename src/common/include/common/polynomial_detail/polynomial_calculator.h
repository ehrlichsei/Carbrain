#ifndef KITCAR_POLYNOMIAL_CALCULATOR_H
#define KITCAR_POLYNOMIAL_CALCULATOR_H

#include <assert.h>
#include <cmath>

#include "common/polynomial_detail/infinitissimal_count.h"
#include "common/polynomial_degree.h"

namespace common {
namespace polynomial_detail {

template <typename C, typename T>
T evaluate(const C &coefficients, const T x) {
  assert(!coefficients.empty());
  const PolynomialDegree highest_degree =
      polynomial_degree_cast(coefficients.size()) - 1;
  T result = coefficients[highest_degree];
  for (PolynomialDegree degree = highest_degree - 1; degree >= 0; --degree) {
    result = result * x + coefficients[degree];
  }
  return result;
}

template <InfinitesimalCount COUNT, typename C>
void derive(C &coefficients) {
  for (InfinitesimalCount derivative = 1; derivative <= COUNT; ++derivative) {
    for (size_t i = 0; i < coefficients.size(); ++i) {
      const PolynomialDegree minimal_degree = 1 + COUNT - derivative;
      const PolynomialDegree degree = minimal_degree + PolynomialDegree(i);
      coefficients[i] = degree * coefficients[i];
    }
  }
}

template <InfinitesimalCount COUNT, typename C>
void integrate(C &coefficients) {
  for (InfinitesimalCount integral = 0; integral < COUNT; ++integral) {
    for (auto i = polynomial_degree_cast(coefficients.size()) - 1; i > integral; --i) {
      coefficients[i] = coefficients[i - 1] / i;
    }
    coefficients[integral] = 0.0;
  }
}

template <typename P, typename F1, typename F2>
void multiply(P &product_coefficients,
              const F1 &first_factor_coefficients,
              const F2 &second_factor_coefficients) {
  assert(first_factor_coefficients.size() > 0);
  assert(second_factor_coefficients.size() == 0 ||
         product_coefficients.size() - 1 ==
             first_factor_coefficients.size() - 1 + second_factor_coefficients.size() - 1);

  for (size_t i = 0; i < first_factor_coefficients.size(); i++) {
    for (size_t j = 0; j < second_factor_coefficients.size(); j++) {
      product_coefficients[i + j] +=
          first_factor_coefficients[i] * second_factor_coefficients[j];
    }
  }
}

template <typename P, typename F1, typename F2>
void add(P &sum_coefficients, const F1 &first_summand_coefficients, const F2 &second_summand_coefficients) {
  assert(sum_coefficients.size() == first_summand_coefficients.size() ||
         sum_coefficients.size() == second_summand_coefficients.size());
  assert(sum_coefficients.size() >= first_summand_coefficients.size() &&
         sum_coefficients.size() >= second_summand_coefficients.size());

  for (size_t i = 0; i < first_summand_coefficients.size(); i++) {
    sum_coefficients[i] += first_summand_coefficients[i];
  }

  for (size_t j = 0; j < second_summand_coefficients.size(); j++) {
    sum_coefficients[j] += second_summand_coefficients[j];
  }
}

template <typename P, typename F1, typename F2>
void minus(P &difference_coefficients, const F1 &minuend_coefficients, const F2 &subtrahend_coefficients) {
  assert(difference_coefficients.size() == minuend_coefficients.size() ||
         difference_coefficients.size() == subtrahend_coefficients.size());
  assert(difference_coefficients.size() >= minuend_coefficients.size() &&
         difference_coefficients.size() >= subtrahend_coefficients.size());

  for (size_t i = 0; i < minuend_coefficients.size(); i++) {
    difference_coefficients[i] += minuend_coefficients[i];
  }

  for (size_t j = 0; j < subtrahend_coefficients.size(); j++) {
    difference_coefficients[j] -= subtrahend_coefficients[j];
  }
}

template <typename C, typename T = typename std::remove_reference<decltype(std::declval<C>().front())>::type>
T calculateReductionError(const C &coefficients, const PolynomialDegree degree) {
  assert(degree >= PolynomialDegrees::MinusInfinity);
  T error = 0;
  for (size_t d = static_cast<size_t>(degree + 1); d < coefficients.size(); ++d) {
    error += coefficients[d] * coefficients[d];
  }
  return sqrt(error);
}

}  // namespace polynomial_detail
}  // namespace common

#endif  // KITCAR_POLYNOMIAL_CALCULATOR_H
