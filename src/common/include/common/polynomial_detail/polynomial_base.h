#ifndef KITCAR_POLYNOMIAL_BASE_H
#define KITCAR_POLYNOMIAL_BASE_H

#include <cmath>
#include <vector>
#include <algorithm>
#include "common/polynomial_detail/derivative_type_calculation.h"
#include "common/polynomial_detail/infinitissimal_count.h"
#include "common/polynomial_detail/integral_type_calculation.h"
#include "common/polynomial_degree.h"

namespace common {
namespace polynomial_detail {

template <PolynomialDegree MAX_DEGREE, typename T>
class PolynomialBase {
 public:
  template <InfinitesimalCount COUNT>
  struct Derivative {
    typedef typename polynomial_detail::DerivativeTypeCalculation<MAX_DEGREE, COUNT, T>::type type;
  };

  typedef typename Derivative<1>::type FirstDerivative;
  typedef typename Derivative<2>::type SecondDerivative;
  typedef typename Derivative<3>::type ThirdDerivative;

  template <InfinitesimalCount COUNT>
  typename Derivative<COUNT>::type calculateDerivative() const {
    return self()->template calculateDerivative<COUNT>();
  }

  /**
   * @brief Alias of calculateFirstDerivative
   * @return First derivative
   */
  FirstDerivative derivate() const { return calculateFirstDerivative(); }

  FirstDerivative calculateFirstDerivative() const {
    return calculateDerivative<1>();
  }

  SecondDerivative calculateSecondDerivative() const {
    return calculateDerivative<2>();
  }

  ThirdDerivative calculateThirdDerivative() const {
    return calculateDerivative<3>();
  }

  template <InfinitesimalCount COUNT>
  struct Integral {
    typedef typename IntegralTypeCalculation<MAX_DEGREE, COUNT, T>::type type;
  };

  typedef typename Integral<1>::type FirstIntegral;
  typedef typename Integral<2>::type SecoundIntegral;
  typedef typename Integral<3>::type ThirdIntegral;

  template <InfinitesimalCount COUNT>
  typename Integral<COUNT>::type calculateIntegral() const {
    return self()->template calculateIntegral<COUNT>();
  }

  FirstIntegral integrate() const { return calculateFirstIntegral(); }

  FirstIntegral calculateFirstIntegral() const {
    return calculateIntegral<1>();
  }

  SecoundIntegral calculateSecoundIntegral() const {
    return calculateIntegral<2>();
  }

  ThirdIntegral calculateThirdIntegral() const {
    return calculateIntegral<3>();
  }


  /**
   * @brief Alias of calculateFirstDerivative
   * @return First derivative
   */
  T operator()(const T x) const { return evaluate(x); }

  T evaluate(const T x) const { return self()->evaluate(x); }

  std::vector<T> evaluate(const std::vector<T>& xs) const {
    std::vector<T> ys;
    ys.reserve(xs.size());
    std::transform(std::begin(xs), std::end(xs), std::back_inserter(ys), *self());
    return ys;
  }

  /**
   * @brief Calculates the Reduction Error
   *
   * Calculates a measure for the error that would occour
   * if the polynomial was reduced to the specified degree.
   *
   * @param degree The degree to test for
   * @return A measure for the error larger than 0
   */
  T calculateReductionError(const PolynomialDegree degree) const {
    return self()->calculateReductionError(degree);
  }

  /**
   * @brief getCoefficients
   *
   * The coefficients are returned in reverse order. For example
   * the polynomial 8x³ + 9x² + 10x + 7 returns [7, 10, 9, 8].
   *
   * @return A vector of the coefficients
   */
  std::vector<T> getCoefficients() const { return self()->getCoefficients(); }

  T calculateCurvature(const T x) const {
    const T dy = self()->calculateFirstDerivative().evaluate(x);
    const T ddy = self()->calculateSecondDerivative().evaluate(x);
    const T curvature = ddy / std::pow(1.0 + dy * dy, 1.5);
    return curvature;
  }

  template <typename S>
  Polynomial<MAX_DEGREE, S> cast() const {
    const auto& coeffs = self()->getCoefficientsList();
    return Polynomial<MAX_DEGREE, S>(coeffs.begin(), coeffs.end());
  }

 private:
  typedef Polynomial<MAX_DEGREE, T> DerivedType;

  const DerivedType* self() const {
    return static_cast<const DerivedType*>(this);
  }
};
}  // polynomial_detail
}  // common namespace

#endif  // KITCAR_POLYNOMIAL_BASE_H
