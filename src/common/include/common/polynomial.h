#ifndef KITCAR_POLYNOMIAL_H
#define KITCAR_POLYNOMIAL_H

#include <array>
#include <ostream>
#include <vector>
#include "common/polynomial_degree.h"
#include "common/polynomial_detail/derivative_type_calculation.h"
#include "common/polynomial_detail/infinitissimal_count.h"
#include "common/polynomial_detail/polynomial_base.h"
#include "common/polynomial_detail/polynomial_calculator.h"
#include "common/polynomial_detail/product_type_calculation.h"
#include "common/polynomial_detail/sum_type_calculation.h"

namespace common {
/*!
 * \brief The Polynomial class implements a polynom and the most common
 * operations on polynomials like evaluation, derivation, intergation,
 * substraction, addition and multiplication.
 *
 * This class can be used with differens scalar types like double or float.
 * Integer-types should also work but are not tested extensively. There are
 * typedef for float polynomials (e.g. DynamicPolynomialf) and double
 * polynomials (e.g. DynamicPolynomiald).
 *
 * There are basically two version of polynomials: static and dynamic
 * polynomials. Static polynomials (LinearPolynomiald, CubicPolynomialf) have
 * fixes size at runtime and are allocated on the stack. Dynamic polynomials
 * (DynamicPolynomiald) have dynamic size at runtime and are allocated on the
 * heap.
 */
namespace polynomial {
/*!
 * \brief The Polynomial class is the implementation for static polynomials.
 *
 * These polynomials have fixed size at runtime and stack allocated.
 */
template <PolynomialDegree MAX_DEGREE, typename T>
class Polynomial : public polynomial_detail::PolynomialBase<MAX_DEGREE, T> {
 public:
  template <InfinitesimalCount COUNT>
  struct Derivative {
    typedef typename polynomial_detail::DerivativeTypeCalculation<MAX_DEGREE, COUNT, T>::type type;
    static const PolynomialDegree coefficients_count = type::TECHNICAL_DEGREE + 1;
    typedef std::array<T, coefficients_count> CoefficientList;
  };

  template <InfinitesimalCount COUNT>
  struct Integral {
    typedef typename polynomial_detail::IntegralTypeCalculation<MAX_DEGREE, COUNT, T>::type type;
    static const PolynomialDegree coefficients_count = type::TECHNICAL_DEGREE + 1;
    typedef std::array<T, coefficients_count> CoefficientList;
  };

  template <PolynomialDegree FACTOR_MAX_DEGREE>
  struct Product {
    typedef typename polynomial_detail::ProductTypeCalculation<MAX_DEGREE, FACTOR_MAX_DEGREE, T>::type type;
    static const PolynomialDegree coefficients_count = type::TECHNICAL_DEGREE + 1;
    typedef std::array<T, coefficients_count> CoefficientList;
  };

  template <PolynomialDegree SUMMAND_MAX_DEGREE>
  struct Sum {
    typedef typename polynomial_detail::SumTypeCalculation<MAX_DEGREE, SUMMAND_MAX_DEGREE, T>::type type;
    static const PolynomialDegree coefficients_count = type::TECHNICAL_DEGREE + 1;
    typedef std::array<T, coefficients_count> CoefficientList;
  };

  typedef std::array<T, MAX_DEGREE + 1> CoefficientList;

  static const PolynomialDegree TECHNICAL_DEGREE = MAX_DEGREE;

  template <class IT>
  Polynomial(IT begin, IT end) {
    assert(static_cast<std::size_t>(std::distance(begin, end)) == coefficients.size());
    std::copy(begin, end, coefficients.begin());
  }

  /*!
   * \brief Creates a polynomial for the given coefficients
   *
   * \param coefficients the coefficients of the polynomial
   */
  Polynomial(const std::vector<T> &coefficients) {
    assert(coefficients.size() <= this->coefficients.size());
    std::copy(coefficients.begin(), coefficients.end(), this->coefficients.begin());
  }

  /*!
   * \brief Creates a polynomial for the given coefficients
   *
   * \param coefficients the coefficients of the polynomial
   */
  Polynomial(const std::initializer_list<T> &coefficients) {
    assert(coefficients.size() <= this->coefficients.size());
    std::copy(coefficients.begin(), coefficients.end(), this->coefficients.begin());
  }

  /*!
   * \brief Creates a polynomial for the given coefficients
   *
   * \param coefficients the coefficients of the polynomial
   *
   * \code
   *    // 8x³ + 9x² + 10x + 7
   *    CubicPolynomial::CoefficientList coefficients = {7, 10, 9, 8};
   *    CubicPolynomial polynomial(coefficients);
   * \endcode
   */
  Polynomial(const CoefficientList &coefficients)
      : coefficients(coefficients) {}

  /*!
   * \brief Creates a polynomial for the given coefficient: p(x) = c
   * \param coefficient the coefficient of the polynomial
   */
  Polynomial(const T coefficient) { coefficients.front() = coefficient; }

  /*!
   * \brief Creates a polynomial of degree MAX_DEGREE from a static polynomial
   * of degree DEGREE
   * if MAX_DEGREE >= DEGREE. This is the only case of implicit conversion of
   * static polynomials
   * because this is the only case where a successful conversion without any
   * loss can be guaranteed at compile time.
   *
   * \param polynom the source polynomial
   */
  template <PolynomialDegree DEGREE,
            typename std::enable_if<MAX_DEGREE >= DEGREE && DEGREE != PolynomialDegrees::Dynamic, int>::type = 0>
  Polynomial(const Polynomial<DEGREE, T> &polynom) {
    const auto &coefficients = polynom.getCoefficientsList();
    std::copy(coefficients.begin(), coefficients.end(), this->coefficients.begin());
  }

  /*!
   * \brief Creates a polynomial from any other polynomial. This conversion has
   * to be done explicitly because a
   * lossless conversion can not be guaranteed.
   * \param polynom the source polynomial
   */
  template <PolynomialDegree DEGREE>
  explicit Polynomial(const Polynomial<DEGREE, T> &polynom) {
    const auto &coefficients = polynom.getCoefficientsList();
    std::copy_n(coefficients.begin(),
                std::min(coefficients.size(), this->coefficients.size()),
                this->coefficients.begin());
  }

  T evaluate(const T x) const {
    return polynomial_detail::evaluate(coefficients, x);
  }

  std::vector<T> evaluate(const std::vector<T> &x_vector) const {
    return polynomial_detail::PolynomialBase<MAX_DEGREE, T>::evaluate(x_vector);
  }

  /**
   * @brief Multiplication of two polynomials
   *
   * @param factor the factor to multiply with
   * @return product polynomial
   */
  template <PolynomialDegree FACTOR_MAX_DEGREE>
  typename Product<FACTOR_MAX_DEGREE>::type operator*(const Polynomial<FACTOR_MAX_DEGREE, T> &factor) const {
    typename Product<FACTOR_MAX_DEGREE>::CoefficientList product_coefficients = {};
    typedef typename Product<FACTOR_MAX_DEGREE>::type ProductType;
    polynomial_detail::multiply(
        product_coefficients, coefficients, factor.getCoefficientsList());
    return ProductType(product_coefficients);
  }

  /**
   * @brief Sum of two polynomials
   *
   * @param summand to summ with
   * @return sum polynomial
   */
  template <PolynomialDegree SUMMAND_MAX_DEGREE>
  typename Sum<SUMMAND_MAX_DEGREE>::type operator+(const Polynomial<SUMMAND_MAX_DEGREE, T> &summand) const {
    typename Sum<SUMMAND_MAX_DEGREE>::CoefficientList sum_coefficients = {};
    typedef typename Sum<SUMMAND_MAX_DEGREE>::type SumType;
    polynomial_detail::add(sum_coefficients, coefficients, summand.getCoefficientsList());
    return SumType(sum_coefficients);
  }

  /**
   * @brief Difference of two polynomials
   *
   * @param subtrahend to subtract
   * @return difference polynomial
   */
  template <PolynomialDegree SUBTRAHEND_MAX_DEGREE>
  typename Sum<SUBTRAHEND_MAX_DEGREE>::type operator-(
      const Polynomial<SUBTRAHEND_MAX_DEGREE, T> &subtrahend) const {
    typename Sum<SUBTRAHEND_MAX_DEGREE>::CoefficientList difference_coefficients = {};
    typedef typename Sum<SUBTRAHEND_MAX_DEGREE>::type SumType;
    polynomial_detail::minus(
        difference_coefficients, coefficients, subtrahend.getCoefficientsList());
    return SumType(difference_coefficients);
  }

  /*!
   * \brief operator - Flips signe of every coefficient.
   * \return the negated polynomial.
   */
  Polynomial<MAX_DEGREE, T> operator-() const {
    return *this * Polynomial<PolynomialDegrees::Constant, T>(-1.0);
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
    return polynomial_detail::calculateReductionError(coefficients, degree);
  }

  /**
   * @brief getCoefficients
   *
   * The coefficients are returned in reverse order. For example
   * the polynomial 8x³ + 9x² + 10x + 7 returns [7, 10, 9, 8].
   *
   * @return A vector of the coefficients
   */
  std::vector<T> getCoefficients() const {
    return std::vector<T>(this->coefficients.begin(), this->coefficients.end());
  }

  const CoefficientList &getCoefficientsList() const { return coefficients; }

  template <InfinitesimalCount COUNT>
  typename Derivative<COUNT>::type calculateDerivative() const {
    typename Derivative<COUNT>::CoefficientList coefficients;
    copyCoefficients<COUNT>(coefficients);
    polynomial_detail::derive<COUNT>(coefficients);
    typedef typename Derivative<COUNT>::type DerivativeType;
    return DerivativeType(coefficients);
  }

  template <InfinitesimalCount COUNT>
  typename Integral<COUNT>::type calculateIntegral() const {
    typename Integral<COUNT>::CoefficientList coefficients;
    std::copy(this->coefficients.cbegin(), this->coefficients.cend(), coefficients.begin());
    coefficients.back() = 0.0;
    polynomial_detail::integrate<COUNT>(coefficients);
    typedef typename Integral<COUNT>::type IntegralType;
    return IntegralType(coefficients);
  }

  bool operator==(const Polynomial<MAX_DEGREE, T> &other_polynomial) const {
    return coefficients == other_polynomial.coefficients;
  }

  bool operator!=(const Polynomial<MAX_DEGREE, T> &other_polynomial) const {
    return !(*this == other_polynomial);
  }

  std::ostream &appendToStream(std::ostream &stream) const {
    for (PolynomialDegree degree = MAX_DEGREE; degree >= 1; --degree) {
      stream << coefficients[degree] << " x^" << degree << " + ";
    }
    stream << coefficients[0];
    return stream;
  }

  /*!
   * \brief operator + adds a constant to a polynomial. This is needed because
   * impicit type conversion,
   * did not work in this context.
   * \param polynomial the polynomial to add.
   * \param d the constant to add to polynomial
   * \return polynomial + d
   */
  friend Polynomial<MAX_DEGREE, T> operator+(const Polynomial<MAX_DEGREE, T> &polynomial,
                                             const T d) {
    return polynomial + Polynomial<PolynomialDegrees::Constant, T>(d);
  }

  /*!
   * \brief operator + is needed to achieve symetrical behaviour.
   * \param d the constant to add.
   * \param polynomial the polynomial
   * \return polynomial + d
   */
  friend Polynomial<MAX_DEGREE, T> operator+(const T d,
                                             const Polynomial<MAX_DEGREE, T> &polynomial) {
    return polynomial + d;
  }

  /*!
   * \brief operator - subtrackts a constant from a polynomial. This is needed
   * because impicit type conversion,
   * did not work in this context.
   * \param polynomial the polynomial.
   * \param d the constant to add to polynomial
   * \return polynomial - d
   */
  friend Polynomial<MAX_DEGREE, T> operator-(const Polynomial<MAX_DEGREE, T> &polynomial,
                                             const T d) {
    return polynomial - Polynomial<PolynomialDegrees::Constant, T>(d);
  }

  /*!
   * \brief operator - subtrackts a polynomial from a constant.
   * \param d the constant to substract from.
   * \param polynomial the polynomial to subtract.
   * \return d - polynomial
   */
  friend Polynomial<MAX_DEGREE, T> operator-(const T d,
                                             const Polynomial<MAX_DEGREE, T> &polynomial) {
    return Polynomial<PolynomialDegrees::Constant, T>(d) - polynomial;
  }

 protected:
  template <InfinitesimalCount COUNT, typename C>
  void copyCoefficients(C &coefficients) const {
    std::copy(this->coefficients.cend() - Derivative<COUNT>::coefficients_count,
              this->coefficients.cend(),
              coefficients.begin());
  }

  // = {} perfroms aggregate initialization.
  // aggregate initialization of an array with empty braces performs value
  // initialization of every element.
  // value initialization calls the default constructor or performs zero
  // initialization.
  // zero initialization for scalar types converts the integral constant zero to
  // the scalar type.
  CoefficientList coefficients = {};
};

/*!
 * \brief The Polynomial<PolynomialDegrees::Dynamic, T> class is the
 * implemetation for dynamic polynomials.
 *
 * These polynomials have dynamic size at runtime and are heap allocated.
 */
template <typename T>
class Polynomial<PolynomialDegrees::Dynamic, T>
    : public polynomial_detail::PolynomialBase<PolynomialDegrees::Dynamic, T> {

 public:
  template <InfinitesimalCount COUNT>
  class Derivative {
   public:
    const Polynomial<PolynomialDegrees::Dynamic, T> *polynomial;
    const PolynomialDegree technical_degree;
    const PolynomialDegree coefficients_count;

    Derivative(const Polynomial<PolynomialDegrees::Dynamic, T> *polynomial)
        : polynomial(polynomial),
          technical_degree(calculateTechnicalDegree(polynomial)),
          coefficients_count(technical_degree + 1) {}

    static PolynomialDegree calculateTechnicalDegree(
        const Polynomial<PolynomialDegrees::Dynamic, T> *polynomial) {
      const auto other_size = polynomial->coefficients.size();
      const PolynomialDegree former_degree =
          other_size == 0 ? 0 : polynomial_degree_cast(other_size) - 1;
      return COUNT > former_degree ? PolynomialDegrees::MinusInfinity
                                   : former_degree - COUNT;
    }

    bool isZeroPolynominal() const {
      return technical_degree == PolynomialDegrees::MinusInfinity;
    }
  };

  typedef std::vector<T> CoefficientList;

  /*!
   * \brief Creates a zero polynomial
   */
  Polynomial() : coefficients() {}

  Polynomial(const Polynomial<PolynomialDegrees::Dynamic, T> &) = default;
  Polynomial(Polynomial<PolynomialDegrees::Dynamic, T> &&) = default;
  Polynomial<PolynomialDegrees::Dynamic, T> &operator=(
      const Polynomial<PolynomialDegrees::Dynamic, T> &) = default;
  Polynomial<PolynomialDegrees::Dynamic, T> &operator=(
      Polynomial<PolynomialDegrees::Dynamic, T> &&) = default;

  template <class IT>
  Polynomial(IT begin, IT end)
      : coefficients(begin, end) {}
  /*!
   * \brief Creates a polynomial for the given coefficients
   *
   * \param coefficients the coefficients of the polynomial
   */
  Polynomial(const CoefficientList &coefficients)
      : coefficients(coefficients) {}
  /*!
     * \brief Creates a polynomial for the given coefficients
     *
     * \param coefficients the coefficients of the polynomial
     */
  Polynomial(const std::initializer_list<T> &coefficients)
      : coefficients(coefficients.begin(), coefficients.end()) {}

  /*!
   * \brief Creates a polynomial for the given coefficient: p(x) = c
   * \param c the coefficient of the polynomial
   */
  Polynomial(const T c) : coefficients(1, c) {}

  /*!
   * \brief Creates a polynomial for the given coefficients
   *
   * \param coefficients the coefficients of the polynomial
   */
  Polynomial(CoefficientList &&coefficients)
      : coefficients(std::move(coefficients)) {}

  /*!
   * \brief Creates a polynomial for the given coefficients
   *
   * \param coefficients the coefficients of the polynomial
   */
  Polynomial(const T coefficients[], PolynomialDegree degree)
      : coefficients(coefficients, coefficients + degree + 1) {}

  /*!
   * \brief Creates a (dynamic) polynomial from every other polynomial.
   * \param other_polynom the source polynomial
   */
  template <PolynomialDegree DEGREE>
  Polynomial(const Polynomial<DEGREE, T> &other_polynom)
      : Polynomial(other_polynom.getCoefficientsList().begin(),
                   other_polynom.getCoefficientsList().end()) {}

  T evaluate(const T x) const {
    return coefficients.empty() ? 0 : polynomial_detail::evaluate(coefficients, x);
  }

  std::vector<T> evaluate(const std::vector<T> &x_vector) const {
    return polynomial_detail::PolynomialBase<PolynomialDegrees::Dynamic, T>::evaluate(x_vector);
  }

  /**
   * @brief Multiplication of two polynomials
   *
   * @param factor the polynomial to multiply with.
   * @return product polynomial
   */
  Polynomial<PolynomialDegrees::Dynamic, T> operator*(
      const Polynomial<PolynomialDegrees::Dynamic, T> &factor) const {
    if (coefficients.empty() || factor.getCoefficientsList().empty()) {
      return Polynomial<PolynomialDegrees::Dynamic, T>();
    }
    const size_t number_of_coefficients =
        (coefficients.size() - 1 + factor.coefficients.size() - 1) + 1;
    CoefficientList product_coefficients(number_of_coefficients);
    polynomial_detail::multiply(product_coefficients, coefficients, factor.coefficients);
    return Polynomial<PolynomialDegrees::Dynamic, T>(std::move(product_coefficients));
  }

  /**
   * @brief Sum of two polynomials
   *
   * @param summand the polynomial to add.
   * @return sum polynomial
   */
  Polynomial<PolynomialDegrees::Dynamic, T> operator+(
      const Polynomial<PolynomialDegrees::Dynamic, T> &summand) const {
    const size_t number_of_coefficients =
        std::max(coefficients.size(), summand.coefficients.size());
    CoefficientList sum_coefficients(number_of_coefficients);
    polynomial_detail::add(sum_coefficients, coefficients, summand.coefficients);
    return Polynomial<PolynomialDegrees::Dynamic, T>(std::move(sum_coefficients));
  }

  /**
   * @brief Difference of two polynomials
   *
   * @param subtrahend the polynomial to subtract.
   * @return difference polynomial
   */
  Polynomial<PolynomialDegrees::Dynamic, T> operator-(
      const Polynomial<PolynomialDegrees::Dynamic, T> &subtrahend) const {
    const size_t number_of_coefficients =
        std::max(coefficients.size(), subtrahend.coefficients.size());
    CoefficientList difference_coefficients(number_of_coefficients);
    polynomial_detail::minus(difference_coefficients, coefficients, subtrahend.coefficients);
    return Polynomial<PolynomialDegrees::Dynamic, T>(std::move(difference_coefficients));
  }

  /*!
   * \brief operator - Flips signe of every coefficient.
   * \return the negated polynomial.
   */
  Polynomial<PolynomialDegrees::Dynamic, T> operator-() const {
    return *this * Polynomial<PolynomialDegrees::Dynamic, T>(-1.0);
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
    return polynomial_detail::calculateReductionError(coefficients, degree);
  }

  /**
   * @brief getCoefficients
   *
   * The coefficients are returned in reverse order. For example
   * the polynomial 8x³ + 9x² + 10x + 7 returns [7, 10, 9, 8].
   *
   * @return A vector of the coefficients
   */
  const std::vector<T> &getCoefficients() const { return coefficients; }

  const CoefficientList &getCoefficientsList() const { return coefficients; }

  template <InfinitesimalCount COUNT>
  Polynomial<PolynomialDegrees::Dynamic, T> calculateDerivative() const {
    Derivative<COUNT> derivative(this);
    if (derivative.isZeroPolynominal()) {
      return Polynomial<PolynomialDegrees::Dynamic, T>();
    }
    CoefficientList coefficients(this->coefficients.end() - derivative.coefficients_count,
                                 this->coefficients.end());
    polynomial_detail::derive<COUNT>(coefficients);
    return Polynomial<PolynomialDegrees::Dynamic, T>(std::move(coefficients));
  }

  template <InfinitesimalCount COUNT>
  Polynomial<PolynomialDegrees::Dynamic, T> calculateIntegral() const {
    if (this->coefficients.empty()) {
      return Polynomial<PolynomialDegrees::Dynamic, T>();
    }
    CoefficientList coefficients(this->coefficients.size() + COUNT, 0.0);
    std::copy(this->coefficients.begin(), this->coefficients.end(), coefficients.begin());
    polynomial_detail::integrate<COUNT>(coefficients);
    return Polynomial<PolynomialDegrees::Dynamic, T>(std::move(coefficients));
  }


  bool operator==(const Polynomial<PolynomialDegrees::Dynamic, T> &other_polynomial) const {
    return coefficients == other_polynomial.coefficients;
  }

  bool operator!=(const Polynomial<PolynomialDegrees::Dynamic, T> &other_polynomial) const {
    return !(*this == other_polynomial);
  }

  std::ostream &appendToStream(std::ostream &stream) const {
    PolynomialDegree degree = PolynomialDegree(coefficients.size()) - 1;
    if (degree == PolynomialDegrees::MinusInfinity) {
      stream << "0";
    }
    for (typename std::vector<T>::const_reverse_iterator it = coefficients.rbegin();
         it != coefficients.rend();
         ++it) {
      if (degree != 0) {
        stream << *it << " x^" << degree << " + ";
      } else {
        stream << coefficients.front();
      }
      --degree;
    }
    return stream;
  }

  /*!
   * \brief operator + adds a constant to a polynomial. This is needed because
   * impicit type conversion,
   * did not work in this context.
   * \param polynomial the polynomial to add.
   * \param d the constant to add to polynomial
   * \return polynomial + d
   */
  template <PolynomialDegree MAX_DEGREE>
  friend Polynomial<PolynomialDegrees::Dynamic, T> operator+(
      const Polynomial<MAX_DEGREE, T> &polynomial, const T d) {
    return polynomial + Polynomial<PolynomialDegrees::Dynamic, T>(d);
  }

  /*!
   * \brief operator + is needed to achieve symetrical behaviour.
   * \param d the constant to add to.
   * \param polynomial the polynmial to add.
   * \return polynomial + d
   */
  template <PolynomialDegree MAX_DEGREE>
  friend Polynomial<PolynomialDegrees::Dynamic, T> operator+(
      const T d, const Polynomial<MAX_DEGREE, T> &polynomial) {
    return polynomial + d;
  }

  /*!
   * \brief operator - subtrackts a constant from a polynomial. This is needed
   * because impicit type conversion,
   * did not work in this context.
   * \param polynomial the polynomial to subtract from
   * \param d the constant to add to polynomial
   * \return polynomial - d
   */
  template <PolynomialDegree MAX_DEGREE>
  friend Polynomial<PolynomialDegrees::Dynamic, T> operator-(
      const Polynomial<MAX_DEGREE, T> &polynomial, const T d) {
    return polynomial - Polynomial<PolynomialDegrees::Dynamic, T>(d);
  }

  /*!
   * \brief operator - subtrackts a polynomial from a constant.
   * \param d the constant to subtract from.
   * \param polynomial the polynomial to subtract
   * \return d - polynomial
   */
  template <PolynomialDegree MAX_DEGREE>
  friend Polynomial<PolynomialDegrees::Dynamic, T> operator-(
      const T d, const Polynomial<MAX_DEGREE, T> &polynomial) {
    return Polynomial<PolynomialDegrees::Dynamic, T>(d) - polynomial;
  }

 protected:
  CoefficientList coefficients;
};

/*!
 * \brief The Polynomial<PolynomialDegrees::MinusInfinity, T> class the
 * implementation for zero polynomial.
 *
 * These polynomials have no allocations at all. This is basically an edge case.
 */
template <typename T>
class Polynomial<PolynomialDegrees::MinusInfinity, T>
    : public polynomial_detail::PolynomialBase<PolynomialDegrees::MinusInfinity, T> {
 public:
  typedef std::array<T, 0> CoefficientList;
  static const PolynomialDegree TECHNICAL_DEGREE = PolynomialDegrees::MinusInfinity;

  /*!
   * \brief Creates a zero polynomial
   * Represents the constant function zero.
   */
  Polynomial() {}

  /*!
   * \brief Creates a zero polynomial
   *
   * Parameter is ignored
   */
  Polynomial(const std::vector<T> &) {}

  /*!
   * \brief Creates a polynomial for the given coefficients
   *
   * Parameter is ignored
   */
  Polynomial(const std::array<T, 0> &) {}

  T evaluate(const T) const { return 0; }

  std::vector<T> evaluate(const std::vector<T> &x_vector) const {
    return polynomial_detail::PolynomialBase<PolynomialDegrees::MinusInfinity, T>::evaluate(x_vector);
  }

  /**
   * @brief Multiplication of two polynomials
   * \return this will always return an MinusInfinity-Polynomial.
   */
  template <PolynomialDegree FACTOR_MAX_DEGREE>
  Polynomial<PolynomialDegrees::MinusInfinity, T> operator*(
      const Polynomial<FACTOR_MAX_DEGREE, T> &) const {
    return *this;
  }

  /**
   * @brief Sum of two polynomials
   *
   * @param summand the polynmial to add.
   * @return sum polynomial
   */
  template <PolynomialDegree SUMMAND_MAX_DEGREE>
  Polynomial<SUMMAND_MAX_DEGREE, T> operator+(const Polynomial<SUMMAND_MAX_DEGREE, T> &summand) const {
    return summand;
  }

  /**
   * @brief Difference of two polynomials
   *
   * @param subtrahend the polynomial to subtract.
   * @return difference polynomial
   */
  template <PolynomialDegree SUBTRAHEND_MAX_DEGREE>
  Polynomial<SUBTRAHEND_MAX_DEGREE, T> operator-(
      const Polynomial<SUBTRAHEND_MAX_DEGREE, T> &subtrahend) const {
    typename Polynomial<SUBTRAHEND_MAX_DEGREE, T>::CoefficientList difference_coefficients = {};
    polynomial_detail::minus(
        difference_coefficients, getCoefficientsList(), subtrahend.getCoefficientsList());
    return Polynomial<SUBTRAHEND_MAX_DEGREE, T>(difference_coefficients);
  }

  /**
   * @brief Calculates the Reduction Error
   *
   * Calculates a measure for the error that would occour
   * if the polynomial was reduced to the specified degree.
   *
   * Note: For zero polynomial the funtion always returns 0.
   *
   * @return Always returns 0
   */
  T calculateReductionError(const PolynomialDegree) const { return 0; }

  /**
   * @brief getCoefficients
   *
   * The coefficients are returned in reverse order.
   *
   * @return An empty vector
   */
  std::vector<T> getCoefficients() const { return {}; }

  CoefficientList getCoefficientsList() const { return CoefficientList(); }

  template <InfinitesimalCount COUNT>
  Polynomial<PolynomialDegrees::MinusInfinity, T> calculateDerivative() const {
    return *this;
  }

  template <InfinitesimalCount COUNT>
  Polynomial<PolynomialDegrees::MinusInfinity, T> calculateIntegral() const {
    return *this;
  }

  bool operator==(const Polynomial<PolynomialDegrees::MinusInfinity, T> &) const {
    return true;
  }

  bool operator!=(const Polynomial<PolynomialDegrees::MinusInfinity, T> &) const {
    return false;
  }

  std::ostream &appendToStream(std::ostream &stream) const {
    stream << "0";
    return stream;
  }
};

template <PolynomialDegree DEGREE, typename T>
std::ostream &operator<<(std::ostream &stream, const Polynomial<DEGREE, T> &polynomial) {
  stream << "Polynomial<MAX_DEGREE=" << DEGREE << ">: ";
  polynomial.appendToStream(stream);
  return stream;
}

typedef Polynomial<PolynomialDegrees::MinusInfinity, double> ZeroPolynomiald;
typedef Polynomial<PolynomialDegrees::Constant, double> ConstantPolynomiald;
typedef Polynomial<PolynomialDegrees::Linear, double> LinearPolynomiald;
typedef Polynomial<PolynomialDegrees::Quadratic, double> QuadraticPolynomiald;
typedef Polynomial<PolynomialDegrees::Cubic, double> CubicPolynomiald;
typedef Polynomial<PolynomialDegrees::Quartic, double> QuarticPolynomiald;
typedef Polynomial<PolynomialDegrees::Quintic, double> QuinticPolynomiald;
typedef Polynomial<PolynomialDegrees::Sextic, double> SexticPolynomiald;
typedef Polynomial<PolynomialDegrees::Septic, double> SepticPolynomiald;
typedef Polynomial<PolynomialDegrees::Dynamic, double> DynamicPolynomiald;

typedef ZeroPolynomiald ZeroPolynomial;
typedef ConstantPolynomiald ConstantPolynomial;
typedef LinearPolynomiald LinearPolynomial;
typedef QuadraticPolynomiald QuadraticPolynomial;
typedef CubicPolynomiald CubicPolynomial;
typedef QuarticPolynomiald QuarticPolynomial;
typedef QuinticPolynomiald QuinticPolynomial;
typedef SexticPolynomiald SexticPolynomial;
typedef SepticPolynomiald SepticPolynomial;
typedef DynamicPolynomiald DynamicPolynomial;

typedef Polynomial<PolynomialDegrees::MinusInfinity, float> ZeroPolynomialf;
typedef Polynomial<PolynomialDegrees::Constant, float> ConstantPolynomialf;
typedef Polynomial<PolynomialDegrees::Linear, float> LinearPolynomialf;
typedef Polynomial<PolynomialDegrees::Quadratic, float> QuadraticPolynomialf;
typedef Polynomial<PolynomialDegrees::Cubic, float> CubicPolynomialf;
typedef Polynomial<PolynomialDegrees::Quartic, float> QuarticPolynomialf;
typedef Polynomial<PolynomialDegrees::Quintic, float> QuinticPolynomialf;
typedef Polynomial<PolynomialDegrees::Sextic, float> SexticPolynomialf;
typedef Polynomial<PolynomialDegrees::Septic, float> SepticPolynomialf;
typedef Polynomial<PolynomialDegrees::Dynamic, float> DynamicPolynomialf;


template <class POLYNOM>
POLYNOM createMonomial(const long double c) {
  typename POLYNOM::CoefficientList coefficients = {};
  coefficients.back() = static_cast<double>(c);
  return POLYNOM(coefficients);
}

// C++11 allows only long double as floating point type.

inline LinearPolynomial operator"" _x(const long double c) {
  return createMonomial<LinearPolynomial>(c);
}

inline QuadraticPolynomial operator"" _x_2(const long double c) {
  return createMonomial<QuadraticPolynomial>(c);
}

inline CubicPolynomial operator"" _x_3(const long double c) {
  return createMonomial<CubicPolynomial>(c);
}

inline QuarticPolynomial operator"" _x_4(const long double c) {
  return createMonomial<QuarticPolynomial>(c);
}

inline QuinticPolynomial operator"" _x_5(const long double c) {
  return createMonomial<QuinticPolynomial>(c);
}

inline SexticPolynomial operator"" _x_6(const long double c) {
  return createMonomial<SexticPolynomial>(c);
}

inline SepticPolynomial operator"" _x_7(const long double c) {
  return createMonomial<SepticPolynomial>(c);
}
}  // namespace polynmial

using namespace polynomial;

}  // namespace common

#endif  // KITCAR_POLYNOMIAL_H
