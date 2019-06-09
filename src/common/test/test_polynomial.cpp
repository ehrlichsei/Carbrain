#include "common/polynomial.h"
#include "common/macros.h"

THIRD_PARTY_HEADERS_BEGIN
#include <gtest/gtest.h>
#include <boost/range/algorithm/equal.hpp>
THIRD_PARTY_HEADERS_END

#include "common/math.h"

DISABLE_SUGGEST_OVERRIDE_WARNING

using namespace common;

namespace {

class PolynomialTest : public ::testing::Test {
 protected:
  template <PolynomialDegree MAX_DEGREE>
  Polynomial<MAX_DEGREE> createPolynomial() {
    static_assert(MAX_DEGREE != PolynomialDegrees::Dynamic, "");
    return ZeroPolynomial();
  }

  template <PolynomialDegree DEGREE>
  DynamicPolynomial createDynamicPolynomial() {
    static_assert(DEGREE != PolynomialDegrees::Dynamic, "");
    return DynamicPolynomial();
  }
};

template <>
QuarticPolynomial PolynomialTest::createPolynomial<PolynomialDegrees::Quartic>() {
  return {11, 7, 5, 3, 2};
}

template <>
CubicPolynomial PolynomialTest::createPolynomial<PolynomialDegrees::Cubic>() {
  return {7 * 1, 5 * 2, 3 * 3, 2 * 4};
}

template <>
QuadraticPolynomial PolynomialTest::createPolynomial<PolynomialDegrees::Quadratic>() {
  return {5 * 2 * 1, 3 * 3 * 2, 2 * 4 * 3};
}

template <>
LinearPolynomial PolynomialTest::createPolynomial<PolynomialDegrees::Linear>() {
  return {3 * 3 * 2 * 1, 2 * 4 * 3 * 2};
}

template <>
ConstantPolynomial PolynomialTest::createPolynomial<PolynomialDegrees::Constant>() {
  return {2 * 4 * 3 * 2 * 1};
}

template <>
ZeroPolynomial PolynomialTest::createPolynomial<PolynomialDegrees::MinusInfinity>() {
  return ZeroPolynomial();
}

template <>
DynamicPolynomial PolynomialTest::createDynamicPolynomial<PolynomialDegrees::Quartic>() {
  return {11, 7, 5, 3, 2};
}

template <>
DynamicPolynomial PolynomialTest::createDynamicPolynomial<PolynomialDegrees::Cubic>() {
  return {7 * 1, 5 * 2, 3 * 3, 2 * 4};
}

template <>
DynamicPolynomial PolynomialTest::createDynamicPolynomial<PolynomialDegrees::Quadratic>() {
  return {5 * 2 * 1, 3 * 3 * 2, 2 * 4 * 3};
}

template <>
DynamicPolynomial PolynomialTest::createDynamicPolynomial<PolynomialDegrees::Linear>() {
  return {3 * 3 * 2 * 1, 2 * 4 * 3 * 2};
}

template <>
DynamicPolynomial PolynomialTest::createDynamicPolynomial<PolynomialDegrees::Constant>() {
  return {2 * 4 * 3 * 2 * 1};
}

template <>
DynamicPolynomial PolynomialTest::createDynamicPolynomial<PolynomialDegrees::MinusInfinity>() {
  return {};
}

// CubicPolynomial(fixed size)

TEST_F(PolynomialTest, CubicPolynomialIdentity) {
  CubicPolynomial polynomial = createPolynomial<PolynomialDegrees::Cubic>();
  CubicPolynomial derivative = polynomial.calculateDerivative<0>();
  CubicPolynomial expected = polynomial;
  ASSERT_EQ(expected, derivative);
}

TEST_F(PolynomialTest, CubicPolynomialFirstDerivative) {
  CubicPolynomial polynomial = createPolynomial<PolynomialDegrees::Cubic>();
  QuadraticPolynomial derivative = polynomial.calculateFirstDerivative();
  QuadraticPolynomial expected = createPolynomial<PolynomialDegrees::Quadratic>();
  ASSERT_EQ(expected, derivative);
}

TEST_F(PolynomialTest, CubicPolynomialSecondDerivative) {
  CubicPolynomial polynomial = createPolynomial<PolynomialDegrees::Cubic>();
  LinearPolynomial derivative = polynomial.calculateSecondDerivative();
  LinearPolynomial expected = createPolynomial<PolynomialDegrees::Linear>();
  ASSERT_EQ(expected, derivative);
}

TEST_F(PolynomialTest, CubicPolynomialDeriveTwice) {
  CubicPolynomial polynomial = createPolynomial<PolynomialDegrees::Cubic>();
  QuadraticPolynomial first_derivative = polynomial.calculateFirstDerivative();
  LinearPolynomial derivative = first_derivative.calculateFirstDerivative();
  LinearPolynomial expected = createPolynomial<PolynomialDegrees::Linear>();
  ASSERT_EQ(expected, derivative);
}

TEST_F(PolynomialTest, CubicPolynomialThirdDerivative) {
  CubicPolynomial polynomial = createPolynomial<PolynomialDegrees::Cubic>();
  ConstantPolynomial derivative = polynomial.calculateThirdDerivative();
  ConstantPolynomial expected = createPolynomial<PolynomialDegrees::Constant>();
  ASSERT_EQ(expected, derivative);
}

TEST_F(PolynomialTest, CubicPolynomialDeriveTrice) {
  CubicPolynomial polynomial = createPolynomial<PolynomialDegrees::Cubic>();
  QuadraticPolynomial first_derivative = polynomial.calculateFirstDerivative();
  LinearPolynomial second_derivative = first_derivative.calculateFirstDerivative();
  ConstantPolynomial derivative = second_derivative.calculateFirstDerivative();
  ConstantPolynomial expected = createPolynomial<PolynomialDegrees::Constant>();
  ASSERT_EQ(expected, derivative);
}

TEST_F(PolynomialTest, CubicPolynomialFourthDerivative) {
  CubicPolynomial polynomial = createPolynomial<PolynomialDegrees::Cubic>();
  ZeroPolynomial derivative = polynomial.calculateDerivative<4>();
  ZeroPolynomial expected = createPolynomial<PolynomialDegrees::MinusInfinity>();
  ASSERT_EQ(expected, derivative);
}

TEST_F(PolynomialTest, CubicPolynomialSecondDerivativeTwice) {
  CubicPolynomial polynomial = createPolynomial<PolynomialDegrees::Cubic>();
  LinearPolynomial second_derivative = polynomial.calculateSecondDerivative();
  ZeroPolynomial derivative = second_derivative.calculateSecondDerivative();
  ZeroPolynomial expected;
  ASSERT_EQ(expected, derivative);
}

TEST_F(PolynomialTest, CubicPolynomialFifthDerivative) {
  CubicPolynomial polynomial = createPolynomial<PolynomialDegrees::Cubic>();
  ZeroPolynomial derivative = polynomial.calculateDerivative<5>();
  ZeroPolynomial expected;
  ASSERT_EQ(expected, derivative);
}

// LinearPolynomial(fixed size)

TEST_F(PolynomialTest, LinearPolynomialFirstDerivative) {
  LinearPolynomial polynomial = createPolynomial<PolynomialDegrees::Linear>();
  ConstantPolynomial derivative = polynomial.calculateFirstDerivative();
  ConstantPolynomial expected = createPolynomial<PolynomialDegrees::Constant>();
  ASSERT_EQ(expected, derivative);
}

TEST_F(PolynomialTest, LinearPolynomialSecondDerivative) {
  LinearPolynomial polynomial = createPolynomial<PolynomialDegrees::Linear>();
  ZeroPolynomial derivative = polynomial.calculateSecondDerivative();
  ZeroPolynomial expected;
  ASSERT_EQ(expected, derivative);
}

// ConstantPolynomial(fixed size)

TEST_F(PolynomialTest, ConstantPolynomialFirstDerivative) {
  ConstantPolynomial polynomial = createPolynomial<PolynomialDegrees::Constant>();
  ZeroPolynomial derivative = polynomial.calculateFirstDerivative();
  ZeroPolynomial expected;
  ASSERT_EQ(expected, derivative);
}

TEST_F(PolynomialTest, ConstantPolynomialSecondDerivative) {
  ConstantPolynomial polynomial = createPolynomial<PolynomialDegrees::Constant>();
  ZeroPolynomial derivative = polynomial.calculateSecondDerivative();
  ZeroPolynomial expected;
  ASSERT_EQ(expected, derivative);
}

// ZeroPolynomial(fixed size)

TEST_F(PolynomialTest, ZeroPolynomialFirstDerivative) {
  ZeroPolynomial polynomial = createPolynomial<PolynomialDegrees::MinusInfinity>();
  ZeroPolynomial derivative = polynomial.calculateFirstDerivative();
  ZeroPolynomial expected;
  ASSERT_EQ(expected, derivative);
}

TEST_F(PolynomialTest, ZeroPolynomialSecondDerivative) {
  ZeroPolynomial polynomial = createPolynomial<PolynomialDegrees::MinusInfinity>();
  ZeroPolynomial derivative = polynomial.calculateSecondDerivative();
  ZeroPolynomial expected;
  ASSERT_EQ(expected, derivative);
}

// CubicPolynomial(dynamic)

TEST_F(PolynomialTest, DynamicCubicPolynomialIdentity) {
  DynamicPolynomial polynomial = createDynamicPolynomial<PolynomialDegrees::Cubic>();
  DynamicPolynomial derivative = polynomial.calculateDerivative<0>();
  DynamicPolynomial expected = polynomial;
  ASSERT_EQ(expected, derivative);
}

TEST_F(PolynomialTest, DynamicCubicPolynomialFirstDerivative) {
  DynamicPolynomial polynomial = createDynamicPolynomial<PolynomialDegrees::Cubic>();
  DynamicPolynomial derivative = polynomial.calculateFirstDerivative();
  DynamicPolynomial expected = createDynamicPolynomial<PolynomialDegrees::Quadratic>();
  ASSERT_EQ(expected, derivative);
}

TEST_F(PolynomialTest, DynamicCubicPolynomialSecondDerivative) {
  DynamicPolynomial polynomial = createDynamicPolynomial<PolynomialDegrees::Cubic>();
  DynamicPolynomial derivative = polynomial.calculateSecondDerivative();
  DynamicPolynomial expected = createDynamicPolynomial<PolynomialDegrees::Linear>();
  ASSERT_EQ(expected, derivative);
}

TEST_F(PolynomialTest, DynamicCubicPolynomialDeriveTwice) {
  DynamicPolynomial polynomial = createDynamicPolynomial<PolynomialDegrees::Cubic>();
  DynamicPolynomial first_derivative = polynomial.calculateFirstDerivative();
  DynamicPolynomial derivative = first_derivative.calculateFirstDerivative();
  DynamicPolynomial expected = createDynamicPolynomial<PolynomialDegrees::Linear>();
  ASSERT_EQ(expected, derivative);
}

TEST_F(PolynomialTest, DynamicCubicPolynomialThirdDerivative) {
  DynamicPolynomial polynomial = createDynamicPolynomial<PolynomialDegrees::Cubic>();
  DynamicPolynomial derivative = polynomial.calculateThirdDerivative();
  DynamicPolynomial expected = createDynamicPolynomial<PolynomialDegrees::Constant>();
  ASSERT_EQ(expected, derivative);
}

TEST_F(PolynomialTest, DynamicCubicPolynomialDeriveTrice) {
  DynamicPolynomial polynomial = createDynamicPolynomial<PolynomialDegrees::Cubic>();
  DynamicPolynomial first_derivative = polynomial.calculateFirstDerivative();
  DynamicPolynomial second_derivative = first_derivative.calculateFirstDerivative();
  DynamicPolynomial derivative = second_derivative.calculateFirstDerivative();
  DynamicPolynomial expected = createDynamicPolynomial<PolynomialDegrees::Constant>();
  ASSERT_EQ(expected, derivative);
}

TEST_F(PolynomialTest, DynamicCubicPolynomialFourthDerivative) {
  DynamicPolynomial polynomial = createDynamicPolynomial<PolynomialDegrees::Cubic>();
  DynamicPolynomial derivative = polynomial.calculateDerivative<4>();
  DynamicPolynomial expected =
      createDynamicPolynomial<PolynomialDegrees::MinusInfinity>();
  ASSERT_EQ(expected, derivative);
}

TEST_F(PolynomialTest, DynamicCubicPolynomialSecondDerivativeTwice) {
  DynamicPolynomial polynomial = createDynamicPolynomial<PolynomialDegrees::Cubic>();
  DynamicPolynomial second_derivative = polynomial.calculateSecondDerivative();
  DynamicPolynomial derivative = second_derivative.calculateSecondDerivative();
  DynamicPolynomial expected;
  ASSERT_EQ(expected, derivative);
}

TEST_F(PolynomialTest, DynamicCubicPolynomialFifthDerivative) {
  DynamicPolynomial polynomial = createDynamicPolynomial<PolynomialDegrees::Cubic>();
  DynamicPolynomial derivative = polynomial.calculateDerivative<5>();
  DynamicPolynomial expected;
  ASSERT_EQ(expected, derivative);
}

// LinearPolynomial(dynamic)

TEST_F(PolynomialTest, DynamicLinearPolynomialFirstDerivative) {
  DynamicPolynomial polynomial = createDynamicPolynomial<PolynomialDegrees::Linear>();
  DynamicPolynomial derivative = polynomial.calculateFirstDerivative();
  DynamicPolynomial expected = createDynamicPolynomial<PolynomialDegrees::Constant>();
  ASSERT_EQ(expected, derivative);
}

TEST_F(PolynomialTest, DynamicLinearPolynomialSecondDerivative) {
  DynamicPolynomial polynomial = createDynamicPolynomial<PolynomialDegrees::Linear>();
  DynamicPolynomial derivative = polynomial.calculateSecondDerivative();
  DynamicPolynomial expected;
  ASSERT_EQ(expected, derivative);
}

// ConstantPolynomial(dynamic)

TEST_F(PolynomialTest, DynamicConstantPolynomialFirstDerivative) {
  DynamicPolynomial polynomial = createDynamicPolynomial<PolynomialDegrees::Constant>();
  DynamicPolynomial derivative = polynomial.calculateFirstDerivative();
  DynamicPolynomial expected;
  ASSERT_EQ(expected, derivative);
}

TEST_F(PolynomialTest, DynamicConstantPolynomialSecondDerivative) {
  DynamicPolynomial polynomial = createDynamicPolynomial<PolynomialDegrees::Constant>();
  DynamicPolynomial derivative = polynomial.calculateSecondDerivative();
  DynamicPolynomial expected;
  ASSERT_EQ(expected, derivative);
}

// ZeroPolynomial(dynamic)

TEST_F(PolynomialTest, DynamicZeroPolynomialFirstDerivative) {
  DynamicPolynomial polynomial =
      createDynamicPolynomial<PolynomialDegrees::MinusInfinity>();
  DynamicPolynomial derivative = polynomial.calculateFirstDerivative();
  DynamicPolynomial expected;
  ASSERT_EQ(expected, derivative);
}

TEST_F(PolynomialTest, DynamicZeroPolynomialSecondDerivative) {
  DynamicPolynomial polynomial =
      createDynamicPolynomial<PolynomialDegrees::MinusInfinity>();
  DynamicPolynomial derivative = polynomial.calculateSecondDerivative();
  DynamicPolynomial expected;
  ASSERT_EQ(expected, derivative);
}

// Evaluation

TEST_F(PolynomialTest, CubicPolynomialEvaluation) {
  // 8x³ + 9x² + 10x + 7
  CubicPolynomial polynomial = createPolynomial<PolynomialDegrees::Cubic>();
  ASSERT_EQ(7.0, polynomial.evaluate(0));
  ASSERT_EQ(8.0 + 9.0 + 10 + 7, polynomial.evaluate(1));
  ASSERT_EQ(8.0 * 8 + 9.0 * 4 + 10 * 2 + 7, polynomial.evaluate(2));
  ASSERT_EQ(8.0 * 27 + 9.0 * 9 + 10 * 3 + 7, polynomial.evaluate(3));
  ASSERT_EQ(-8.0 + 9.0 - 10 + 7, polynomial.evaluate(-1));
  ASSERT_EQ(-8.0 * 8 + 9.0 * 4 - 10 * 2 + 7, polynomial.evaluate(-2));
  ASSERT_EQ(-8.0 * 27 + 9.0 * 9 - 10 * 3 + 7, polynomial.evaluate(-3));
}

TEST_F(PolynomialTest, ConstantPolynomialEvaluation) {
  // 48
  ConstantPolynomial polynomial = createPolynomial<PolynomialDegrees::Constant>();
  ASSERT_EQ(48, polynomial.evaluate(0));
  ASSERT_EQ(48, polynomial.evaluate(1));
  ASSERT_EQ(48, polynomial.evaluate(-1));
}

TEST_F(PolynomialTest, ZeroPolynomialEvaluation) {
  // 0
  ZeroPolynomial polynomial = createPolynomial<PolynomialDegrees::MinusInfinity>();
  ASSERT_EQ(0, polynomial.evaluate(0));
  ASSERT_EQ(0, polynomial.evaluate(1));
  ASSERT_EQ(0, polynomial.evaluate(-1));
}

TEST_F(PolynomialTest, DynamicZeroPolynomialEvaluation) {
  // 0
  DynamicPolynomial polynomial =
      createDynamicPolynomial<PolynomialDegrees::MinusInfinity>();
  ASSERT_EQ(0, polynomial.evaluate(0));
  ASSERT_EQ(0, polynomial.evaluate(1));
  ASSERT_EQ(0, polynomial.evaluate(-1));
}

// Vector Evaluation

TEST_F(PolynomialTest, CubicPolynomialVectorEvaluation) {
  // 8x³ + 9x² + 10x + 7
  CubicPolynomial polynomial = createPolynomial<PolynomialDegrees::Cubic>();

  const std::vector<double> x_vector = {0, 1, 2, 3, -1, -2, -3};

  const std::vector<double> result_vector = polynomial.evaluate(x_vector);

  ASSERT_EQ(x_vector.size(), result_vector.size());
  ASSERT_EQ(7.0, result_vector.at(0));
  ASSERT_EQ(8.0 + 9.0 + 10 + 7, result_vector.at(1));
  ASSERT_EQ(8.0 * 8 + 9.0 * 4 + 10 * 2 + 7, result_vector.at(2));
  ASSERT_EQ(8.0 * 27 + 9.0 * 9 + 10 * 3 + 7, result_vector.at(3));
  ASSERT_EQ(-8.0 + 9.0 - 10 + 7, result_vector.at(4));
  ASSERT_EQ(-8.0 * 8 + 9.0 * 4 - 10 * 2 + 7, result_vector.at(5));
  ASSERT_EQ(-8.0 * 27 + 9.0 * 9 - 10 * 3 + 7, result_vector.at(6));
}

TEST_F(PolynomialTest, DynamicPolynomialVectorEvaluation) {
  // 8x³ + 9x² + 10x + 7
  const DynamicPolynomial polynomial = {7, 10, 9, 8};

  const std::vector<double> x_vector = {0, 1, 2, 3, -1, -2, -3};

  const std::vector<double> result_vector = polynomial.evaluate(x_vector);

  ASSERT_EQ(x_vector.size(), result_vector.size());
  ASSERT_EQ(7.0, result_vector.at(0));
  ASSERT_EQ(8.0 + 9.0 + 10 + 7, result_vector.at(1));
  ASSERT_EQ(8.0 * 8 + 9.0 * 4 + 10 * 2 + 7, result_vector.at(2));
  ASSERT_EQ(8.0 * 27 + 9.0 * 9 + 10 * 3 + 7, result_vector.at(3));
  ASSERT_EQ(-8.0 + 9.0 - 10 + 7, result_vector.at(4));
  ASSERT_EQ(-8.0 * 8 + 9.0 * 4 - 10 * 2 + 7, result_vector.at(5));
  ASSERT_EQ(-8.0 * 27 + 9.0 * 9 - 10 * 3 + 7, result_vector.at(6));
}

// Construction

TEST_F(PolynomialTest, CubicPolynomialVectorConstruction) {
  const CubicPolynomial polynomial = {42};
  ASSERT_EQ(42, polynomial.evaluate(0));
  ASSERT_EQ(42, polynomial.evaluate(1));
  ASSERT_EQ(42, polynomial.evaluate(-1));
}

TEST_F(PolynomialTest, CubicPolynomialArrayConstruction) {
  const CubicPolynomial polynomial = {42};
  ASSERT_EQ(42, polynomial.evaluate(0));
  ASSERT_EQ(42, polynomial.evaluate(1));
  ASSERT_EQ(42, polynomial.evaluate(-1));
}

// Usage Examples

TEST_F(PolynomialTest, CubicPolynomialUsage) {
  // 8x³ + 9x² + 10x + 7
  const CubicPolynomial polynomial = {7, 10, 9, 8};
  ASSERT_EQ(7.0, polynomial.evaluate(0));
  ASSERT_EQ(8.0 + 9.0 + 10 + 7, polynomial.evaluate(1));
  ASSERT_EQ(8.0 * 8 + 9.0 * 4 + 10 * 2 + 7, polynomial.evaluate(2));
  ASSERT_EQ(8.0 * 27 + 9.0 * 9 + 10 * 3 + 7, polynomial.evaluate(3));
  ASSERT_EQ(-8.0 + 9.0 - 10 + 7, polynomial.evaluate(-1));
  ASSERT_EQ(-8.0 * 8 + 9.0 * 4 - 10 * 2 + 7, polynomial.evaluate(-2));
  ASSERT_EQ(-8.0 * 27 + 9.0 * 9 - 10 * 3 + 7, polynomial.evaluate(-3));
}

// Operators

TEST_F(PolynomialTest, CubicPolynomialFunctionCall) {
  // 8x³ + 9x² + 10x + 7
  CubicPolynomial polynomial = createPolynomial<PolynomialDegrees::Cubic>();
  ASSERT_EQ(7.0, polynomial(0));
  ASSERT_EQ(8.0 + 9.0 + 10 + 7, polynomial(1));
  ASSERT_EQ(8.0 * 8 + 9.0 * 4 + 10 * 2 + 7, polynomial(2));
  ASSERT_EQ(8.0 * 27 + 9.0 * 9 + 10 * 3 + 7, polynomial(3));
  ASSERT_EQ(-8.0 + 9.0 - 10 + 7, polynomial(-1));
  ASSERT_EQ(-8.0 * 8 + 9.0 * 4 - 10 * 2 + 7, polynomial(-2));
  ASSERT_EQ(-8.0 * 27 + 9.0 * 9 - 10 * 3 + 7, polynomial(-3));
}

TEST_F(PolynomialTest, ConstantPolynomialFunctionCall) {
  // 48
  ConstantPolynomial polynomial = createPolynomial<PolynomialDegrees::Constant>();
  ASSERT_EQ(48, polynomial(0));
  ASSERT_EQ(48, polynomial(1));
  ASSERT_EQ(48, polynomial(-1));
}

TEST_F(PolynomialTest, ZeroPolynomialFunctionCall) {
  // 0
  ZeroPolynomial polynomial = createPolynomial<PolynomialDegrees::MinusInfinity>();
  ASSERT_EQ(0, polynomial(0));
  ASSERT_EQ(0, polynomial(1));
  ASSERT_EQ(0, polynomial(-1));
}

TEST_F(PolynomialTest, DynamicZeroPolynomialFunctionCall) {
  // 0
  DynamicPolynomial polynomial =
      createDynamicPolynomial<PolynomialDegrees::MinusInfinity>();
  ASSERT_EQ(0, polynomial(0));
  ASSERT_EQ(0, polynomial(1));
  ASSERT_EQ(0, polynomial(-1));
}

TEST_F(PolynomialTest, ZeroPolynomialMultipliedWithZeroPolynomial) {
  ZeroPolynomial first_factor = createPolynomial<PolynomialDegrees::MinusInfinity>();
  ZeroPolynomial second_factor = createPolynomial<PolynomialDegrees::MinusInfinity>();
  ZeroPolynomial product = first_factor * second_factor;
  ZeroPolynomial expected;
  ASSERT_EQ(expected, product);
}

TEST_F(PolynomialTest, ZeroPolynomialMultipliedWithConstantPolynomial) {
  ZeroPolynomial first_factor = createPolynomial<PolynomialDegrees::MinusInfinity>();
  ConstantPolynomial second_factor = createPolynomial<PolynomialDegrees::Constant>();
  ZeroPolynomial product = first_factor * second_factor;
  ZeroPolynomial expected;
  ASSERT_EQ(expected, product);
}

TEST_F(PolynomialTest, ConstantPolynomialMultipliedWithZeroPolynomial) {
  ConstantPolynomial first_factor = createPolynomial<PolynomialDegrees::Constant>();
  ZeroPolynomial second_factor = createPolynomial<PolynomialDegrees::MinusInfinity>();
  ZeroPolynomial product = first_factor * second_factor;
  ZeroPolynomial expected;
  ASSERT_EQ(expected, product);
}

TEST_F(PolynomialTest, ConstantPolynomialMultipliedWithConstantPolynomial) {
  const ConstantPolynomial first_factor = {4};
  const ConstantPolynomial second_factor = {5};
  const ConstantPolynomial product = first_factor * second_factor;
  const ConstantPolynomial expected = {20};
  ASSERT_EQ(expected, product);
}

TEST_F(PolynomialTest, ConstantPolynomialMultipliedWithLinearPolynomial) {
  const ConstantPolynomial first_factor = {4};
  const LinearPolynomial second_factor = {5, 3};
  // 3x + 5
  const LinearPolynomial product = first_factor * second_factor;
  // 12x + 20 = (3x + 5) * 4
  const LinearPolynomial expected = {20, 12};
  ASSERT_EQ(expected, product);
}

TEST_F(PolynomialTest, LinearPolynomialMultipliedWithConstantPolynomial) {
  // 3x + 5
  const LinearPolynomial first_factor = {5, 3};
  const ConstantPolynomial second_factor = {4};
  const LinearPolynomial product = first_factor * second_factor;
  // 12x + 20 = (3x + 5) * 4
  const LinearPolynomial expected = {20, 12};
  ASSERT_EQ(expected, product);
}

TEST_F(PolynomialTest, LinearPolynomialMultipliedWithLinearPolynomial) {
  // 3x + 5
  const LinearPolynomial first_factor = {5, 3};
  // 6x + 4
  const LinearPolynomial second_factor = {4, 6};
  const QuadraticPolynomial product = first_factor * second_factor;
  // 18x² + (12 + 30)x + 20 = (3x + 5) * (6x + 4)
  const QuadraticPolynomial expected = {20, 42, 18};
  ASSERT_EQ(expected, product);
}

TEST_F(PolynomialTest, LinearPolynomialMultipliedWithQuadraticPolynomial) {
  // 3x + 5
  const LinearPolynomial first_factor = {5, 3};
  // 2x² + 6x + 4
  const QuadraticPolynomial second_factor = {4, 6, 2};
  const CubicPolynomial product = first_factor * second_factor;
  // 6x³ + (18 + 10)x² + (12 + 30)x + 20 = (3x + 5) * (2x² + 6x + 4)
  const CubicPolynomial expected = {20, 42, 28, 6};
  ASSERT_EQ(expected, product);
}

TEST_F(PolynomialTest, DynamicZeroPolynomialMultipliedWithDynamicZeroPolynomial) {
  DynamicPolynomial first_factor =
      createDynamicPolynomial<PolynomialDegrees::MinusInfinity>();
  DynamicPolynomial second_factor =
      createDynamicPolynomial<PolynomialDegrees::MinusInfinity>();
  DynamicPolynomial product = first_factor * second_factor;
  DynamicPolynomial expected;
  ASSERT_EQ(expected, product);
}

TEST_F(PolynomialTest, DynamicZeroPolynomialMultipliedWithDynamicConstantPolynomial) {
  DynamicPolynomial first_factor =
      createDynamicPolynomial<PolynomialDegrees::MinusInfinity>();
  DynamicPolynomial second_factor =
      createDynamicPolynomial<PolynomialDegrees::Constant>();
  DynamicPolynomial product = first_factor * second_factor;
  DynamicPolynomial expected;
  ASSERT_EQ(expected, product);
}

TEST_F(PolynomialTest, DynamicConstantPolynomialMultipliedWithDynamicZeroPolynomial) {
  DynamicPolynomial first_factor =
      createDynamicPolynomial<PolynomialDegrees::Constant>();
  DynamicPolynomial second_factor =
      createDynamicPolynomial<PolynomialDegrees::MinusInfinity>();
  DynamicPolynomial product = first_factor * second_factor;
  DynamicPolynomial expected;
  ASSERT_EQ(expected, product);
}

TEST_F(PolynomialTest, DynamicConstantPolynomialMultipliedWithDynamicConstantPolynomial) {
  const DynamicPolynomial first_factor = {4};
  const DynamicPolynomial second_factor = {5};
  const DynamicPolynomial product = first_factor * second_factor;
  const DynamicPolynomial expected = {20};
  ASSERT_EQ(expected, product);
}

TEST_F(PolynomialTest, DynamicConstantPolynomialMultipliedWithDynamicLinearPolynomial) {
  const DynamicPolynomial first_factor = {4};
  // 3x + 5
  const DynamicPolynomial second_factor = {5, 3};
  const DynamicPolynomial product = first_factor * second_factor;
  // 12x + 20 = (3x + 5) * 4
  const DynamicPolynomial expected = {20, 12};
  ASSERT_EQ(expected, product);
}

TEST_F(PolynomialTest, DynamicLinearPolynomialMultipliedWithDynamicConstantPolynomial) {
  // 3x + 5
  const DynamicPolynomial first_factor = {5, 3};
  const DynamicPolynomial second_factor = {4};
  const DynamicPolynomial product = first_factor * second_factor;
  // 12x + 20 = (3x + 5) * 4
  DynamicPolynomial expected = {20, 12};
  ASSERT_EQ(expected, product);
}

TEST_F(PolynomialTest, DynamicLinearPolynomialMultipliedWithDynamicLinearPolynomial) {
  // 3x + 5
  const DynamicPolynomial first_factor = {5, 3};
  // 6x + 4
  const DynamicPolynomial second_factor = {4, 6};
  const DynamicPolynomial product = first_factor * second_factor;
  // 18x² + (12 + 30)x + 20 = (3x + 5) * (6x + 4)
  const DynamicPolynomial expected = {20, 42, 18};
  ASSERT_EQ(expected, product);
}

TEST_F(PolynomialTest, DynamicLinearPolynomialMultipliedWithDynamicQuadraticPolynomial) {
  // 3x + 5
  const DynamicPolynomial first_factor = {5, 3};
  // 2x² + 6x + 4
  const DynamicPolynomial second_factor = {4, 6, 2};
  const DynamicPolynomial product = first_factor * second_factor;
  // 6x³ + (18 + 10)x² + (12 + 30)x + 20 = (3x + 5) * (2x² + 6x + 4)
  const DynamicPolynomial expected = {20, 42, 28, 6};
  ASSERT_EQ(expected, product);
}

TEST_F(PolynomialTest, ZeroPolynomialAddedToZeroPolynomial) {
  ZeroPolynomial first_summand = createPolynomial<PolynomialDegrees::MinusInfinity>();
  ZeroPolynomial second_summand = createPolynomial<PolynomialDegrees::MinusInfinity>();
  ZeroPolynomial sum = first_summand + second_summand;
  ZeroPolynomial expected;
  ASSERT_EQ(expected, sum);
}

TEST_F(PolynomialTest, ZeroPolynomialAddedToConstantPolynomial) {
  ZeroPolynomial first_summand = createPolynomial<PolynomialDegrees::MinusInfinity>();
  ConstantPolynomial second_summand = createPolynomial<PolynomialDegrees::Constant>();
  ConstantPolynomial sum = first_summand + second_summand;
  ConstantPolynomial expected = createPolynomial<PolynomialDegrees::Constant>();
  ASSERT_EQ(expected, sum);
}

TEST_F(PolynomialTest, ConstantPolynomialAddedToZeroPolynomial) {
  ConstantPolynomial first_summand = createPolynomial<PolynomialDegrees::Constant>();
  ZeroPolynomial second_summand = createPolynomial<PolynomialDegrees::MinusInfinity>();
  ConstantPolynomial sum = first_summand + second_summand;
  ConstantPolynomial expected = createPolynomial<PolynomialDegrees::Constant>();
  ASSERT_EQ(expected, sum);
}

TEST_F(PolynomialTest, ConstantPolynomialAddedToConstantPolynomial) {
  const ConstantPolynomial first_summand = {4};
  const ConstantPolynomial second_summand = {5};
  const ConstantPolynomial sum = first_summand + second_summand;
  const ConstantPolynomial expected = {9};
  ASSERT_EQ(expected, sum);
}

TEST_F(PolynomialTest, ConstantPolynomialAddedToLinearPolynomial) {
  const ConstantPolynomial first_summand = {4};
  // 3x + 5
  const LinearPolynomial second_summand = {5, 3};
  const LinearPolynomial sum = first_summand + second_summand;
  // 3x + 9
  const LinearPolynomial expected = {9, 3};
  ASSERT_EQ(expected, sum);
}

TEST_F(PolynomialTest, LinearPolynomialAddedToConstantPolynomial) {
  // 3x + 5
  const LinearPolynomial first_summand = {5, 3};
  const ConstantPolynomial second_summand = {4};
  const LinearPolynomial sum = first_summand + second_summand;
  // 3x + 9
  const LinearPolynomial expected = {9, 3};
  ASSERT_EQ(expected, sum);
}

TEST_F(PolynomialTest, LinearPolynomialAddedToLinearPolynomial) {
  // 3x + 5
  const LinearPolynomial first_summand = {5, 3};
  // 7x + 4
  const LinearPolynomial second_summand = {4, 7};
  const LinearPolynomial sum = first_summand + second_summand;
  // 10x + 9 = (3x + 5) + (7x + 4)
  const LinearPolynomial expected = {9, 10};
  ASSERT_EQ(expected, sum);
}

TEST_F(PolynomialTest, LinearPolynomialAddedToQuadraticPolynomial) {
  // 3x + 5
  const LinearPolynomial first_summand = {5, 3};
  // 2x² + 7x + 4
  const QuadraticPolynomial second_summand = {4, 7, 2};
  const QuadraticPolynomial sum = first_summand + second_summand;
  // 2x² + 10x + 9 = (3x + 5) + (2x² + 7x + 4)
  const QuadraticPolynomial expected = {9, 10, 2};
  ASSERT_EQ(expected, sum);
}

TEST_F(PolynomialTest, DynamicZeroPolynomialAddedToDynamicZeroPolynomial) {
  DynamicPolynomial first_summand =
      createDynamicPolynomial<PolynomialDegrees::MinusInfinity>();
  DynamicPolynomial second_summand =
      createDynamicPolynomial<PolynomialDegrees::MinusInfinity>();
  DynamicPolynomial sum = first_summand + second_summand;
  DynamicPolynomial expected;
  ASSERT_EQ(expected, sum);
}

TEST_F(PolynomialTest, DynamicZeroPolynomialAddedToDynamicConstantPolynomial) {
  DynamicPolynomial first_summand =
      createDynamicPolynomial<PolynomialDegrees::MinusInfinity>();
  DynamicPolynomial second_summand =
      createDynamicPolynomial<PolynomialDegrees::Constant>();
  DynamicPolynomial sum = first_summand + second_summand;
  DynamicPolynomial expected = createDynamicPolynomial<PolynomialDegrees::Constant>();
  ASSERT_EQ(expected, sum);
}

TEST_F(PolynomialTest, DynamicConstantPolynomialAddedToDynamicZeroPolynomial) {
  DynamicPolynomial first_summand =
      createDynamicPolynomial<PolynomialDegrees::Constant>();
  DynamicPolynomial second_summand =
      createDynamicPolynomial<PolynomialDegrees::MinusInfinity>();
  DynamicPolynomial sum = first_summand + second_summand;
  DynamicPolynomial expected = createDynamicPolynomial<PolynomialDegrees::Constant>();
  ASSERT_EQ(expected, sum);
}

TEST_F(PolynomialTest, DynamicConstantPolynomialAddedToDynamicConstantPolynomial) {
  const DynamicPolynomial first_summand = {4};
  const DynamicPolynomial second_summand = {5};
  const DynamicPolynomial sum = first_summand + second_summand;
  const DynamicPolynomial expected = {9};
  ASSERT_EQ(expected, sum);
}

TEST_F(PolynomialTest, DynamicConstantPolynomialAddedToDynamicLinearPolynomial) {
  const DynamicPolynomial first_summand = {4};
  // 3x + 5
  const DynamicPolynomial second_summand = {5, 3};
  const DynamicPolynomial sum = first_summand + second_summand;
  // 3x + 9 = (3x + 5) + 4
  const DynamicPolynomial expected = {9, 3};
  ASSERT_EQ(expected, sum);
}

TEST_F(PolynomialTest, DynamicLinearPolynomialAddedToDynamicConstantPolynomial) {
  // 3x + 5
  const DynamicPolynomial first_summand = {5, 3};
  const DynamicPolynomial second_summand = {4};
  const DynamicPolynomial sum = first_summand + second_summand;
  // 3x + 9 = (3x + 5) + 4
  const DynamicPolynomial expected = {9, 3};
  ASSERT_EQ(expected, sum);
}

TEST_F(PolynomialTest, DynamicLinearPolynomialAddedToDynamicLinearPolynomial) {
  // 3x + 5
  const DynamicPolynomial first_summand = {5, 3};
  // 7x + 4
  const DynamicPolynomial second_summand = {4, 7};
  const DynamicPolynomial sum = first_summand + second_summand;
  // 10x + 9 = (3x + 5) + (7x + 4)
  const DynamicPolynomial expected = {9, 10};
  ASSERT_EQ(expected, sum);
}

TEST_F(PolynomialTest, DynamicLinearPolynomialAddedToDynamicQuadraticPolynomial) {
  // 3x + 5
  const DynamicPolynomial first_summand = {5, 3};
  // 2x² + 7x + 4
  const DynamicPolynomial second_summand = {4, 7, 2};
  const DynamicPolynomial sum = first_summand + second_summand;
  // 2x² + 10x + 9 = (3x + 5) + (2x² + 7x + 4)
  const DynamicPolynomial expected = {9, 10, 2};
  ASSERT_EQ(expected, sum);
}

TEST_F(PolynomialTest, ZeroPolynomialMinusZeroPolynomial) {
  ZeroPolynomial minuend = createPolynomial<PolynomialDegrees::MinusInfinity>();
  ZeroPolynomial subtrahend = createPolynomial<PolynomialDegrees::MinusInfinity>();
  ZeroPolynomial difference = minuend - subtrahend;
  ZeroPolynomial expected;
  ASSERT_EQ(expected, difference);
}

TEST_F(PolynomialTest, ZeroPolynomialMinusConstantPolynomial) {
  ZeroPolynomial minuend = createPolynomial<PolynomialDegrees::MinusInfinity>();
  ConstantPolynomial subtrahend = createPolynomial<PolynomialDegrees::Constant>();
  ConstantPolynomial difference = minuend - subtrahend;
  const ConstantPolynomial expected = {-48};
  ASSERT_EQ(expected, difference);
}

TEST_F(PolynomialTest, ConstantPolynomialMinusZeroPolynomial) {
  ConstantPolynomial minuend = createPolynomial<PolynomialDegrees::Constant>();
  ZeroPolynomial subtrahend = createPolynomial<PolynomialDegrees::MinusInfinity>();
  ConstantPolynomial difference = minuend - subtrahend;
  ConstantPolynomial expected = createPolynomial<PolynomialDegrees::Constant>();
  ASSERT_EQ(expected, difference);
}

TEST_F(PolynomialTest, ConstantPolynomialMinusConstantPolynomial) {
  const ConstantPolynomial minuend = {4};
  const ConstantPolynomial subtrahend = {5};
  const ConstantPolynomial difference = minuend - subtrahend;
  const ConstantPolynomial expected = {-1};
  ASSERT_EQ(expected, difference);
}

TEST_F(PolynomialTest, ConstantPolynomialMinusLinearPolynomial) {
  const ConstantPolynomial minuend = {4};
  // 3x + 5
  const LinearPolynomial subtrahend = {5, 3};
  const LinearPolynomial difference = minuend - subtrahend;
  // -3x - 1
  const LinearPolynomial expected = {-1, -3};
  ASSERT_EQ(expected, difference);
}

TEST_F(PolynomialTest, LinearPolynomialMinusConstantPolynomial) {
  // 3x + 5
  const LinearPolynomial minuend = {5, 3};
  const ConstantPolynomial subtrahend = {4};
  const LinearPolynomial difference = minuend - subtrahend;
  // 3x + 1
  const LinearPolynomial expected = {1, 3};
  ASSERT_EQ(expected, difference);
}

TEST_F(PolynomialTest, LinearPolynomialMinusLinearPolynomial) {
  // 3x + 5
  const LinearPolynomial minuend = {5, 3};
  // 7x + 4
  const LinearPolynomial subtrahend = {4, 7};
  const LinearPolynomial difference = minuend - subtrahend;
  // -4x + 1 = (3x + 5) - (7x + 4)
  const LinearPolynomial expected = {1, -4};
  ASSERT_EQ(expected, difference);
}

TEST_F(PolynomialTest, LinearPolynomialMinusQuadraticPolynomial) {
  // 3x + 5
  const LinearPolynomial minuend = {5, 3};
  // 2x² + 7x + 4
  const QuadraticPolynomial subtrahend = {4, 7, 2};
  const QuadraticPolynomial difference = minuend - subtrahend;
  // -2x² - 4x + 1 = (3x + 5) - (2x² + 7x + 4)
  const QuadraticPolynomial expected = {1, -4, -2};
  ASSERT_EQ(expected, difference);
}

TEST_F(PolynomialTest, DynamicZeroPolynomialMinusDynamicZeroPolynomial) {
  DynamicPolynomial minuend =
      createDynamicPolynomial<PolynomialDegrees::MinusInfinity>();
  DynamicPolynomial subtrahend =
      createDynamicPolynomial<PolynomialDegrees::MinusInfinity>();
  DynamicPolynomial difference = minuend - subtrahend;
  DynamicPolynomial expected;
  ASSERT_EQ(expected, difference);
}

TEST_F(PolynomialTest, DynamicZeroPolynomialMinusDynamicConstantPolynomial) {
  DynamicPolynomial minuend =
      createDynamicPolynomial<PolynomialDegrees::MinusInfinity>();
  DynamicPolynomial subtrahend = createDynamicPolynomial<PolynomialDegrees::Constant>();
  DynamicPolynomial difference = minuend - subtrahend;
  const DynamicPolynomial expected = {-48};
  ASSERT_EQ(expected, difference);
}

TEST_F(PolynomialTest, DynamicConstantPolynomialMinusDynamicZeroPolynomial) {
  DynamicPolynomial minuend = createDynamicPolynomial<PolynomialDegrees::Constant>();
  DynamicPolynomial subtrahend =
      createDynamicPolynomial<PolynomialDegrees::MinusInfinity>();
  DynamicPolynomial difference = minuend - subtrahend;
  DynamicPolynomial expected = createDynamicPolynomial<PolynomialDegrees::Constant>();
  ASSERT_EQ(expected, difference);
}

TEST_F(PolynomialTest, DynamicConstantPolynomialMinusDynamicConstantPolynomial) {
  const DynamicPolynomial minuend = {4};
  const DynamicPolynomial subtrahend = {5};
  const DynamicPolynomial difference = minuend - subtrahend;
  const DynamicPolynomial expected = {-1};
  ASSERT_EQ(expected, difference);
}

TEST_F(PolynomialTest, DynamicConstantPolynomialMinusDynamicLinearPolynomial) {
  const DynamicPolynomial minuend = {4};
  // 3x + 5
  const DynamicPolynomial subtrahend = {5, 3};
  const DynamicPolynomial difference = minuend - subtrahend;
  // -3x - 1 = 4 - (3x + 5)
  const DynamicPolynomial expected = {-1, -3};
  ASSERT_EQ(expected, difference);
}

TEST_F(PolynomialTest, DynamicLinearPolynomialMinusDynamicConstantPolynomial) {
  // 3x + 5
  const DynamicPolynomial minuend = {5, 3};
  const DynamicPolynomial subtrahend = {4};
  const DynamicPolynomial difference = minuend - subtrahend;
  // 3x + 1 = (3x + 5) - 4
  const DynamicPolynomial expected = {1, 3};
  ASSERT_EQ(expected, difference);
}

TEST_F(PolynomialTest, DynamicLinearPolynomialMinusDynamicLinearPolynomial) {
  // 3x + 5
  const DynamicPolynomial minuend = {5, 3};
  // 7x + 4
  const DynamicPolynomial subtrahend = {4, 7};
  const DynamicPolynomial difference = minuend - subtrahend;
  // -4x + 1 = (3x + 5) - (7x + 4)
  const DynamicPolynomial expected = {1, -4};
  ASSERT_EQ(expected, difference);
}

TEST_F(PolynomialTest, DynamicLinearPolynomialMinusDynamicQuadraticPolynomial) {
  // 3x + 5
  const DynamicPolynomial minuend = {5, 3};
  // 2x² + 7x + 4
  const DynamicPolynomial subtrahend = {4, 7, 2};
  const DynamicPolynomial difference = minuend - subtrahend;
  // -2x² - 4x + 1 = (3x + 5) - (2x² + 7x + 4)
  const DynamicPolynomial expected = {1, -4, -2};
  ASSERT_EQ(expected, difference);
}

TEST_F(PolynomialTest, DynamicMinusDouble) {
  double a = 4.2;
  DynamicPolynomial polynomial = createDynamicPolynomial<PolynomialDegrees::Quadratic>();

  DynamicPolynomial polynomial_minus_a = polynomial - a;
  std::vector<double> polynomial_coefficients = polynomial.getCoefficients();
  std::vector<double> polynomial_minus_a_coefficients =
      polynomial_minus_a.getCoefficients();
  ASSERT_EQ(polynomial_minus_a_coefficients[0], polynomial_coefficients[0] - a);
  ASSERT_EQ(polynomial_minus_a_coefficients[1], polynomial_coefficients[1]);
  ASSERT_EQ(polynomial_minus_a_coefficients[2], polynomial_coefficients[2]);

  DynamicPolynomial a_minus_polynomial = a - polynomial;
  std::vector<double> a_minus_polynomial_coefficiens =
      a_minus_polynomial.getCoefficients();
  ASSERT_EQ(a_minus_polynomial_coefficiens[0], a - polynomial_coefficients[0]);
  ASSERT_EQ(a_minus_polynomial_coefficiens[1], -polynomial_coefficients[1]);
  ASSERT_EQ(a_minus_polynomial_coefficiens[2], -polynomial_coefficients[2]);
}

TEST_F(PolynomialTest, StaticMinusDouble) {
  double a = 4.2;
  QuarticPolynomial polynomial = createPolynomial<PolynomialDegrees::Quadratic>();

  QuarticPolynomial polynomial_minus_a = polynomial - a;
  std::vector<double> polynomial_coefficients = polynomial.getCoefficients();
  std::vector<double> polynomial_minus_a_coefficients =
      polynomial_minus_a.getCoefficients();
  ASSERT_EQ(polynomial_minus_a_coefficients[0], polynomial_coefficients[0] - a);
  ASSERT_EQ(polynomial_minus_a_coefficients[1], polynomial_coefficients[1]);
  ASSERT_EQ(polynomial_minus_a_coefficients[2], polynomial_coefficients[2]);

  QuarticPolynomial a_minus_polynomial = a - polynomial;
  std::vector<double> a_minus_polynomial_coefficiens =
      a_minus_polynomial.getCoefficients();
  ASSERT_EQ(a_minus_polynomial_coefficiens[0], a - polynomial_coefficients[0]);
  ASSERT_EQ(a_minus_polynomial_coefficiens[1], -polynomial_coefficients[1]);
  ASSERT_EQ(a_minus_polynomial_coefficiens[2], -polynomial_coefficients[2]);
}

// Squared Polynomials

TEST_F(PolynomialTest, ZeroPolynomialSquared) {
  ZeroPolynomial polynomial;
  ZeroPolynomial product = squared(polynomial);
  ZeroPolynomial expected;
  ASSERT_EQ(expected, product);
}

TEST_F(PolynomialTest, LinearPolynomialSquared) {
  // 3x + 5
  const LinearPolynomial polynomial = {5, 3};
  const QuadraticPolynomial product = squared(polynomial);
  // 9x² + 30x + 25 = (3x + 5)²
  const QuadraticPolynomial expected = {25, 30, 9};
  ASSERT_EQ(expected, product);
}

TEST_F(PolynomialTest, DynamicZeroPolynomialSquared) {
  DynamicPolynomial polynomial;
  DynamicPolynomial product = squared(polynomial);
  DynamicPolynomial expected;
  ASSERT_EQ(expected, product);
}

TEST_F(PolynomialTest, DynamicLinearPolynomialSquared) {
  // 3x + 5
  const DynamicPolynomial polynomial = {5, 3};
  const DynamicPolynomial product = squared(polynomial);
  // 9x² + 30x + 25 = (3x + 5)²
  const DynamicPolynomial expected = {25, 30, 9};
  ASSERT_EQ(expected, product);
}
// Curvature

TEST_F(PolynomialTest, CubicPolynomialCurvatureCalculation) {
  // 8x³ + 9x² + 10x + 7
  // df = 24x² + 18x + 10
  // ddf = 48x + 18
  const CubicPolynomial polynomial = {7, 10, 9, 8};
  ASSERT_EQ(0.0, polynomial.calculateCurvature(-0.375));
  ASSERT_EQ((48 * 2.0 + 18) / pow(1.0 + pow(24 * 2.0 * 2.0 + 18 * 2.0 + 10, 2.0), 1.5),
            polynomial.calculateCurvature(2.0));
}

// Reduction Error Calculation

TEST_F(PolynomialTest, CubicPolynomialReductionError) {
  // 8x³ + 9x² + 10x + 7
  const CubicPolynomial polynomial = {7, 10, 9, 8};
  ASSERT_EQ(0, polynomial.calculateReductionError(PolynomialDegrees::Quartic));
  ASSERT_EQ(0, polynomial.calculateReductionError(PolynomialDegrees::Cubic));
  ASSERT_EQ(sqrt(8 * 8), polynomial.calculateReductionError(PolynomialDegrees::Quadratic));
  ASSERT_EQ(sqrt(8 * 8 + 9 * 9),
            polynomial.calculateReductionError(PolynomialDegrees::Linear));
  ASSERT_EQ(sqrt(8 * 8 + 9 * 9 + 10 * 10),
            polynomial.calculateReductionError(PolynomialDegrees::Constant));
  ASSERT_EQ(sqrt(8 * 8 + 9 * 9 + 10 * 10 + 7 * 7),
            polynomial.calculateReductionError(PolynomialDegrees::MinusInfinity));
}

TEST_F(PolynomialTest, DynamicPolynomialReductionError) {
  // 8x³ + 9x² + 10x + 7
  const DynamicPolynomial polynomial = {7, 10, 9, 8};
  ASSERT_EQ(0, polynomial.calculateReductionError(PolynomialDegrees::Quartic));
  ASSERT_EQ(0, polynomial.calculateReductionError(PolynomialDegrees::Cubic));
  ASSERT_EQ(sqrt(8 * 8), polynomial.calculateReductionError(PolynomialDegrees::Quadratic));
  ASSERT_EQ(sqrt(8 * 8 + 9 * 9),
            polynomial.calculateReductionError(PolynomialDegrees::Linear));
  ASSERT_EQ(sqrt(8 * 8 + 9 * 9 + 10 * 10),
            polynomial.calculateReductionError(PolynomialDegrees::Constant));
  ASSERT_EQ(sqrt(8 * 8 + 9 * 9 + 10 * 10 + 7 * 7),
            polynomial.calculateReductionError(PolynomialDegrees::MinusInfinity));
}

// Get Coefficients

TEST_F(PolynomialTest, CubicPolynomialGetCoefficients) {
  // 8x³ + 9x² + 10x + 7
  const CubicPolynomial polynomial = {7, 10, 9, 8};
  const std::vector<double> result = polynomial.getCoefficients();
  ASSERT_EQ(7.0, result.at(0));
  ASSERT_EQ(10.0, result.at(1));
  ASSERT_EQ(9.0, result.at(2));
  ASSERT_EQ(8.0, result.at(3));
}

TEST_F(PolynomialTest, DynamicPolynomialGetCoefficients) {
  // 8x³ + 9x² + 10x + 7
  const DynamicPolynomial polynomial = {7, 10, 9, 8};
  const std::vector<double> result = polynomial.getCoefficients();
  ASSERT_EQ(7.0, result.at(0));
  ASSERT_EQ(10.0, result.at(1));
  ASSERT_EQ(9.0, result.at(2));
  ASSERT_EQ(8.0, result.at(3));
}

TEST_F(PolynomialTest, ZeroPolynomialGetCoefficients) {
  ZeroPolynomial polynomial;
  std::vector<double> result = polynomial.getCoefficients();
  ASSERT_EQ(0, result.size());
}

// Integral

TEST_F(PolynomialTest, DynamicFirstSimpleIntegral) {
  const DynamicPolynomial polynomial = {1.0, 2.0, 3.0};
  const DynamicPolynomial integral = polynomial.integrate();
  const DynamicPolynomial::CoefficientList integral_coefficients =
      integral.getCoefficients();
  ASSERT_EQ(polynomial.getCoefficients().size() + 1, integral_coefficients.size());
  ASSERT_EQ(0.0, integral_coefficients[0]);
  ASSERT_EQ(1.0, integral_coefficients[1]);
  ASSERT_EQ(1.0, integral_coefficients[2]);
  ASSERT_EQ(1.0, integral_coefficients[3]);
}

TEST_F(PolynomialTest, DynamicIntegralDerivate) {
  DynamicPolynomial polynom = createDynamicPolynomial<PolynomialDegrees::Quartic>();
  DynamicPolynomial integral_derivate = polynom.integrate().derivate();
  DynamicPolynomial::CoefficientList polynom_coefficients = polynom.getCoefficients();
  DynamicPolynomial::CoefficientList integral_derivate_coefficients =
      integral_derivate.getCoefficients();

  ASSERT_EQ(polynom_coefficients.size(), integral_derivate_coefficients.size());
  for (std::size_t i = 0; i < polynom_coefficients.size(); ++i) {
    ASSERT_EQ(polynom_coefficients[i], integral_derivate_coefficients[i]);
  }
}

TEST_F(PolynomialTest, QuadraticFirstSimpleIntegral) {
  const QuadraticPolynomial polynomial = {1.0, 2.0, 3.0};
  const CubicPolynomial integral = polynomial.integrate();
  const std::vector<double> integral_coefficients = integral.getCoefficients();
  ASSERT_EQ(0.0, integral_coefficients[0]);
  ASSERT_EQ(1.0, integral_coefficients[1]);
  ASSERT_EQ(1.0, integral_coefficients[2]);
  ASSERT_EQ(1.0, integral_coefficients[3]);
}

TEST_F(PolynomialTest, QuarticIntegralDerivate) {
  QuarticPolynomial polynom = createPolynomial<PolynomialDegrees::Quartic>();
  QuarticPolynomial integral_derivate = polynom.integrate().derivate();
  std::vector<double> polynom_coefficients = polynom.getCoefficients();
  std::vector<double> integral_derivate_coefficients =
      integral_derivate.getCoefficients();

  for (std::size_t i = 0; i < polynom_coefficients.size(); ++i) {
    ASSERT_EQ(polynom_coefficients[i], integral_derivate_coefficients[i]);
  }
}

TEST_F(PolynomialTest, DynamicFirstSimpleIntegral2) {
  const DynamicPolynomial polynomial = {2.0};
  DynamicPolynomial integral = polynomial.calculateIntegral<2>();
  DynamicPolynomial::CoefficientList integral_coefficients = integral.getCoefficients();
  ASSERT_EQ(polynomial.getCoefficients().size() + 2, integral_coefficients.size());
  ASSERT_EQ(0.0, integral_coefficients[0]);
  ASSERT_EQ(0.0, integral_coefficients[1]);
  ASSERT_EQ(1.0, integral_coefficients[2]);
}

TEST_F(PolynomialTest, DynamicIntegralDerivate2) {
  DynamicPolynomial polynom = createDynamicPolynomial<PolynomialDegrees::Quartic>();
  DynamicPolynomial integral_derivate =
      polynom.calculateIntegral<2>().calculateDerivative<2>();
  DynamicPolynomial::CoefficientList polynom_coefficients = polynom.getCoefficients();
  DynamicPolynomial::CoefficientList integral_derivate_coefficients =
      integral_derivate.getCoefficients();

  ASSERT_EQ(polynom_coefficients.size(), integral_derivate_coefficients.size());
  for (std::size_t i = 0; i < polynom_coefficients.size(); ++i) {
    ASSERT_EQ(polynom_coefficients[i], integral_derivate_coefficients[i]);
  }
}

TEST_F(PolynomialTest, StaticFirstSimpleIntegral2) {
  const ConstantPolynomial polynomial = {2.0};
  QuadraticPolynomial integral = polynomial.calculateIntegral<2>();
  std::vector<double> integral_coefficients = integral.getCoefficients();
  ASSERT_EQ(polynomial.getCoefficients().size() + 2, integral_coefficients.size());
  ASSERT_EQ(0.0, integral_coefficients[0]);
  ASSERT_EQ(0.0, integral_coefficients[1]);
  ASSERT_EQ(1.0, integral_coefficients[2]);
}

TEST_F(PolynomialTest, StaticIntegralDerivate2) {
  QuarticPolynomial polynom = createPolynomial<PolynomialDegrees::Quartic>();
  QuarticPolynomial integral_derivate =
      polynom.calculateIntegral<2>().calculateDerivative<2>();
  std::vector<double> polynom_coefficients = polynom.getCoefficients();
  std::vector<double> integral_derivate_coefficients =
      integral_derivate.getCoefficients();

  ASSERT_EQ(polynom_coefficients.size(), integral_derivate_coefficients.size());
  for (std::size_t i = 0; i < polynom_coefficients.size(); ++i) {
    ASSERT_EQ(polynom_coefficients[i], integral_derivate_coefficients[i]);
  }
}

// Conversions

TEST_F(PolynomialTest, StaticToDynamicPolynomialConversion) {
  QuarticPolynomial static_polynomial = createPolynomial<PolynomialDegrees::Quartic>();
  DynamicPolynomial dynamic_polynomial = static_polynomial;
  std::vector<double> static_coefficients = static_polynomial.getCoefficients();
  std::vector<double> dynamic_coefficients = dynamic_polynomial.getCoefficients();

  ASSERT_EQ(static_coefficients.size(), dynamic_coefficients.size());

  for (std::size_t i = 0; i < static_coefficients.size(); ++i) {
    ASSERT_EQ(static_coefficients[i], dynamic_coefficients[i]);
  }
}

TEST_F(PolynomialTest, QuadraticToQuarticConversion) {
  QuadraticPolynomial quadratic_polynomial =
      createPolynomial<PolynomialDegrees::Quadratic>();
  QuarticPolynomial quartic_polynomial = quadratic_polynomial;

  QuadraticPolynomial::CoefficientList quadratic_coefficients =
      quadratic_polynomial.getCoefficientsList();
  QuarticPolynomial::CoefficientList quartic_coefficients =
      quartic_polynomial.getCoefficientsList();

  ASSERT_EQ(quadratic_coefficients[0], quartic_coefficients[0]);
  ASSERT_EQ(quadratic_coefficients[1], quartic_coefficients[1]);
  ASSERT_EQ(quadratic_coefficients[2], quartic_coefficients[2]);
  ASSERT_EQ(0, quartic_coefficients[3]);
  ASSERT_EQ(0, quartic_coefficients[4]);
}


TEST_F(PolynomialTest, QuarticToQuadraticConversion) {
  QuarticPolynomial quartic_polynomial = createPolynomial<PolynomialDegrees::Quartic>();
  // QuadraticPolynomial quadratic_polynomial = quartic_polynomial; //This line
  // should not compile!
  QuadraticPolynomial quadratic_polynomial(quartic_polynomial);

  QuadraticPolynomial::CoefficientList quadratic_coefficients =
      quadratic_polynomial.getCoefficientsList();
  QuarticPolynomial::CoefficientList quartic_coefficients =
      quartic_polynomial.getCoefficientsList();

  ASSERT_EQ(quartic_coefficients[0], quadratic_coefficients[0]);
  ASSERT_EQ(quartic_coefficients[1], quadratic_coefficients[1]);
  ASSERT_EQ(quartic_coefficients[2], quadratic_coefficients[2]);
}

TEST_F(PolynomialTest, DynamicToQuarticConversion) {
  DynamicPolynomial quartic_dynamic_polynomial =
      createDynamicPolynomial<PolynomialDegrees::Quartic>();
  // QuarticPolynomial quartic_polynomial = sextic_dynamic_polynomial; //This
  // line should not compile!
  CubicPolynomial cubic_polynomial(quartic_dynamic_polynomial);

  DynamicPolynomial::CoefficientList quartic_dynamic_coefficients =
      quartic_dynamic_polynomial.getCoefficients();
  CubicPolynomial::CoefficientList cubic_coefficients =
      cubic_polynomial.getCoefficientsList();

  ASSERT_EQ(quartic_dynamic_coefficients[0], cubic_coefficients[0]);
  ASSERT_EQ(quartic_dynamic_coefficients[1], cubic_coefficients[1]);
  ASSERT_EQ(quartic_dynamic_coefficients[2], cubic_coefficients[2]);
  ASSERT_EQ(quartic_dynamic_coefficients[3], cubic_coefficients[3]);
}

TEST_F(PolynomialTest, PathologicConversions) {
  ZeroPolynomial zero_polynomial;
  DynamicPolynomial dynamic_zero_polynomial = zero_polynomial;
  ASSERT_EQ(zero_polynomial.getCoefficients().size(),
            dynamic_zero_polynomial.getCoefficients().size());
  QuarticPolynomial quartic_zero_polynomial = zero_polynomial;
  for (double c : quartic_zero_polynomial.getCoefficientsList()) {
    ASSERT_EQ(0.0, c);
  }
  // zero_polynomial = createDynamicPolynomial<PolynomialDegrees::Quintic>();
  // //This line should not compile!
}

// Unary Operator

TEST_F(PolynomialTest, QuarticUnaryMinus) {
  QuarticPolynomial polynomial = createPolynomial<PolynomialDegrees::Quadratic>();
  QuarticPolynomial minus_polynomial = -polynomial;

  QuarticPolynomial::CoefficientList polynomial_coefficients =
      polynomial.getCoefficientsList();
  QuarticPolynomial::CoefficientList minus_polynomial_coefficients =
      minus_polynomial.getCoefficientsList();

  for (std::size_t i = 0; i < polynomial_coefficients.size(); i++) {
    ASSERT_EQ(polynomial_coefficients[i], -minus_polynomial_coefficients[i]);
  }
}


TEST_F(PolynomialTest, DynamicUnaryMinus) {
  DynamicPolynomial polynomial = createDynamicPolynomial<PolynomialDegrees::Quadratic>();
  DynamicPolynomial minus_polynomial = -polynomial;

  DynamicPolynomial::CoefficientList polynomial_coefficients =
      polynomial.getCoefficientsList();
  DynamicPolynomial::CoefficientList minus_polynomial_coefficients =
      minus_polynomial.getCoefficientsList();

  for (std::size_t i = 0; i < polynomial_coefficients.size(); i++) {
    ASSERT_EQ(polynomial_coefficients[i], -minus_polynomial_coefficients[i]);
  }
}


// Literals

TEST_F(PolynomialTest, QuarticLiterals) {
  QuarticPolynomial polynomial = 1.0_x_4 + 2.0_x_3 + 3.0_x_2 + 4.0_x + 5.0;
  QuarticPolynomial::CoefficientList coefficients = polynomial.getCoefficientsList();
  ASSERT_EQ(5.0, coefficients[0]);
  ASSERT_EQ(4.0, coefficients[1]);
  ASSERT_EQ(3.0, coefficients[2]);
  ASSERT_EQ(2.0, coefficients[3]);
  ASSERT_EQ(1.0, coefficients[4]);
}

TEST_F(PolynomialTest, QuarticLiteralsWithHoles) {
  QuarticPolynomial polynomial = 2.0_x_3 + 4.0_x + 5.0;
  QuarticPolynomial::CoefficientList coefficients = polynomial.getCoefficientsList();
  ASSERT_EQ(5.0, coefficients[0]);
  ASSERT_EQ(4.0, coefficients[1]);
  ASSERT_EQ(0.0, coefficients[2]);
  ASSERT_EQ(2.0, coefficients[3]);
  ASSERT_EQ(0.0, coefficients[4]);
}

TEST_F(PolynomialTest, DynamicPolynomialLiterals) {
  DynamicPolynomial polynomial = 1.0_x_4 + 2.0_x_3 + 3.0_x_2 + 4.0_x + 5.0;
  DynamicPolynomial::CoefficientList coefficients = polynomial.getCoefficients();
  ASSERT_EQ(5, coefficients.size());
  ASSERT_EQ(5.0, coefficients[0]);
  ASSERT_EQ(4.0, coefficients[1]);
  ASSERT_EQ(3.0, coefficients[2]);
  ASSERT_EQ(2.0, coefficients[3]);
  ASSERT_EQ(1.0, coefficients[4]);

  polynomial = -1.0_x_4 - 2.0_x_3 - 3.0_x_2 - 4.0_x - 5.0;
  coefficients = polynomial.getCoefficients();
  ASSERT_EQ(5, coefficients.size());
  ASSERT_EQ(-5.0, coefficients[0]);
  ASSERT_EQ(-4.0, coefficients[1]);
  ASSERT_EQ(-3.0, coefficients[2]);
  ASSERT_EQ(-2.0, coefficients[3]);
  ASSERT_EQ(-1.0, coefficients[4]);
}

TEST_F(PolynomialTest, DynamicPolynomialLiteralsWithHoles) {
  DynamicPolynomial polynomial = 2.0_x_3 + 4.0_x + 5.0;
  DynamicPolynomial::CoefficientList coefficients = polynomial.getCoefficients();
  ASSERT_EQ(4, coefficients.size());
  ASSERT_EQ(5.0, coefficients[0]);
  ASSERT_EQ(4.0, coefficients[1]);
  ASSERT_EQ(0.0, coefficients[2]);
  ASSERT_EQ(2.0, coefficients[3]);

  polynomial = -2.0_x_3 - 4.0_x - 5.0;
  coefficients = polynomial.getCoefficients();
  ASSERT_EQ(4, coefficients.size());
  ASSERT_EQ(-5.0, coefficients[0]);
  ASSERT_EQ(-4.0, coefficients[1]);
  ASSERT_EQ(-0.0, coefficients[2]);
  ASSERT_EQ(-2.0, coefficients[3]);
}

TEST_F(PolynomialTest, LiteralsSize) {
  DynamicPolynomial p0 = 1.0;
  ASSERT_EQ(1, p0.getCoefficients().size());
  DynamicPolynomial p1 = 1.0_x;
  ASSERT_EQ(2, p1.getCoefficients().size());
  DynamicPolynomial p2 = 1.0_x_2;
  ASSERT_EQ(3, p2.getCoefficients().size());
  DynamicPolynomial p3 = 1.0_x_3;
  ASSERT_EQ(4, p3.getCoefficients().size());
  DynamicPolynomial p4 = 1.0_x_4;
  ASSERT_EQ(5, p4.getCoefficients().size());
  DynamicPolynomial p5 = 1.0_x_5;
  ASSERT_EQ(6, p5.getCoefficients().size());
  DynamicPolynomial p6 = 1.0_x_6;
  ASSERT_EQ(7, p6.getCoefficients().size());
  DynamicPolynomial p7 = 1.0_x_7;
  ASSERT_EQ(8, p7.getCoefficients().size());
}

TEST_F(PolynomialTest, IntegrateEmptyDynamicPolynomial) {
  const DynamicPolynomial empty_p = {};
  const DynamicPolynomial integrated = empty_p.integrate();
  EXPECT_TRUE(integrated.getCoefficientsList().empty());
}

TEST_F(PolynomialTest, DynamicPolynomialToStream) {
  const DynamicPolynomial p = {10, 3, 6, 1};
  std::stringstream ss;
  ss << p;
  EXPECT_EQ(
      std::string("Polynomial<MAX_DEGREE=-2>: 1 x^3 + 6 x^2 + 3 x^1 + 10"), ss.str());
}

TEST_F(PolynomialTest, EmptyDynamicPolynomialToStream) {
  const DynamicPolynomial p;
  std::stringstream ss;
  ss << p;
  EXPECT_EQ(std::string("Polynomial<MAX_DEGREE=-2>: 0"), ss.str());
}


TEST_F(PolynomialTest, StaticPolynomialToStream) {
  const CubicPolynomial p = {10, 3, 6, 1};
  std::stringstream ss;
  ss << p;
  EXPECT_EQ(std::string("Polynomial<MAX_DEGREE=3>: 1 x^3 + 6 x^2 + 3 x^1 + 10"), ss.str());
}

TEST_F(PolynomialTest, ZeroPolynomialToStream) {
  const ZeroPolynomial p;
  std::stringstream ss;
  ss << p;
  EXPECT_EQ("Polynomial<MAX_DEGREE=-1>: 0", ss.str());
}

TEST_F(PolynomialTest, DynamicCastDoubleToFloat) {
  const DynamicPolynomial p = {10.9, 3, 6.32432, 1.1};
  const auto p_float = p.cast<float>();
  const auto& double_coeffs = p.getCoefficients();
  const auto& float_coeffs = p_float.getCoefficients();
  for (std::size_t i = 0; i < double_coeffs.size(); i++) {
    EXPECT_NEAR(double_coeffs[i], float_coeffs[i], 1e-6);
  }
}

TEST_F(PolynomialTest, DynamicCastFloatToDouble) {
  const DynamicPolynomialf p = {10.9, 3, 6.32432, 1.1};
  const auto p_double = p.cast<double>();
  const auto& double_coeffs = p_double.getCoefficients();
  const auto& float_coeffs = p.getCoefficients();
  for (std::size_t i = 0; i < double_coeffs.size(); i++) {
    EXPECT_EQ(double_coeffs[i], float_coeffs[i]);
  }
}

TEST_F(PolynomialTest, StaticCastDoubleToFloat) {
  const CubicPolynomial p = {10.9, 3, 6.32432, 1.1};
  const auto p_float = p.cast<float>();
  const auto& double_coeffs = p.getCoefficients();
  const auto& float_coeffs = p_float.getCoefficients();
  for (std::size_t i = 0; i < double_coeffs.size(); i++) {
    EXPECT_NEAR(double_coeffs[i], float_coeffs[i], 1e-6);
  }
}

TEST_F(PolynomialTest, StaticCastFloatToDouble) {
  const CubicPolynomialf p = {10.9, 3, 6.32432, 1.1};
  const auto p_double = p.cast<double>();
  const auto& double_coeffs = p_double.getCoefficients();
  const auto& float_coeffs = p.getCoefficients();
  for (std::size_t i = 0; i < double_coeffs.size(); i++) {
    EXPECT_EQ(double_coeffs[i], float_coeffs[i]);
  }
}

// Bugs

TEST_F(PolynomialTest, DynamicLinearPolynomialSecondDerivativeEvaluation) {
  const DynamicPolynomial polynomial = {1.0, 2.0};
  const DynamicPolynomial derivative = polynomial.calculateSecondDerivative();
  ASSERT_EQ(0, derivative.evaluate(42.0));
}

}  // namespace

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
