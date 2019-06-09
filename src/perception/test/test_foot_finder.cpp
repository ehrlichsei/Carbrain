#include <common/macros.h>

THIRD_PARTY_HEADERS_BEGIN
#include <gtest/gtest.h>
#include <eigen3/Eigen/Dense>
THIRD_PARTY_HEADERS_END

#include "../src/utils/foot_finder.h"
#include "common/polynomial_utils.h"

DISABLE_SUGGEST_OVERRIDE_WARNING

using namespace common;

Eigen::Vector2d createQueryPoint(const common::DynamicPolynomial& polynom,
                                 const double x_target,
                                 const double d) {

  return common::point(polynom, x_target) + d * common::normal(polynom, x_target);
}


TEST(FootFinderTests, trivialTest) {
  double coefficients[] = {0.0, 0.0, 0.1};
  common::DynamicPolynomial polynom(coefficients, common::PolynomialDegrees::Quadratic);
  const auto fp = utils::findLotfusspunkt(polynom, Eigen::Vector2d::Zero());
  EXPECT_DOUBLE_EQ(0.0, fp.x());
  EXPECT_DOUBLE_EQ(0.0, fp.y());
}

TEST(FootFinderTests, zeroTest) {
  double coefficients[] = {0.0, 0.0, 0.0};
  common::DynamicPolynomial polynom(coefficients, common::PolynomialDegrees::Quadratic);
  const double x0 = utils::findLotfusspunktX(polynom, Eigen::Vector2d(5.0, -10.0));
  EXPECT_DOUBLE_EQ(5.0, x0);
}

TEST(FootFinderTests, linearTest) {
  double coefficients[] = {0.0, 1.0, 0.0};
  common::DynamicPolynomial polynom(coefficients, common::PolynomialDegrees::Quadratic);
  const double x0 = utils::findLotfusspunktX(polynom, Eigen::Vector2d(10.0, 11.0));
  EXPECT_DOUBLE_EQ(10.5, x0);
}

TEST(FootFinderTests, cubicTest) {
  double coefficients[] = {0.0, 1.0, 0.0, 1.0};
  common::DynamicPolynomial polynom(coefficients, common::PolynomialDegrees::Cubic);

  double x_target = -10.0;
  Eigen::Vector2d p = createQueryPoint(polynom, x_target, 2.0);
  double x0 = utils::findLotfusspunktX(polynom, p);
  EXPECT_DOUBLE_EQ(x_target, x0);

  x_target = 10.0;
  p = createQueryPoint(polynom, x_target, 2.0);
  x0 = utils::findLotfusspunktX(polynom, p);
  EXPECT_DOUBLE_EQ(x_target, x0);

  x_target = 0.0;
  p = createQueryPoint(polynom, x_target, 2.0);
  x0 = utils::findLotfusspunktX(polynom, p);
  EXPECT_NEAR(x0, x_target, 0.001);
}

TEST(FootFinderTests, exceededMaxIterationsTest) {
  testing::internal::CaptureStdout();
  double coefficients[] = {0.0, 1.0, 0.0};
  common::DynamicPolynomial polynom(coefficients, common::PolynomialDegrees::Quadratic);
  const auto p = Eigen::Vector2d(10.0, 11.0);
  const double x0 = utils::findLotfusspunktX(polynom, p, 0.001, 0);
  // input == output if max_iteration == 0
  EXPECT_DOUBLE_EQ(p.x(), x0);
  // warning which contains "Iteration" is printed if max_iteration is exceeded
  EXPECT_TRUE(testing::internal::GetCapturedStdout().find("Iteration") > 0);
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
