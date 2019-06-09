#include "common/polynomial_utils.h"
#include "common/macros.h"

THIRD_PARTY_HEADERS_BEGIN
#include <gtest/gtest.h>
THIRD_PARTY_HEADERS_END

DISABLE_SUGGEST_OVERRIDE_WARNING

TEST(PolynomialUtils, tangentConstantDynamic) {
  const common::DynamicPolynomial p = {0.5};
  for (double x = 0; x < 10; x += 0.1) {
    const Eigen::Vector2d t = common::tangent(p, x);
    EXPECT_EQ(0.0, t.y());
    EXPECT_EQ(1.0, t.x());
  }
}

TEST(PolynomialUtils, tangentLinearDynamic) {
  const common::DynamicPolynomial p = {0.5, 1.0};
  for (double x = 0; x < 10; x += 0.1) {
    const Eigen::Vector2d t = common::tangent(p, x);
    EXPECT_DOUBLE_EQ(std::sqrt(2) / 2.0, t.x());
    EXPECT_DOUBLE_EQ(std::sqrt(2) / 2.0, t.y());
  }
}

TEST(PolynomialUtils, tangentAngleConstantDynamic) {
  const common::DynamicPolynomial p = {0.5};
  for (double x = 0; x < 10; x += 0.1) {
    const double a = common::tangentAngle(p, x);
    EXPECT_EQ(0.0, a);
  }
}

TEST(PolynomialUtils, tangentAngleLinearDynamic) {
  const common::DynamicPolynomial p = {0.5, 1.0};
  for (double x = 0; x < 10; x += 0.1) {
    const double a = common::tangentAngle(p, x);
    EXPECT_DOUBLE_EQ(M_PI / 4.0, a);
  }
}

TEST(PolynomialUtils, normalConstantDynamic) {
  const common::DynamicPolynomial p = {0.5};
  for (double x = 0; x < 10; x += 0.1) {
    const Eigen::Vector2d n = common::normal(p, x);
    EXPECT_EQ(1.0, n.y());
    EXPECT_EQ(0.0, n.x());
  }
}

TEST(PolynomialUtils, normalLinearDynamic) {
  const common::DynamicPolynomial p = {0.5, 1.0};
  for (double x = 0; x < 10; x += 0.1) {
    const Eigen::Vector2d n = common::normal(p, x);
    EXPECT_DOUBLE_EQ(-std::sqrt(2) / 2.0, n.x());
    EXPECT_DOUBLE_EQ(std::sqrt(2) / 2.0, n.y());
  }
}

TEST(PolynomialUtils, normalAngleConstantDynamic) {
  const common::DynamicPolynomial p = {0.5};
  for (double x = 0; x < 10; x += 0.1) {
    const double a = common::normalAngle(p, x);
    EXPECT_EQ(M_PI / 2.0, a);
  }
}

TEST(PolynomialUtils, normalAngleLinearDynamic) {
  const common::DynamicPolynomial p = {0.5, 1.0};
  for (double x = 0; x < 10; x += 0.1) {
    const double a = common::normalAngle(p, x);
    EXPECT_DOUBLE_EQ(3 * M_PI / 4.0, a);
  }
}

TEST(PolynomialUtils, pointConstantDynamic) {
  const common::DynamicPolynomial poly = {0.5};
  for (double x = 0; x < 10; x += 0.1) {
    const Eigen::Vector2d p = common::point(poly, x);
    EXPECT_EQ(0.5, p.y());
    EXPECT_EQ(x, p.x());
  }
}

TEST(PolynomialUtils, pointLinearDynamic) {
  const common::DynamicPolynomial poly = {0.5, 1.0};
  for (double x = 0; x < 10; x += 0.1) {
    const Eigen::Vector2d p = common::point(poly, x);
    EXPECT_DOUBLE_EQ(x, p.x());
    EXPECT_DOUBLE_EQ(x + 0.5, p.y());
  }
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
