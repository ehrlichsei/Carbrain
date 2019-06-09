#include "common/math.h"
#include "common/macros.h"

THIRD_PARTY_HEADERS_BEGIN
#include <gtest/gtest.h>
THIRD_PARTY_HEADERS_END

DISABLE_SUGGEST_OVERRIDE_WARNING

using namespace common::math;

TEST(Sgn, integer) {
  EXPECT_EQ(sgn(std::numeric_limits<long long>::lowest()), -1);
  EXPECT_EQ(sgn(std::numeric_limits<int>::lowest()), -1);
  EXPECT_EQ(sgn(-9), -1);
  EXPECT_EQ(sgn(-8), -1);
  EXPECT_EQ(sgn(-2), -1);
  EXPECT_EQ(sgn(-1), -1);
  EXPECT_EQ(sgn(0), 0);
  EXPECT_EQ(sgn(1), 1);
  EXPECT_EQ(sgn(2), 1);
  EXPECT_EQ(sgn(8), 1);
  EXPECT_EQ(sgn(9), 1);
  EXPECT_EQ(sgn(std::numeric_limits<int>::max()), 1);
  EXPECT_EQ(sgn(std::numeric_limits<long long>::max()), 1);
  EXPECT_EQ(sgn(std::numeric_limits<unsigned long long>::max()), 1);
}

TEST(Sgn, floatingPoint) {
  EXPECT_EQ(sgn(-std::numeric_limits<float>::infinity()), -1);
  EXPECT_EQ(sgn(-std::numeric_limits<double>::infinity()), -1);
  EXPECT_EQ(sgn(std::numeric_limits<float>::lowest()), -1);
  EXPECT_EQ(sgn(std::numeric_limits<double>::lowest()), -1);
  EXPECT_EQ(sgn(-9.0), -1);
  EXPECT_EQ(sgn(-8.0), -1);
  EXPECT_EQ(sgn(-2.0), -1);
  EXPECT_EQ(sgn(-1.0), -1);
  EXPECT_EQ(sgn(-0.4), -1);
  EXPECT_EQ(sgn(0.0), 0);
  EXPECT_EQ(sgn(0.4), 1);
  EXPECT_EQ(sgn(1.0), 1);
  EXPECT_EQ(sgn(2.0), 1);
  EXPECT_EQ(sgn(8.0), 1);
  EXPECT_EQ(sgn(9.0), 1);
  EXPECT_EQ(sgn(std::numeric_limits<float>::max()), 1);
  EXPECT_EQ(sgn(std::numeric_limits<double>::max()), 1);
  EXPECT_EQ(sgn(std::numeric_limits<float>::infinity()), 1);
  EXPECT_EQ(sgn(std::numeric_limits<double>::infinity()), 1);
}

TEST(Squared, integer) {
  EXPECT_EQ(squared(-3), 9);
  EXPECT_EQ(squared(-2), 4);
  EXPECT_EQ(squared(-1), 1);
  EXPECT_EQ(squared(0), 0);
  EXPECT_EQ(squared(1), 1);
  EXPECT_EQ(squared(2), 4);
  EXPECT_EQ(squared(3), 9);
}

TEST(Squared, floatingPoint) {
  EXPECT_DOUBLE_EQ(squared(-3.0), 9.0);
  EXPECT_DOUBLE_EQ(squared(-2.0), 4.0);
  EXPECT_DOUBLE_EQ(squared(0.0), 0.0);
  EXPECT_DOUBLE_EQ(squared(2.0), 4.0);
  EXPECT_DOUBLE_EQ(squared(3.0), 9.0);

  EXPECT_DOUBLE_EQ(squared(std::numeric_limits<double>::infinity()),
                   std::numeric_limits<double>::infinity());
  EXPECT_DOUBLE_EQ(squared(-std::numeric_limits<double>::infinity()),
                   std::numeric_limits<double>::infinity());
  EXPECT_DOUBLE_EQ(squared(std::numeric_limits<float>::infinity()),
                   std::numeric_limits<float>::infinity());
  EXPECT_DOUBLE_EQ(squared(-std::numeric_limits<float>::infinity()),
                   std::numeric_limits<float>::infinity());

  EXPECT_TRUE(std::isnan(squared(std::numeric_limits<double>::quiet_NaN())));
}

TEST(Modulo, integer) {
  EXPECT_EQ(positiveModulo(-21, 4), 3);
  EXPECT_EQ(positiveModulo(-22, 4), 2);
  EXPECT_EQ(positiveModulo(-16, 3), 2);
  EXPECT_EQ(positiveModulo(-15, 3), 0);
  EXPECT_EQ(positiveModulo(-3, 1), 0);

  EXPECT_EQ(positiveModulo(21, -4), -3);
  EXPECT_EQ(positiveModulo(22, -4), -2);
  EXPECT_EQ(positiveModulo(16, -3), -2);
  EXPECT_EQ(positiveModulo(15, -3), 0);
  EXPECT_EQ(positiveModulo(3, -1), 0);

  EXPECT_EQ(positiveModulo(21, 4), 1);
  EXPECT_EQ(positiveModulo(22, 4), 2);
  EXPECT_EQ(positiveModulo(16, 3), 1);
  EXPECT_EQ(positiveModulo(15, 3), 0);
  EXPECT_EQ(positiveModulo(3, 1), 0);

  EXPECT_EQ(positiveModulo(-21, -4), -1);
  EXPECT_EQ(positiveModulo(-22, -4), -2);
  EXPECT_EQ(positiveModulo(-16, -3), -1);
  EXPECT_EQ(positiveModulo(-15, -3), 0);
  EXPECT_EQ(positiveModulo(-3, -1), 0);

  EXPECT_EQ(positiveModulo(0, 4), 0);
  EXPECT_EQ(positiveModulo(0, -4), 0);


  EXPECT_EQ(positiveModulo(22L, 4), 2);
  EXPECT_EQ(positiveModulo(22L, 4L), 2);
  EXPECT_EQ(positiveModulo(22, 4L), 2);
}

TEST(Modulo, floatingPoint) {
  constexpr double max_error = 1e-6;

  EXPECT_NEAR(positiveModulo(-21.0, 4.0), 3.0, max_error);
  EXPECT_NEAR(positiveModulo(-22.0, 4.0), 2.0, max_error);
  EXPECT_NEAR(positiveModulo(-16.0, 3.0), 2.0, max_error);
  EXPECT_NEAR(positiveModulo(-15.0, 3.0), 0.0, max_error);
  EXPECT_NEAR(positiveModulo(-3.0, 1.0), 0.0, max_error);

  EXPECT_NEAR(positiveModulo(21.0, -4.0), -3.0, max_error);
  EXPECT_NEAR(positiveModulo(22.0, -4.0), -2.0, max_error);
  EXPECT_NEAR(positiveModulo(16.0, -3.0), -2.0, max_error);
  EXPECT_NEAR(positiveModulo(15.0, -3.0), 0.0, max_error);
  EXPECT_NEAR(positiveModulo(3.0, -1.0), 0.0, max_error);

  EXPECT_NEAR(positiveModulo(21.0, 4.0), 1.0, max_error);
  EXPECT_NEAR(positiveModulo(22.0, 4.0), 2.0, max_error);
  EXPECT_NEAR(positiveModulo(16.0, 3.0), 1.0, max_error);
  EXPECT_NEAR(positiveModulo(15.0, 3.0), 0.0, max_error);
  EXPECT_NEAR(positiveModulo(3.0, 1.0), 0.0, max_error);

  EXPECT_NEAR(positiveModulo(-21.0, -4.0), -1.0, max_error);
  EXPECT_NEAR(positiveModulo(-22.0, -4.0), -2.0, max_error);
  EXPECT_NEAR(positiveModulo(-16.0, -3.0), -1.0, max_error);
  EXPECT_NEAR(positiveModulo(-15.0, -3.0), 0.0, max_error);
  EXPECT_NEAR(positiveModulo(-3.0, -1.0), 0.0, max_error);

  EXPECT_NEAR(positiveModulo(0.0, 4.0), 0.0, max_error);
  EXPECT_NEAR(positiveModulo(0.0, -4.0), 0.0, max_error);

  EXPECT_NEAR(positiveModulo(21.4, 4.3), 4.2, max_error);
  EXPECT_NEAR(positiveModulo(22.2, -4.5), -0.3, max_error);
  EXPECT_NEAR(positiveModulo(-16.3, 3.4), 0.7, max_error);
  EXPECT_NEAR(positiveModulo(-15.5, -3.3), -2.3, max_error);
  EXPECT_NEAR(positiveModulo(8.8, -2.2), 0.0, max_error);

  EXPECT_TRUE(std::isnan(positiveModulo(std::numeric_limits<double>::quiet_NaN(), 4.0)));
  EXPECT_TRUE(std::isnan(positiveModulo(std::numeric_limits<double>::quiet_NaN(), 0.0)));
  EXPECT_TRUE(std::isnan(positiveModulo(0.0, std::numeric_limits<double>::quiet_NaN())));
  EXPECT_TRUE(std::isnan(positiveModulo(4.0, std::numeric_limits<double>::quiet_NaN())));

  EXPECT_TRUE(std::isnan(positiveModulo(std::numeric_limits<double>::infinity(), -4.0)));
  EXPECT_TRUE(std::isnan(positiveModulo(std::numeric_limits<double>::infinity(), 0.0)));
  EXPECT_TRUE(std::isnan(positiveModulo(std::numeric_limits<double>::infinity(), 4.0)));

  EXPECT_TRUE(std::isnan(positiveModulo(-std::numeric_limits<double>::infinity(), -4.0)));
  EXPECT_TRUE(std::isnan(positiveModulo(-std::numeric_limits<double>::infinity(), 0.0)));
  EXPECT_TRUE(std::isnan(positiveModulo(-std::numeric_limits<double>::infinity(), 4.0)));

  EXPECT_TRUE(std::isnan(positiveModulo(std::numeric_limits<double>::infinity(),
                                        std::numeric_limits<double>::infinity())));
  EXPECT_TRUE(std::isnan(positiveModulo(-std::numeric_limits<double>::infinity(),
                                        std::numeric_limits<double>::infinity())));
  EXPECT_TRUE(std::isnan(positiveModulo(std::numeric_limits<double>::infinity(),
                                        -std::numeric_limits<double>::infinity())));
  EXPECT_TRUE(std::isnan(positiveModulo(-std::numeric_limits<double>::infinity(),
                                        -std::numeric_limits<double>::infinity())));

  EXPECT_TRUE(std::isnan(positiveModulo(4.0, 0.0)));
  EXPECT_TRUE(std::isnan(positiveModulo(-4.0, 0.0)));
  EXPECT_TRUE(std::isnan(positiveModulo(4.0, -0.0)));
  EXPECT_TRUE(std::isnan(positiveModulo(-4.0, -0.0)));


  EXPECT_NEAR(positiveModulo(-16.3f, 3.4), 0.7, max_error);
  EXPECT_NEAR(positiveModulo(-16.3f, 3.4f), 0.7, max_error);
  EXPECT_NEAR(positiveModulo(-16.3, 3.4f), 0.7, max_error);
  EXPECT_NEAR(positiveModulo(22.4, 4.0f), 2.4, max_error);
  EXPECT_NEAR(positiveModulo(22.4, 4), 2.4, max_error);
  EXPECT_NEAR(positiveModulo(22.4f, 4), 2.4, max_error);
  EXPECT_NEAR(positiveModulo(22.4, 4L), 2.4, max_error);
  EXPECT_NEAR(positiveModulo(22.4f, 4L), 2.4, max_error);
  EXPECT_NEAR(positiveModulo(22, 4.0), 2.0, max_error);
  EXPECT_NEAR(positiveModulo(22L, 4.0), 2.0, max_error);
  EXPECT_NEAR(positiveModulo(22L, 4.0f), 2.0, max_error);
  EXPECT_NEAR(positiveModulo(22u, 4.0), 2.0, max_error);
}

TEST(WrapAngle, minusPiToPi) {
  constexpr double max_error = 1e-6;
  EXPECT_NEAR(wrapAngleMinusPiToPi(-800.4 * M_PI), -0.4 * M_PI, max_error);
  EXPECT_NEAR(wrapAngleMinusPiToPi(-0.5 * M_PI), -0.5 * M_PI, max_error);
  EXPECT_NEAR(wrapAngleMinusPiToPi(-2 * M_PI), 0.0, max_error);
  EXPECT_NEAR(wrapAngleMinusPiToPi(-M_PI), -M_PI, max_error);
  EXPECT_NEAR(wrapAngleMinusPiToPi(0.0), 0.0, max_error);
  EXPECT_NEAR(wrapAngleMinusPiToPi(0.5 * M_PI), 0.5 * M_PI, max_error);
  EXPECT_NEAR(wrapAngleMinusPiToPi(M_PI), -M_PI, max_error);
  EXPECT_NEAR(wrapAngleMinusPiToPi(2 * M_PI), 0.0, max_error);
  EXPECT_NEAR(wrapAngleMinusPiToPi(800.4 * M_PI), 0.4 * M_PI, max_error);

  EXPECT_TRUE(std::isnan(wrapAngleMinusPiToPi(std::numeric_limits<float>::quiet_NaN())));
  EXPECT_TRUE(std::isnan(wrapAngleMinusPiToPi(std::numeric_limits<double>::quiet_NaN())));

  EXPECT_TRUE(std::isnan(wrapAngleMinusPiToPi(-std::numeric_limits<float>::infinity())));
  EXPECT_TRUE(std::isnan(wrapAngleMinusPiToPi(std::numeric_limits<float>::infinity())));
  EXPECT_TRUE(std::isnan(wrapAngleMinusPiToPi(-std::numeric_limits<double>::infinity())));
  EXPECT_TRUE(std::isnan(wrapAngleMinusPiToPi(std::numeric_limits<double>::infinity())));
}

TEST(WrapAngle, zeroToTwoPi) {
  constexpr double max_error = 1e-6;
  EXPECT_NEAR(wrapAngleZeroToTwoPi(-800.4 * M_PI), 1.6 * M_PI, max_error);
  EXPECT_NEAR(wrapAngleZeroToTwoPi(-0.5 * M_PI), 1.5 * M_PI, max_error);
  EXPECT_NEAR(wrapAngleZeroToTwoPi(-2 * M_PI), 0.0, max_error);
  EXPECT_NEAR(wrapAngleZeroToTwoPi(-M_PI), M_PI, max_error);
  EXPECT_NEAR(wrapAngleZeroToTwoPi(0.0), 0.0, max_error);
  EXPECT_NEAR(wrapAngleZeroToTwoPi(0.5 * M_PI), 0.5 * M_PI, max_error);
  EXPECT_NEAR(wrapAngleZeroToTwoPi(M_PI), M_PI, max_error);
  EXPECT_NEAR(wrapAngleZeroToTwoPi(2 * M_PI), 0.0, max_error);
  EXPECT_NEAR(wrapAngleZeroToTwoPi(800.4 * M_PI), 0.4 * M_PI, max_error);

  EXPECT_TRUE(std::isnan(wrapAngleZeroToTwoPi(std::numeric_limits<float>::quiet_NaN())));
  EXPECT_TRUE(std::isnan(wrapAngleZeroToTwoPi(std::numeric_limits<double>::quiet_NaN())));

  EXPECT_TRUE(std::isnan(wrapAngleZeroToTwoPi(-std::numeric_limits<float>::infinity())));
  EXPECT_TRUE(std::isnan(wrapAngleZeroToTwoPi(std::numeric_limits<float>::infinity())));
  EXPECT_TRUE(std::isnan(wrapAngleZeroToTwoPi(-std::numeric_limits<double>::infinity())));
  EXPECT_TRUE(std::isnan(wrapAngleZeroToTwoPi(std::numeric_limits<double>::infinity())));
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
