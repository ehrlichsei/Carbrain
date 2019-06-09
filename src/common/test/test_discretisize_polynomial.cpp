#include "common/discretisize.h"
#include "common/macros.h"

THIRD_PARTY_HEADERS_BEGIN
#include <gtest/gtest.h>
#include <limits>
THIRD_PARTY_HEADERS_END

#include "common/polynomial.h"

DISABLE_SUGGEST_OVERRIDE_WARNING

TEST(DiscretisizePolynomial, DefaultParametersAreNoOp) {
  const common::DynamicPolynomial p{{1, 1, 1}};
  std::vector<Eigen::Vector2d> points;
  common::discretisize(p, common::DiscretizationParams{}, &points);
  EXPECT_TRUE(points.empty());
}

TEST(DiscretisizePolynomial, ZeroSpanIsNoOp) {
  const common::DynamicPolynomial p{{1, 1, 1}};
  std::vector<Eigen::Vector2d> points;
  common::discretisize(p, {100, 100, 0}, &points);
  EXPECT_TRUE(points.empty());
}

TEST(DiscretisizePolynomial, DeathOnNegativeStepSize) {
  const common::DynamicPolynomial p{{1, 1, 1}};
  std::vector<Eigen::Vector2d> points;
  DISABLE_USED_BUT_MARKED_UNUSED_WARNING
  EXPECT_DEATH(common::discretisize(p, {0, 1, -1}, &points), "step > 0");
  ENABLE_USED_BUT_MARKED_UNUSED_WARNING
}

TEST(DiscretisizePolynomial, DeathOnSwappedStartEnd) {
  const common::DynamicPolynomial p{{1, 1, 1}};
  std::vector<Eigen::Vector2d> points;
  DISABLE_USED_BUT_MARKED_UNUSED_WARNING
  EXPECT_DEATH(common::discretisize(p, {1, 0, 1}, &points),
               "params.max >= params.min");
  ENABLE_USED_BUT_MARKED_UNUSED_WARNING
}

TEST(DiscretisizePolynomial, DeathOnNonFiniteInputs) {
  const common::DynamicPolynomial p{{1, 1, 1}};
  std::vector<Eigen::Vector2d> points;

  DISABLE_USED_BUT_MARKED_UNUSED_WARNING
  EXPECT_DEATH(common::discretisize(
                   p, {0, std::numeric_limits<double>::infinity(), 1}, &points),
               "std::isfinite");
  EXPECT_DEATH(common::discretisize(
                   p, {std::numeric_limits<double>::infinity(), 0, 1}, &points),
               "std::isfinite");
  EXPECT_DEATH(common::discretisize(
                   p, {0, 1, std::numeric_limits<double>::infinity()}, &points),
               "std::isfinite");
  ENABLE_USED_BUT_MARKED_UNUSED_WARNING
}

template <class Polynomial>
void testZeroPolynomial() {
  const Polynomial p;
  std::vector<Eigen::Vector2d> points;
  common::discretisize(p, {0, 10, 1}, &points);
  EXPECT_EQ(10, points.size());

  for (std::size_t i = 0; i < 10; i++) {
    EXPECT_EQ(points[i], Eigen::Vector2d(i, 0));
  }
}

TEST(DiscretisizePolynomial, ZeroPolynomial) {
  testZeroPolynomial<common::ZeroPolynomial>();
  testZeroPolynomial<common::DynamicPolynomial>();
}

template <class Polynomial>
void testConstantPolynomial() {
  const double c = 6;
  const Polynomial p{c};
  std::vector<Eigen::Vector2d> points;
  common::discretisize(p, {-10, 10, 2}, &points);
  EXPECT_EQ(10, points.size());

  for (std::size_t i = 10; i < 10; i++) {
    EXPECT_EQ(Eigen::Vector2d(i * 2 - 10, c), points[i]);
    if (i == 0) {
      continue;
    }
    EXPECT_EQ(2, points[i].x() - points[i - 1].x());
  }
}

TEST(DiscretisizePolynomial, ConstantPolynomial) {
  testConstantPolynomial<common::ConstantPolynomial>();
  testConstantPolynomial<common::DynamicPolynomial>();
}

template <class Polynomial>
void testLinearPolynomial() {
  const Polynomial p{std::vector<double>{{5, 2}}};
  std::vector<Eigen::Vector2d> points;
  common::discretisize(p, {-1, 5, 0.5}, &points);
  EXPECT_EQ(12, points.size());

  for (std::size_t i = 0; i < 12; i++) {
    const double x = i * 0.5 - 1;
    EXPECT_EQ(Eigen::Vector2d(x, p(x)), points[i]);
    if (i == 0) {
      continue;
    }
    EXPECT_EQ(0.5, points[i].x() - points[i - 1].x());
  }
}

TEST(DiscretisizePolynomial, LinearPolynomial) {
  testLinearPolynomial<common::LinearPolynomial>();
  testLinearPolynomial<common::DynamicPolynomial>();
}

template <class Polynomial>
void testCubicPolynomial() {
  const Polynomial p{std::vector<double>{{1, 2, 3, 4}}};
  std::vector<Eigen::Vector2d> points;
  common::discretisize(p, {-10, 5, 0.5}, &points);
  EXPECT_EQ(30, points.size());

  for (std::size_t i = 0; i < 30; i++) {
    const double x = i * 0.5 - 10;
    EXPECT_EQ(Eigen::Vector2d(x, p(x)), points[i]);
  }
}

TEST(DiscretisizePolynomial, CubicPolynomial) {
  testCubicPolynomial<common::CubicPolynomial>();
  testCubicPolynomial<common::DynamicPolynomial>();
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
