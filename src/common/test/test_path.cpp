#include <common/macros.h>

THIRD_PARTY_HEADERS_BEGIN
#include <gtest/gtest.h>
THIRD_PARTY_HEADERS_END
#include "common/path.h"

DISABLE_SUGGEST_OVERRIDE_WARNING

TEST(Path, numberOfPointsForLocalFitMustBeOdd) {
  EXPECT_ANY_THROW(common::Path<>({{0, 0}, {1, 0}, {2, 0}}, {0.4, 0.8}, 6));
  EXPECT_NO_THROW(common::Path<>({{0, 0}, {1, 0}, {2, 0}}, {0.4, 0.8}, 3));
}

TEST(Path, pathPointsMustHaveDistance) {
  EXPECT_ANY_THROW(common::Path<>({{0, 0}, {0.00001, 0}, {0.1, 0}}, {0.4, 0.8}, 3));
  EXPECT_NO_THROW(common::Path<>({{0, 0}, {1, 0}, {2, 0}}, {0.4, 0.8}, 3));
}

TEST(Path, pathPointsMoreAtLeastpolynomialDegree) {
  EXPECT_ANY_THROW(common::Path<>({{0, 0}, {0.1, 0}}, {0.4, 0.8}, 3));
  EXPECT_NO_THROW(common::Path<>({{0, 0}, {1, 0}, {2, 0}}, {0.4, 0.8}, 3));
}

TEST(Path, numberOfPointsForLocalFitMustBeGreaterThanPolynomialDegree) {
  EXPECT_ANY_THROW(common::Path<>({{0, 0}, {1, 0}, {2, 0}}, {0.4, 0.8}, 1));
  EXPECT_NO_THROW(common::Path<>({{0, 0}, {1, 0}, {2, 0}}, {0.4, 0.8}, 3));
}

TEST(Path, closestPointOnPathAfterEnd) {
  const common::Vector2dVector ps = {{0, 0}, {1, 0}, {2, 0}};
  common::Path<> path(ps, {3, 4}, 3);
  auto p = path(0.0);
  EXPECT_DOUBLE_EQ(3.0, p.x());
  EXPECT_DOUBLE_EQ(0.0, p.y());
}

TEST(Path, closestPointOnPathBeforeBegin) {
  const common::Vector2dVector ps = {{0, 0}, {1, 0}, {2, 0}};
  common::Path<> path(ps, {-3, -1}, 3);
  auto p = path(0.0);
  EXPECT_DOUBLE_EQ(-3.0, p.x());
  EXPECT_DOUBLE_EQ(0.0, p.y());
}

TEST(Path, interpolationConstantLine) {
  const common::Vector2dVector ps = {{0, 0}, {1, 0}, {2, 0}};
  common::Path<> path(ps, {0.5, 0}, 3);

  EXPECT_DOUBLE_EQ(0.5, path(0.0).x());
  EXPECT_DOUBLE_EQ(0.0, path(0.0).y());


  EXPECT_DOUBLE_EQ(0.0, path(-0.5).x());
  EXPECT_DOUBLE_EQ(0.0, path(-0.5).y());

  EXPECT_DOUBLE_EQ(0.3, path(-0.2).x());
  EXPECT_DOUBLE_EQ(0.0, path(-0.2).y());

  EXPECT_DOUBLE_EQ(1.5, path(1.0).x());
  EXPECT_DOUBLE_EQ(0.0, path(1.0).y());
}

TEST(Path, interpolationStraightLine) {
  const common::Vector2dVector ps = {{0, 1}, {1, 2}, {2, 3}};
  common::Path<> path(ps, {1, 1}, 3);

  EXPECT_NEAR(0.5, path(0.0).x(), 1e-9);
  EXPECT_NEAR(1.5, path(0.0).y(), 1e-9);

  EXPECT_NEAR(0.0, path(-0.70710678118).x(), 1e-9);
  EXPECT_NEAR(1.0, path(-0.70710678118).y(), 1e-9);

  EXPECT_NEAR(0.3, path(-0.28284271247).x(), 1e-9);
  EXPECT_NEAR(1.3, path(-0.28284271247).y(), 1e-9);

  EXPECT_NEAR(0.7, path(0.28284271247).x(), 1e-9);
  EXPECT_NEAR(1.7, path(0.28284271247).y(), 1e-9);

  EXPECT_NEAR(1.5, path(1.41421356237).x(), 1e-9);
  EXPECT_NEAR(2.5, path(1.41421356237).y(), 1e-9);

  EXPECT_NEAR(2.0, path(2.12132034356).x(), 1e-9);
  EXPECT_NEAR(3.0, path(2.12132034356).y(), 1e-9);
}

TEST(Path, interpolationQuadraticLine) {
  const common::Vector2dVector ps = {{-1, 1}, {0, 0}, {1, 1}};
  common::Path<> path(ps, {0.4, -0.1}, 3);

  EXPECT_NEAR(0.18860, path(0.0).x(), 1e-4);
  EXPECT_NEAR(0.035570, path(0.0).y(), 1e-4);

  EXPECT_NEAR(0.0, path(-0.26672067786).x(), 1e-3);
  EXPECT_NEAR(0.0, path(-0.26672067786).y(), 1e-3);

  EXPECT_NEAR(-1.0, path(-1.68093424023).x(), 1e-3);
  EXPECT_NEAR(1.0, path(-1.68093424023).y(), 1e-3);

  EXPECT_NEAR(1.0, path(1.14749288451).x(), 1e-3);
  EXPECT_NEAR(1.0, path(1.14749288451).y(), 1e-3);
}

TEST(Path, extrapolationStraightLine) {
  const common::Vector2dVector ps = {{0, 1}, {1, 2}, {2, 3}};
  common::Path<> path(ps, {1, 1}, 3);

  EXPECT_NEAR(-1.0, path(-1.70710678118).x(), 1e-9);
  EXPECT_NEAR(0.0, path(-1.70710678118).y(), 1e-9);

  EXPECT_NEAR(3.0, path(3.12132034356).x(), 1e-9);
  EXPECT_NEAR(4.0, path(3.12132034356).y(), 1e-9);
}

TEST(Path, extrapolationQuadraticLine) {
  const common::Vector2dVector ps = {{-1, 1}, {0, 0}, {1, 1}};
  common::Path<> path(ps, {0.4, -0.1}, 3);

  EXPECT_NEAR(-2.0, path(-2.68093424023).x(), 1e-3);
  EXPECT_NEAR(4.0, path(-2.68093424023).y(), 1e-3);

  EXPECT_NEAR(2.0, path(2.14749288451).x(), 1e-3);
  EXPECT_NEAR(4.0, path(2.14749288451).y(), 1e-3);
}

TEST(Path, inRange) {
  const common::Vector2dVector ps = {{-1, 1}, {0, 0}, {1, 1}};
  common::Path<> path(ps, {0.0, 0.0}, 3);
  EXPECT_TRUE(path.isArcLengthInRange(0.0));
  EXPECT_TRUE(path.isArcLengthInRange(0.4));
  EXPECT_TRUE(path.isArcLengthInRange(-0.4));
  EXPECT_TRUE(path.isArcLengthInRange(1.40));
  EXPECT_TRUE(path.isArcLengthInRange(-1.40));
  EXPECT_FALSE(path.isArcLengthInRange(1.42));
  EXPECT_FALSE(path.isArcLengthInRange(-1.42));
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
