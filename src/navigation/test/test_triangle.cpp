#include <common/macros.h>

THIRD_PARTY_HEADERS_BEGIN
#include <gtest/gtest.h>
THIRD_PARTY_HEADERS_END

#include "navigation/triangle.h"

DISABLE_SUGGEST_OVERRIDE_WARNING

// execute tests with one of the following command:
// catkin_make run_tests_navigation
// catkin_make run_tests_navigation_gtest_triangle_test

namespace Eigen {

void PrintTo(const Eigen::Vector2d& vector, ::std::ostream* os) {
  *os << "\n" << vector;
}

}  // namespace Eigen

namespace {

class TriangleTest : public ::testing::Test {
 public:
  static const Eigen::Vector2d POINT_A;
  static const Eigen::Vector2d POINT_B;
  static const Eigen::Vector2d POINT_C;
  static const Eigen::Vector2d POINT_D;
  static const Eigen::Vector2d POINT_E;
  static const Eigen::Vector2d POINT_F;
  static const Eigen::Vector2d POINT_G;

  static Triangle createTriangle() {
    return Triangle(POINT_A, POINT_B, POINT_C);
  }
};

/*
 *     ^
 *     |           A
 *  6 -|           o
 *     |          / \
 *  4 -|  D+     / + \    F+  Test Triangle
 *     |        /  E  \
 *  2 -|       o---+---o
 *     |      B    G    C
 *     |---+---+---+---+---+-->
 *     |   '   |   '   |   '
 *    -2  -1   0   1   2   3
 */
const Eigen::Vector2d TriangleTest::POINT_A(1, 6);
const Eigen::Vector2d TriangleTest::POINT_B(0, 2);
const Eigen::Vector2d TriangleTest::POINT_C(2, 2);
const Eigen::Vector2d TriangleTest::POINT_D(-1, 4);
const Eigen::Vector2d TriangleTest::POINT_E(1, 4);
const Eigen::Vector2d TriangleTest::POINT_F(3, 4);
const Eigen::Vector2d TriangleTest::POINT_G(1, 2);

TEST_F(TriangleTest, getCornerPoints) {
  const Triangle triangle = createTriangle();
  const Triangle::CornerPoints points = triangle.getCornerPoints();
  ASSERT_EQ(3, points.size());
  ASSERT_EQ(POINT_A, points[0]);
  ASSERT_EQ(POINT_B, points[1]);
  ASSERT_EQ(POINT_C, points[2]);
}

TEST_F(TriangleTest, isPointOutsideCornerPoints) {
  const Triangle triangle = createTriangle();
  ASSERT_FALSE(triangle.isPointOutside(POINT_A));
  ASSERT_FALSE(triangle.isPointOutside(POINT_B));
  ASSERT_FALSE(triangle.isPointOutside(POINT_C));
}

TEST_F(TriangleTest, isPointOutsidePointD) {
  const Triangle triangle = createTriangle();
  ASSERT_TRUE(triangle.isPointOutside(POINT_D));
}

TEST_F(TriangleTest, isPointOutsidePointE) {
  const Triangle triangle = createTriangle();
  ASSERT_FALSE(triangle.isPointOutside(POINT_E));
}

TEST_F(TriangleTest, isPointOutsidePointF) {
  const Triangle triangle = createTriangle();
  ASSERT_TRUE(triangle.isPointOutside(POINT_F));
}

TEST_F(TriangleTest, isPointOutsidePointG) {
  const Triangle triangle = createTriangle();
  ASSERT_FALSE(triangle.isPointOutside(POINT_G));
}

TEST_F(TriangleTest, isPointInsideCornerPoints) {
  const Triangle triangle = createTriangle();
  ASSERT_FALSE(triangle.isPointInside(POINT_A));
  ASSERT_FALSE(triangle.isPointInside(POINT_B));
  ASSERT_FALSE(triangle.isPointInside(POINT_C));
}

TEST_F(TriangleTest, isPointInsidePointD) {
  const Triangle triangle = createTriangle();
  ASSERT_FALSE(triangle.isPointInside(POINT_D));
}

TEST_F(TriangleTest, isPointInsidePointE) {
  const Triangle triangle = createTriangle();
  ASSERT_TRUE(triangle.isPointInside(POINT_E));
}

TEST_F(TriangleTest, isPointInsidePointF) {
  const Triangle triangle = createTriangle();
  ASSERT_FALSE(triangle.isPointInside(POINT_F));
}

TEST_F(TriangleTest, isPointInsidePointG) {
  const Triangle triangle = createTriangle();
  ASSERT_FALSE(triangle.isPointInside(POINT_G));
}

}  // namespace

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
