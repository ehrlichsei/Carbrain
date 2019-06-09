#include <common/macros.h>

THIRD_PARTY_HEADERS_BEGIN
#include <gtest/gtest.h>
THIRD_PARTY_HEADERS_END

#include "navigation/line_segment.h"

DISABLE_SUGGEST_OVERRIDE_WARNING

// execute tests with one of the following command:
// catkin_make run_tests_navigation
// catkin_make run_tests_navigation_gtest_line_segment_test

namespace Eigen {

void PrintTo(const Eigen::Vector2d& vector, ::std::ostream* os) {
  *os << "\n" << vector;
}

}  // namespace Eigen

namespace {

class LineSegmentTest : public ::testing::Test {
 public:
  static const Eigen::Vector2d START_POINT;
  static const Eigen::Vector2d END_POINT;
  static const Eigen::Vector2d POINT_A;
  static const Eigen::Vector2d POINT_B;
  static const Eigen::Vector2d POINT_C;
  static const Eigen::Vector2d POINT_D;
  static const Eigen::Vector2d POINT_E;
  static const Eigen::Vector2d POINT_F;
  static const Eigen::Vector2d POINT_G;
  static const Eigen::Vector2d POINT_H;
  static const Eigen::Vector2d POINT_I;
  static const Eigen::Vector2d POINT_J;

  static LineSegment createLineSegment() {
    return LineSegment(START_POINT, END_POINT);
  }
};

//
//     ^
//     |      E+      F+
//  6 -|           o
//     |           |
//  4 -|  A+  B+  I+  C+   D+   Test LineSegment
//     |           |
//  2 -|           o
//     |      G+      H+
//     |---+---+---+---+---+-->
//     |   '   |   '   |   '
//    -2  -1   0   1   2   3
//

//     ^
//     |           ^
//  8 -|           |
//     |      E+  J+  F+
//  6 -|           o
//     |           |
//  4 -|  A+  B+  I+  C+   D+   Test LineSegment intepreted as ray
//     |           |
//  2 -|           o
//     |      G+      H+
//     |---+---+---+---+---+-->
//     |   '   |   '   |   '
//    -2  -1   0   1   2   3
//
const Eigen::Vector2d LineSegmentTest::START_POINT(1, 2);
const Eigen::Vector2d LineSegmentTest::END_POINT(1, 6);
const Eigen::Vector2d LineSegmentTest::POINT_A(-1, 4);
const Eigen::Vector2d LineSegmentTest::POINT_B(0, 4);
const Eigen::Vector2d LineSegmentTest::POINT_C(2, 4);
const Eigen::Vector2d LineSegmentTest::POINT_D(3, 4);
const Eigen::Vector2d LineSegmentTest::POINT_E(0, 7);
const Eigen::Vector2d LineSegmentTest::POINT_F(2, 7);
const Eigen::Vector2d LineSegmentTest::POINT_G(0, 1);
const Eigen::Vector2d LineSegmentTest::POINT_H(2, 1);
const Eigen::Vector2d LineSegmentTest::POINT_I(1, 4);
const Eigen::Vector2d LineSegmentTest::POINT_J(1, 7);

TEST_F(LineSegmentTest, getStartPoint) {
  const LineSegment line_segment = createLineSegment();
  ASSERT_EQ(START_POINT, line_segment.getStartPoint());
}

TEST_F(LineSegmentTest, getRightPole) {
  const LineSegment line_segment = createLineSegment();
  ASSERT_EQ(END_POINT, line_segment.getEndPoint());
}

TEST_F(LineSegmentTest, getLength) {
  const LineSegment line_segment = createLineSegment();
  ASSERT_EQ(6 - 2, line_segment.getLength());
}

TEST_F(LineSegmentTest, intersectionWithNonIntersectingParallelLineSegment) {
  const LineSegment first_line_segment = createLineSegment();
  const Eigen::Translation2d translation(1, 0);
  const LineSegment second_line_segment(translation * START_POINT, translation * END_POINT);
  ASSERT_FALSE(first_line_segment.intersects(second_line_segment));
}

TEST_F(LineSegmentTest, intersectionWithNonIntersectingParallelShiftedLineSegment) {
  const LineSegment first_line_segment = createLineSegment();
  const Eigen::Translation2d translation(1, 3);
  const LineSegment second_line_segment(translation * START_POINT, translation * END_POINT);
  ASSERT_FALSE(first_line_segment.intersects(second_line_segment));
}

TEST_F(LineSegmentTest, intersectionWithIntersectingParallelShiftedLineSegment) {
  const LineSegment first_line_segment = createLineSegment();
  const Eigen::Translation2d translation(0, 2);
  const LineSegment second_line_segment(translation * START_POINT, translation * END_POINT);
  // Parallel lines are considered as none intersecting for numerical reasons
  ASSERT_FALSE(first_line_segment.intersects(second_line_segment));
}

TEST_F(LineSegmentTest, intersectionWithIntersectingLineSegment) {
  const LineSegment first_line_segment = createLineSegment();
  const LineSegment second_line_segment(POINT_B, POINT_C);
  ASSERT_TRUE(first_line_segment.intersects(second_line_segment));
}

TEST_F(LineSegmentTest, intersectionWithNonIntersectingLineSegmentLeft) {
  const LineSegment first_line_segment = createLineSegment();
  const LineSegment second_line_segment(POINT_A, POINT_B);
  ASSERT_FALSE(first_line_segment.intersects(second_line_segment));
}

TEST_F(LineSegmentTest, intersectionWithNonIntersectingLineSegmentRight) {
  const LineSegment first_line_segment = createLineSegment();
  const LineSegment second_line_segment(POINT_C, POINT_D);
  ASSERT_FALSE(first_line_segment.intersects(second_line_segment));
}

TEST_F(LineSegmentTest, intersectionWithNonIntersectingLineSegmentBelow) {
  const LineSegment first_line_segment = createLineSegment();
  const LineSegment second_line_segment(POINT_G, POINT_H);
  ASSERT_FALSE(first_line_segment.intersects(second_line_segment));
}

TEST_F(LineSegmentTest, intersectionWithNonIntersectingLineSegmentAbove) {
  const LineSegment first_line_segment = createLineSegment();
  const LineSegment second_line_segment(POINT_E, POINT_F);
  ASSERT_FALSE(first_line_segment.intersects(second_line_segment));
}

TEST_F(LineSegmentTest, intersectionWithIntersectingLineSegmentDiagonal) {
  const LineSegment first_line_segment = createLineSegment();
  const LineSegment second_line_segment(POINT_E, POINT_H);
  ASSERT_TRUE(first_line_segment.intersects(second_line_segment));
}

TEST_F(LineSegmentTest, intersectionWithIdenticalLineSegment) {
  const LineSegment first_line_segment = createLineSegment();
  const LineSegment second_line_segment = createLineSegment();
  // Parallel lines are considered as none intersecting for numerical reasons
  ASSERT_FALSE(first_line_segment.intersects(second_line_segment));
}

TEST_F(LineSegmentTest, intersectionPointWithIntersectingLineSegment) {
  const LineSegment first_line_segment = createLineSegment();
  const LineSegment second_line_segment(POINT_B, POINT_C);
  Eigen::Vector2d intersection_point;
  first_line_segment.intersects(second_line_segment, &intersection_point);
  ASSERT_EQ(POINT_I, intersection_point);
}

// Ray

TEST_F(LineSegmentTest, rayIntersectionWithNonIntersectingParallelLineSegment) {
  const LineSegment first_line_segment = createLineSegment();
  const Eigen::Translation2d translation(1, 0);
  const LineSegment second_line_segment(translation * START_POINT, translation * END_POINT);
  ASSERT_FALSE(first_line_segment.intersectsRay(second_line_segment));
}

TEST_F(LineSegmentTest, rayIntersectionWithNonIntersectingParallelShiftedLineSegment) {
  const LineSegment first_line_segment = createLineSegment();
  const Eigen::Translation2d translation(1, 3);
  const LineSegment second_line_segment(translation * START_POINT, translation * END_POINT);
  ASSERT_FALSE(first_line_segment.intersectsRay(second_line_segment));
}

TEST_F(LineSegmentTest, rayIntersectionWithIntersectingParallelShiftedLineSegment) {
  const LineSegment first_line_segment = createLineSegment();
  const Eigen::Translation2d translation(0, 2);
  const LineSegment second_line_segment(translation * START_POINT, translation * END_POINT);
  // Parallel lines are considered as none intersecting for numerical reasons
  ASSERT_FALSE(first_line_segment.intersectsRay(second_line_segment));
}

TEST_F(LineSegmentTest, rayIntersectionWithIntersectingLineSegment) {
  const LineSegment first_line_segment = createLineSegment();
  const LineSegment second_line_segment(POINT_B, POINT_C);
  ASSERT_TRUE(first_line_segment.intersectsRay(second_line_segment));
}

TEST_F(LineSegmentTest, rayIntersectionWithNonIntersectingLineSegmentLeft) {
  const LineSegment first_line_segment = createLineSegment();
  const LineSegment second_line_segment(POINT_A, POINT_B);
  ASSERT_FALSE(first_line_segment.intersectsRay(second_line_segment));
}

TEST_F(LineSegmentTest, rayIntersectionWithNonIntersectingLineSegmentRight) {
  const LineSegment first_line_segment = createLineSegment();
  const LineSegment second_line_segment(POINT_C, POINT_D);
  ASSERT_FALSE(first_line_segment.intersectsRay(second_line_segment));
}

TEST_F(LineSegmentTest, rayIntersectionWithNonIntersectingLineSegmentBelow) {
  const LineSegment first_line_segment = createLineSegment();
  const LineSegment second_line_segment(POINT_G, POINT_H);
  ASSERT_FALSE(first_line_segment.intersectsRay(second_line_segment));
}

TEST_F(LineSegmentTest, rayIntersectionWithIntersectingLineSegmentAbove) {
  const LineSegment first_line_segment = createLineSegment();
  const LineSegment second_line_segment(POINT_E, POINT_F);
  ASSERT_TRUE(first_line_segment.intersectsRay(second_line_segment));
}

TEST_F(LineSegmentTest, rayIntersectionWithIntersectingLineSegmentDiagonal) {
  const LineSegment first_line_segment = createLineSegment();
  const LineSegment second_line_segment(POINT_E, POINT_H);
  ASSERT_TRUE(first_line_segment.intersectsRay(second_line_segment));
}

TEST_F(LineSegmentTest, rayIntersectionWithIdenticalLineSegment) {
  const LineSegment first_line_segment = createLineSegment();
  const LineSegment second_line_segment = createLineSegment();
  // Parallel lines are considered as none intersecting for numerical reasons
  ASSERT_FALSE(first_line_segment.intersectsRay(second_line_segment));
}

TEST_F(LineSegmentTest, rayIntersectionPointWithIntersectingLineSegment) {
  const LineSegment first_line_segment = createLineSegment();
  const LineSegment second_line_segment(POINT_B, POINT_C);
  Eigen::Vector2d intersection_point;
  first_line_segment.intersectsRay(second_line_segment, &intersection_point);
  ASSERT_EQ(POINT_I, intersection_point);
}

TEST_F(LineSegmentTest, rayIntersectionPointWithIntersectingLineSegmentAbove) {
  const LineSegment first_line_segment = createLineSegment();
  const LineSegment second_line_segment(POINT_E, POINT_F);
  Eigen::Vector2d intersection_point;
  first_line_segment.intersectsRay(second_line_segment, &intersection_point);
  ASSERT_EQ(POINT_J, intersection_point);
}

//
//     ^
//     |      E+      F+
//  6 -|           o
//     |           |
//  4 -|  A+  B+  I+  C+   D+   Test LineSegment
//     |           |
//  2 -|           o
//     |      G+      H+
//     |---+---+---+---+---+-->
//     |   '   |   '   |   '
//    -2  -1   0   1   2   3
//

TEST_F(LineSegmentTest, isPointAOnRightSide) {
  const LineSegment line_segment = createLineSegment();
  ASSERT_FALSE(line_segment.isPointOnRightSide(POINT_A));
}

TEST_F(LineSegmentTest, isPointIOnRightSide) {
  const LineSegment line_segment = createLineSegment();
  ASSERT_FALSE(line_segment.isPointOnRightSide(POINT_I));
}

TEST_F(LineSegmentTest, isPointCOnRightSide) {
  const LineSegment line_segment = createLineSegment();
  ASSERT_TRUE(line_segment.isPointOnRightSide(POINT_C));
}

TEST_F(LineSegmentTest, isPointEOnRightSide) {
  const LineSegment line_segment = createLineSegment();
  ASSERT_FALSE(line_segment.isPointOnRightSide(POINT_E));
}

TEST_F(LineSegmentTest, isPointFOnRightSide) {
  const LineSegment line_segment = createLineSegment();
  ASSERT_TRUE(line_segment.isPointOnRightSide(POINT_F));
}

TEST_F(LineSegmentTest, isPointAOnLeftSide) {
  const LineSegment line_segment = createLineSegment();
  ASSERT_TRUE(line_segment.isPointOnLeftSide(POINT_A));
}

TEST_F(LineSegmentTest, isPointIOnLeftSide) {
  const LineSegment line_segment = createLineSegment();
  ASSERT_FALSE(line_segment.isPointOnLeftSide(POINT_I));
}

TEST_F(LineSegmentTest, isPointCOnLeftSide) {
  const LineSegment line_segment = createLineSegment();
  ASSERT_FALSE(line_segment.isPointOnLeftSide(POINT_C));
}

TEST_F(LineSegmentTest, isPointEOnLeftSide) {
  const LineSegment line_segment = createLineSegment();
  ASSERT_TRUE(line_segment.isPointOnLeftSide(POINT_E));
}

TEST_F(LineSegmentTest, isPointFOnLeftSide) {
  const LineSegment line_segment = createLineSegment();
  ASSERT_FALSE(line_segment.isPointOnLeftSide(POINT_F));
}

TEST_F(LineSegmentTest, containsProjectedPointOfPointA) {
  const LineSegment line_segment = createLineSegment();
  ASSERT_TRUE(line_segment.containsProjectedPoint(POINT_A));
}

TEST_F(LineSegmentTest, containsProjectedPointOfPointI) {
  const LineSegment line_segment = createLineSegment();
  ASSERT_TRUE(line_segment.containsProjectedPoint(POINT_I));
}

TEST_F(LineSegmentTest, containsProjectedPointOfPointC) {
  const LineSegment line_segment = createLineSegment();
  ASSERT_TRUE(line_segment.containsProjectedPoint(POINT_C));
}

TEST_F(LineSegmentTest, containsProjectedPointOfPointE) {
  const LineSegment line_segment = createLineSegment();
  ASSERT_FALSE(line_segment.containsProjectedPoint(POINT_E));
}

TEST_F(LineSegmentTest, containsProjectedPointOfPointF) {
  const LineSegment line_segment = createLineSegment();
  ASSERT_FALSE(line_segment.containsProjectedPoint(POINT_F));
}

TEST_F(LineSegmentTest, projectedPointOfPointA) {
  const LineSegment line_segment = createLineSegment();
  Eigen::Vector2d projected_point;
  line_segment.containsProjectedPoint(POINT_A, &projected_point);
  ASSERT_EQ(POINT_I, projected_point);
}

TEST_F(LineSegmentTest, projectedPointOfPointI) {
  const LineSegment line_segment = createLineSegment();
  Eigen::Vector2d projected_point;
  line_segment.containsProjectedPoint(POINT_I, &projected_point);
  ASSERT_EQ(POINT_I, projected_point);
}

TEST_F(LineSegmentTest, projectedPointOfPointC) {
  const LineSegment line_segment = createLineSegment();
  Eigen::Vector2d projected_point;
  line_segment.containsProjectedPoint(POINT_C, &projected_point);
  ASSERT_EQ(POINT_I, projected_point);
}

TEST_F(LineSegmentTest, swapEndpoints) {
  LineSegment line_segment = createLineSegment();
  ASSERT_TRUE(line_segment.isPointOnLeftSide(POINT_A));
  line_segment.swapEndpoints();
  ASSERT_TRUE(line_segment.isPointOnRightSide(POINT_A));
}

TEST_F(LineSegmentTest, transform) {
  LineSegment line_segment = createLineSegment();
  const Eigen::Affine2d transformation(Eigen::Rotation2D<double>(M_PI));
  line_segment.transform(transformation);
  ASSERT_EQ(transformation * START_POINT, line_segment.getStartPoint());
  ASSERT_EQ(transformation * END_POINT, line_segment.getEndPoint());
}

TEST_F(LineSegmentTest, swapEndpointsReturnThis) {
  LineSegment line_segment = createLineSegment();
  ;
  ASSERT_EQ(&line_segment, &(line_segment.swapEndpoints()));
}

TEST_F(LineSegmentTest, transformReturnThis) {
  LineSegment line_segment = createLineSegment();
  const Eigen::Affine2d transformation(Eigen::Rotation2D<double>(M_PI));
  ASSERT_EQ(&line_segment, &(line_segment.transform(transformation)));
}

TEST_F(LineSegmentTest, swappedEndpoints) {
  LineSegment line_segment = createLineSegment();
  ASSERT_TRUE(line_segment.isPointOnLeftSide(POINT_A));
  ASSERT_TRUE(line_segment.swappedEndpoints().isPointOnRightSide(POINT_A));
}

TEST_F(LineSegmentTest, transformed) {
  LineSegment line_segment = createLineSegment();
  const Eigen::Affine2d transformation(Eigen::Rotation2D<double>(M_PI));
  ASSERT_EQ(transformation * START_POINT,
            line_segment.transformed(transformation).getStartPoint());
  ASSERT_EQ(transformation * END_POINT,
            line_segment.transformed(transformation).getEndPoint());
}

}  // namespace

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
