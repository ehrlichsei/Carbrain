#include <common/macros.h>

THIRD_PARTY_HEADERS_BEGIN
#include <gtest/gtest.h>
THIRD_PARTY_HEADERS_END

#include "../src/path_preprocessing/boundingbox.hpp"
DISABLE_SUGGEST_OVERRIDE_WARNING

TEST(BoundingBoxTest, defaultInitilizesZero) {
  const BoundingBox bb;
  EXPECT_EQ(Eigen::Vector2d::Zero(), bb.getMin());
  EXPECT_EQ(Eigen::Vector2d::Zero(), bb.getMax());
  EXPECT_EQ(Eigen::Vector2d::Zero(), bb.getCenter());
}

TEST(BoundingBoxTest, assertNonEmptyPointVector) {
  DISABLE_USED_BUT_MARKED_UNUSED_WARNING
  EXPECT_DEATH(BoundingBox(std::vector<Eigen::Vector2d>()), "!points.empty()");
  ENABLE_USED_BUT_MARKED_UNUSED_WARNING
}

TEST(BoundingBoxTest, constructSinglePoint) {
  const BoundingBox bb(std::vector<Eigen::Vector2d>({{1.0, 2.0}}));
  EXPECT_EQ(Eigen::Vector2d(1.0, 2.0), bb.getMin());
  EXPECT_EQ(Eigen::Vector2d(1.0, 2.0), bb.getMax());
  EXPECT_EQ(Eigen::Vector2d(1.0, 2.0), bb.getCenter());
}

TEST(BoundingBoxTest, constructMultiPoint) {
  const BoundingBox bb(std::vector<Eigen::Vector2d>({{1.0, 2.0}, {0.5, 4.0}}));
  EXPECT_EQ(Eigen::Vector2d(0.5, 2.0), bb.getMin());
  EXPECT_EQ(Eigen::Vector2d(1.0, 4.0), bb.getMax());
  EXPECT_EQ(Eigen::Vector2d(0.75, 3.0), bb.getCenter());
}

TEST(BoundingBoxTest, enlargePoints) {
  BoundingBox bb(std::vector<Eigen::Vector2d>({{1.0, 2.0}}));
  bb.enlarge({0.5, 4.0});
  EXPECT_EQ(Eigen::Vector2d(0.5, 2.0), bb.getMin());
  EXPECT_EQ(Eigen::Vector2d(1.0, 4.0), bb.getMax());
  EXPECT_EQ(Eigen::Vector2d(0.75, 3.0), bb.getCenter());
}

TEST(BoundingBoxTest, enlargeDouble) {
  BoundingBox bb(std::vector<Eigen::Vector2d>({{1.0, 2.0}}));
  bb.enlarge(2.0);
  EXPECT_EQ(Eigen::Vector2d(-1.0, 0.0), bb.getMin());
  EXPECT_EQ(Eigen::Vector2d(3.0, 4.0), bb.getMax());
  EXPECT_EQ(Eigen::Vector2d(1.0, 2.0), bb.getCenter());
}

TEST(BoundingBoxTest, inputPointAreContained) {
  const std::vector<Eigen::Vector2d> inputs = {{1.0, 2.0}, {5.0, 10.0}, {3.0, 1.0}};
  const BoundingBox bb(inputs);
  for (const auto& p : inputs) {
    EXPECT_TRUE(bb.containsPoint(p));
  }
}

TEST(BoundingBoxTest, MinMaxCenterAreContained) {
  const std::vector<Eigen::Vector2d> inputs = {{1.0, 2.0}, {5.0, 10.0}, {3.0, 1.0}};
  const BoundingBox bb(inputs);
  EXPECT_TRUE(bb.containsPoint(bb.getMin()));
  EXPECT_TRUE(bb.containsPoint(bb.getMax()));
  EXPECT_TRUE(bb.containsPoint(bb.getCenter()));
}

TEST(BoundingBoxTest, copy) {
  const BoundingBox bb(std::vector<Eigen::Vector2d>({{1.0, 2.0},{3.0, 4.0}}));
  BoundingBox bb_copy(bb);
  EXPECT_EQ(bb.getMin(), bb_copy.getMin());
  EXPECT_EQ(bb.getMax(), bb_copy.getMax());
  EXPECT_EQ(bb.getCenter(), bb_copy.getCenter());
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
