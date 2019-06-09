#include "common/macros.h"

#include <Eigen/Geometry>

DISABLE_SUGGEST_OVERRIDE_WARNING

template <class Range, class Transformation>
void transform(Range& range, const Transformation& trafo) {
  for (auto&& re : range) {
    re = trafo * re;
  }
}

THIRD_PARTY_HEADERS_BEGIN
#include <gtest/gtest.h>
THIRD_PARTY_HEADERS_END

#include <boost/range/algorithm/equal.hpp>

TEST(EigenTransform, TransfromVectro2dIdentity) {
  const std::vector<Eigen::Vector2d> vs = {{0, 1}, {1, 2}, {2, 3}};
  auto ps = vs;
  transform(ps, Eigen::Affine2d::Identity());
  EXPECT_TRUE(boost::equal(vs, ps));
}

TEST(EigenTransform, TransfromVectro2dTranslation) {
  std::vector<Eigen::Vector2d> vs = {{0, 1}, {1, 2}, {2, 3}};
  transform(vs, Eigen::Translation2d(1, 2));
  const std::vector<Eigen::Vector2d> ps = {{1, 3}, {2, 4}, {3, 5}};
  EXPECT_TRUE(boost::equal(vs, ps));
}

TEST(EigenTransform, TransfromVectro2dRotation) {
  std::vector<Eigen::Vector2d> vs = {{0, 1}, {1, 2}, {2, 3}};
  transform(vs, Eigen::Rotation2Dd(M_PI));
  const std::vector<Eigen::Vector2d> ps = {{0, -1}, {-2, -4}, {-3, -5}};
  EXPECT_TRUE(boost::equal(vs, ps));
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
