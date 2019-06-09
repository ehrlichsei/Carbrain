#include "common/eigen_adaptors.h"
#include "common/macros.h"

THIRD_PARTY_HEADERS_BEGIN
#include <gtest/gtest.h>
#include <common/basic_statistics_eigen.h>
THIRD_PARTY_HEADERS_END

DISABLE_SUGGEST_OVERRIDE_WARNING

TEST(Adaptors, eigen_casted_int_to_double) {
  const std::vector<Eigen::Vector2i> is = {{1, 0}, {1, 5}, {-1, 6}};
  const auto ds = is | common::eigen_casted<double>;
  EXPECT_EQ(Eigen::Vector2i(1 / 3, 11 / 3), common::mean(is));
  EXPECT_EQ(Eigen::Vector2d(1.0 / 3.0, 11.0 / 3.0), common::mean(ds));
}

TEST(Adaptors, eigen_transformed_translation) {
  const std::vector<Eigen::Vector2d> ps = {{1, 0}, {1, 5}, {-1, 6}};
  const auto translated = ps | common::eigen_transformed(Eigen::Translation2d(1, 3));
  auto p = ps.begin();
  auto t = translated.begin();
  for (; p != ps.end(); ++p, ++t) {
    EXPECT_EQ(*p + Eigen::Vector2d(1, 3), *t);
  }
}

TEST(Adaptors, eigen_transformed_rotation) {
  const std::vector<Eigen::Vector2d> ps = {{1, 0}, {1, 5}, {-1, 6}};
  const std::vector<Eigen::Vector2d> rs = {{0, 1}, {-5, 1}, {-6, -1}};
  const auto rotated = ps | common::eigen_transformed(Eigen::Rotation2Dd(M_PI / 2.0));
  auto rr = rs.begin();
  auto r = rotated.begin();
  for (; rr != rs.end(); ++rr, ++r) {
    EXPECT_NEAR(rr->x(), r->x(), 1e-12);
    EXPECT_NEAR(rr->y(), r->y(), 1e-12);
  }
}

TEST(Adaptors, eigen_transformed_affine) {
  const std::vector<Eigen::Vector2d> ps = {{1, 0}, {1, 5}, {-1, 6}};
  const std::vector<Eigen::Vector2d> rs = {{1, 4}, {-4, 4}, {-5, 2}};
  const auto rotated = ps | common::eigen_transformed(Eigen::Translation2d(1, 3) *
                                                      Eigen::Rotation2Dd(M_PI / 2.0));
  auto rr = rs.begin();
  auto r = rotated.begin();
  for (; rr != rs.end(); ++rr, ++r) {
    EXPECT_NEAR(rr->x(), r->x(), 1e-12);
    EXPECT_NEAR(rr->y(), r->y(), 1e-12);
  }
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
