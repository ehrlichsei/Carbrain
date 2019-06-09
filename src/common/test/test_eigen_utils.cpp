#include "common/eigen_utils.h"
#include "common/macros.h"

THIRD_PARTY_HEADERS_BEGIN
#include <gtest/gtest.h>
THIRD_PARTY_HEADERS_END

DISABLE_SUGGEST_OVERRIDE_WARNING

TEST(EigenUtils, to2D_t3D_consistency) {
  const Eigen::Vector2d s(6.0, 4.2);
  const auto t = to2D(to3D(s));
  EXPECT_EQ(s.x(), t.x());
  EXPECT_EQ(s.y(), t.y());
}

TEST(EigenUtils, to2D_t3D_consistency_float) {
  const Eigen::Vector2f s(6.f, 4.2f);
  const auto t = to2D(to3D(s));
  static_assert(std::is_same<decltype(t)::Scalar, float>::value,"");
  EXPECT_EQ(s.x(), t.x());
  EXPECT_EQ(s.y(), t.y());
}

TEST(EigenUtils, to3D_t2D_consistency) {
  const Eigen::Vector3d s(6.0, 4.2, 100.0);
  const auto t = to3D(to2D(s));
  EXPECT_EQ(s.x(), t.x());
  EXPECT_EQ(s.y(), t.y());
  EXPECT_EQ(0.0, t.z());
}

TEST(EigenUtils, to3D_t2D_consistency_float) {
  const Eigen::Vector3f s(6.f, 4.2f, 100.f);
  const auto t = to3D(to2D(s));
  EXPECT_EQ(s.x(), t.x());
  EXPECT_EQ(s.y(), t.y());
  EXPECT_EQ(0.f, t.z());
}

TEST(EigenUtils, toEigenVector_to2D_consistency_Vector3d) {
  const Eigen::Vector3d s (0.0, 5.0, -1);
  const auto to_2d = common::to2D(s);
  const auto to_v = common::to<Eigen::Vector2d>(s);
  static_assert(std::is_same<decltype(to_2d), decltype(to_v)>::value, "");
  EXPECT_EQ(to_2d.x(), to_v.x());
  EXPECT_EQ(to_2d.y(), to_v.y());
}

TEST(EigenUtils, toEigenVector_to2D_consistency_Vector2d) {
  const Eigen::Vector2d s (1.0, 5.0);
  const auto to_2d = common::to2D(s);
  const auto to_v = common::to<Eigen::Vector2d>(s);
  static_assert(std::is_same<decltype(to_2d), decltype(to_v)>::value, "");
  EXPECT_EQ(to_2d.x(), to_v.x());
  EXPECT_EQ(to_2d.y(), to_v.y());
}


TEST(EigenUtils, toEigenVector_to3D_consistency_Vector3d) {
  const Eigen::Vector3d s (0.0, 5.0, -1);
  const auto to_3d = common::to3D(s);
  const auto to_v = common::to<Eigen::Vector3d>(s);
  static_assert(std::is_same<decltype(to_3d), decltype(to_v)>::value, "");
  EXPECT_EQ(to_3d.x(), to_v.x());
  EXPECT_EQ(to_3d.y(), to_v.y());
  EXPECT_EQ(to_3d.z(), to_v.z());
}

TEST(EigenUtils, toEigenVector_to3D_consistency_Vector2d) {
  const Eigen::Vector2d s (1.0, 5.0);
  const auto to_3d = common::to3D(s);
  const auto to_v = common::to<Eigen::Vector3d>(s);
  static_assert(std::is_same<decltype(to_3d), decltype(to_v)>::value, "");
  EXPECT_EQ(to_3d.x(), to_v.x());
  EXPECT_EQ(to_3d.y(), to_v.y());
  EXPECT_EQ(to_3d.z(), to_v.z());
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
