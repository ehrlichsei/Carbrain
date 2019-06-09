#include "common/angle_conversions.h"
#include "common/macros.h"

THIRD_PARTY_HEADERS_BEGIN
#include <gtest/gtest.h>
THIRD_PARTY_HEADERS_END

DISABLE_SUGGEST_OVERRIDE_WARNING

using namespace Eigen;
using namespace common;

TEST(AngleConversions, radiansDegree) {
  EXPECT_EQ(toRad(toDegree(M_PI)), M_PI);

  // integers
  EXPECT_EQ(toRad(180), 3);
  EXPECT_EQ(toDegree(3), 180);
}

TEST(AngleConversion, toYaw3D) {
  EXPECT_DOUBLE_EQ(0.5, toYaw(Quaterniond(AngleAxisd(0.5, Vector3d::UnitZ()))));
  EXPECT_DOUBLE_EQ(0.0, toYaw(Quaterniond(AngleAxisd(0.0, Vector3d::UnitZ()))));
  EXPECT_DOUBLE_EQ(-0.5, toYaw(Quaterniond(AngleAxisd(-0.5, Vector3d::UnitZ()))));
  EXPECT_DOUBLE_EQ(2.0, toYaw(Quaterniond(AngleAxisd(2.0, Vector3d::UnitZ()))));
  EXPECT_DOUBLE_EQ(-2.0, toYaw(Quaterniond(AngleAxisd(-2.0, Vector3d::UnitZ()))));
  EXPECT_DOUBLE_EQ(1.0e-4, toYaw(Quaterniond(AngleAxisd(1.0e-4, Vector3d::UnitZ()))));
  EXPECT_DOUBLE_EQ(-1.0e-4, toYaw(Quaterniond(AngleAxisd(-1.0e-4, Vector3d::UnitZ()))));

  EXPECT_DOUBLE_EQ(0.5, toYaw(AngleAxisd(0.5, Vector3d::UnitZ()).toRotationMatrix()));
  EXPECT_DOUBLE_EQ(0.0, toYaw(AngleAxisd(0.0, Vector3d::UnitZ()).toRotationMatrix()));
  EXPECT_DOUBLE_EQ(-0.5, toYaw(AngleAxisd(-0.5, Vector3d::UnitZ()).toRotationMatrix()));
  EXPECT_DOUBLE_EQ(2.0, toYaw(AngleAxisd(2.0, Vector3d::UnitZ()).toRotationMatrix()));
  EXPECT_DOUBLE_EQ(-2.0, toYaw(AngleAxisd(-2.0, Vector3d::UnitZ()).toRotationMatrix()));
  EXPECT_DOUBLE_EQ(
      1.0e-4, toYaw(AngleAxisd(1.0e-4, Vector3d::UnitZ()).toRotationMatrix()));
  EXPECT_DOUBLE_EQ(
      -1.0e-4, toYaw(AngleAxisd(-1.0e-4, Vector3d::UnitZ()).toRotationMatrix()));
}

TEST(AngleConversion, toYaw2D) {
  EXPECT_DOUBLE_EQ(0.5, toYaw(Rotation2Dd(0.5).matrix()));
  EXPECT_DOUBLE_EQ(-0.5, toYaw(Rotation2Dd(-0.5).matrix()));
  EXPECT_DOUBLE_EQ(2.0, toYaw(Rotation2Dd(2.0).matrix()));
  EXPECT_DOUBLE_EQ(-2.0, toYaw(Rotation2Dd(-2.0).matrix()));
  EXPECT_DOUBLE_EQ(0.0, toYaw(static_cast<Matrix2d>(Matrix2d::Identity())));
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
