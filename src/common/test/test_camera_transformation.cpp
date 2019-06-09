#include "common/camera_transformation.h"
#include "common/camera_transformation_parameters.h"
#include "common/macros.h"

THIRD_PARTY_HEADERS_BEGIN
#include <gtest/gtest.h>
THIRD_PARTY_HEADERS_END

#include "common/test/dummy_parameter_handler.h"

DISABLE_SUGGEST_OVERRIDE_WARNING

using namespace Eigen;
using namespace common;

TEST(CameraTransformationTest, trivial) {
  DummyParameterHandler parameter_handler;

  parameter_handler.addParam(CAMERA_FOCAL_LENGTH_X, 1.0);
  parameter_handler.addParam(CAMERA_FOCAL_LENGTH_Y, 1.0);
  parameter_handler.addParam(CAMERA_OPTICAL_CENTER_X, 1.0);
  parameter_handler.addParam(CAMERA_OPTICAL_CENTER_Y, 2.0);

  // camera looks from 1m above the ground staright down
  parameter_handler.addParam(CAMERA_R11, 1.0);
  parameter_handler.addParam(CAMERA_R12, 0.0);
  parameter_handler.addParam(CAMERA_R13, 0.0);
  parameter_handler.addParam(CAMERA_R21, 0.0);
  parameter_handler.addParam(CAMERA_R22, 1.0);
  parameter_handler.addParam(CAMERA_R23, 0.0);
  parameter_handler.addParam(CAMERA_R31, 0.0);
  parameter_handler.addParam(CAMERA_R32, 0.0);
  parameter_handler.addParam(CAMERA_R33, 1.0);

  parameter_handler.addParam(CAMERA_T1, 0.0);
  parameter_handler.addParam(CAMERA_T2, 0.0);
  parameter_handler.addParam(CAMERA_T3, 1000.0); //minde mm!

  const std::unique_ptr<CameraTransformation> trafo =
      std::make_unique<CameraTransformation>(&parameter_handler);

  EXPECT_EQ(1, trafo->transformGroundToImageExact(Vector3d::Zero()).x());
  EXPECT_EQ(2, trafo->transformGroundToImageExact(Vector3d::Zero()).y());
  EXPECT_EQ(1.01, trafo->transformGroundToImageExact(Vector3d(0.01, 0.01, 0)).x());
  EXPECT_EQ(2.01, trafo->transformGroundToImageExact(Vector3d(0.01, 0.01, 0)).y());
  EXPECT_EQ(1, trafo->transformGroundToImage(Vector3d(0.01, 0.01, 0)).x());
  EXPECT_EQ(2, trafo->transformGroundToImage(Vector3d(0.01, 0.01, 0)).y());

  EXPECT_DOUBLE_EQ(-1, trafo->transformImageToGround(Vector2i::Zero()).x());
  EXPECT_DOUBLE_EQ(-2, trafo->transformImageToGround(Vector2i::Zero()).y());
  EXPECT_DOUBLE_EQ(0 , trafo->transformImageToGround(Vector2i::Zero()).z());

  const Vector2iVector ps = {{0,1}, {1,0}};
  Vector3dVector vps;
  trafo->transformImageToGround(ps, &vps);
  EXPECT_EQ(vps.size(), ps.size());
  Vector2iVector pps;
  trafo->transformGroundToImage(vps, &pps);
  EXPECT_EQ(pps.size(), ps.size());
  EXPECT_TRUE(std::equal(ps.begin(), ps.end(), pps.begin()));

  const auto P = trafo->getProjectionMatrix();
  EXPECT_EQ(1, P(0,0));
  EXPECT_EQ(0, P(0,1));
  EXPECT_EQ(1, P(0,2));
  EXPECT_EQ(1, P(0,3));

  EXPECT_EQ(0, P(1,0));
  EXPECT_EQ(1, P(1,1));
  EXPECT_EQ(2, P(1,2));
  EXPECT_EQ(2, P(1,3));

  EXPECT_EQ(0, P(2,0));
  EXPECT_EQ(0, P(2,1));
  EXPECT_EQ(1, P(2,2));
  EXPECT_EQ(1, P(2,3));
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
