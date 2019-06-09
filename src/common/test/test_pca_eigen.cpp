#include "common/pca_eigen.h"
#include "common/macros.h"

THIRD_PARTY_HEADERS_BEGIN
#include <gtest/gtest.h>
THIRD_PARTY_HEADERS_END
#include "common/eigen_utils.h"

DISABLE_SUGGEST_OVERRIDE_WARNING

using namespace Eigen;

TEST(PCA_Eigen, getAngle0) {
  EXPECT_DOUBLE_EQ(0, common::getAngle(Vector2d::UnitX(), Vector2d::UnitX()));
  EXPECT_DOUBLE_EQ(0, common::getAngle(Vector2d::UnitY(), Vector2d::UnitY()));
  EXPECT_DOUBLE_EQ(0, common::getAngle(Vector2d(1, 2), Vector2d(1, 2)));
}

TEST(PCA_Eigen, getAngle90) {
  EXPECT_DOUBLE_EQ(M_PI / 2, common::getAngle(Vector2d::UnitY(), Vector2d::UnitX()));
  EXPECT_DOUBLE_EQ(-M_PI / 2, common::getAngle(Vector2d::UnitX(), Vector2d::UnitY()));
  EXPECT_DOUBLE_EQ(M_PI / 2, common::getAngle(Vector2d(1, 2), Vector2d(2, -1)));
}

TEST(PCA_Eigen, getAngleWarping) {
  const double angle = M_PI + 0.001;
  const double warped_angle = M_PI - 0.001;
  EXPECT_DOUBLE_EQ(warped_angle,
                   common::getAngle(Vector2d::UnitX(),
                                    Eigen::Rotation2Dd(angle) * Vector2d::UnitX()));
}

static const MatrixXd m_45 =
    common::toMatrix2D(common::Vector2dVector({{0, 0}, {1, 1}, {-1, -1}}));
static const double eps = 0.00001;

TEST(PCA_Eigen, angle_45) {
  const double angle = common::getAngleToPrincipalComponent(Vector2d::UnitX(), m_45);
  EXPECT_EQ(M_PI / 4, angle);
  const double angle_2 = common::getAngleToPrincipalComponent(
      Vector2d::UnitX(), common::Vector2dVector({{0, 0}, {1, 1}, {-1, -1}}));
  EXPECT_EQ(M_PI / 4, angle_2);
}

TEST(PCA_Eigen, principal_component_analysis_m_45) {
  Eigen::MatrixXd eigenvectors;
  Eigen::VectorXd eigenvalues;
  common::principle_component_analysis(m_45, &eigenvectors, &eigenvalues);

  EXPECT_NEAR(0.0, eigenvalues(0), eps);
  EXPECT_NEAR(2.0, eigenvalues(1), eps);
}

TEST(PCA_Eigen, assertOnEmpty) {
  const common::Vector2dVector vs;
  DISABLE_USED_BUT_MARKED_UNUSED_WARNING
  EXPECT_DEATH(common::getPrincipalComponent(vs),
               "Not enough data points for pca!");
  ENABLE_USED_BUT_MARKED_UNUSED_WARNING
}

TEST(PCA_Eigen, assertOnTooSmall) {
  const common::Vector2dVector vs = {{0, 0}};
  DISABLE_USED_BUT_MARKED_UNUSED_WARNING
  EXPECT_DEATH(common::getPrincipalComponent(vs),
               "Not enough data points for pca!");
  ENABLE_USED_BUT_MARKED_UNUSED_WARNING
}

TEST(PCA_Eigen, noAssertOnSquared) {
  const common::Vector2dVector vs = {{0, 0}, {1, 1}};
  common::getPrincipalComponent(vs);
  SUCCEED();
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
