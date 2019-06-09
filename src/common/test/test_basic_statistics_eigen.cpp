#include <common/macros.h>

THIRD_PARTY_HEADERS_BEGIN
#include <gtest/gtest.h>
THIRD_PARTY_HEADERS_END
#include "common/types.h"
#include "common/basic_statistics_eigen.h"

DISABLE_SUGGEST_OVERRIDE_WARNING

TEST(BasicStatisticsEigen, emptySumVector2i) {
  const common::Vector2iVector v;
  const auto sum = common::sum(v);
  EXPECT_EQ(Eigen::Vector2i(0, 0), sum);
}

TEST(BasicStatisticsEigen, meanVector2i) {
  const common::Vector2iVector v = {{1, 1}, {3, 3}};
  const auto mean = common::mean(v);
  EXPECT_EQ(Eigen::Vector2i(2, 2), mean);
}

TEST(BasicStatisticsEigen, meanVector3d) {
  const common::Vector3dVector v = {{1, 1, 1}, {3, 3, 3}};
  const auto mean = common::mean(v);
  EXPECT_EQ(Eigen::Vector3d(2, 2, 2), mean);
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
