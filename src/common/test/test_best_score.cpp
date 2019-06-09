#include "common/best_score.h"
#include "common/macros.h"

THIRD_PARTY_HEADERS_BEGIN
#include <gtest/gtest.h>
#include <vector>
#include <list>
THIRD_PARTY_HEADERS_END

DISABLE_SUGGEST_OVERRIDE_WARNING

TEST(Containers, min_score_empty) {
  const std::vector<double> v;
  const auto min = common::min_score(v, [](const auto x) { return x * x; });
  EXPECT_EQ(v.end(), min);
}

TEST(Containers, max_score_empty) {
  const std::vector<double> v;
  const auto max = common::max_score(v, [](const auto x) { return x * x; });
  EXPECT_EQ(v.end(), max);
}


TEST(Containers, minmax_score_empty) {
  const std::vector<double> v;
  const auto minmax = common::minmax_score(v, [](const auto x) { return x * x; });
  EXPECT_EQ(v.end(), minmax.first);
  EXPECT_EQ(v.end(), minmax.second);
}

TEST(Containers, min_score_vector) {
  const std::vector<double> v = {-2, 2, 1, 4};
  const double min = *common::min_score(v, [](const auto x) { return x * x; });
  EXPECT_DOUBLE_EQ(min, 1);
}

TEST(Containers, max_score_vector) {
  const std::vector<double> v = {-2, 2, 1, 2};
  const double& max = *common::max_score(v, [](const auto x) { return x * x; });
  EXPECT_DOUBLE_EQ(-2, max);
}

TEST(Containers, minmax_score_vector) {
  const std::vector<double> v = {-2, 2, 1, 2};
  const auto minmax = common::minmax_score(v, [](const auto x) { return x * x; });
  EXPECT_DOUBLE_EQ(-2, *minmax.second);
  EXPECT_DOUBLE_EQ(1, *minmax.first);
}

TEST(Containers, min_score_list) {
  const std::list<double> v = {-2, 2, 1, 4};
  const double min = *common::min_score(v, [](const auto x) { return x * x; });
  EXPECT_DOUBLE_EQ(min, 1);
}

TEST(Containers, max_score_list) {
  const std::list<double> v = {-2, 2, 1, 2};
  const double& max = *common::max_score(v, [](const auto x) { return x * x; });
  EXPECT_DOUBLE_EQ(-2, max);
}

TEST(Containers, minmax_score_list) {
  const std::list<double> v = {-2, 2, 1, 2};
  const auto minmax = common::minmax_score(v, [](const auto x) { return x * x; });
  EXPECT_DOUBLE_EQ(-2, *minmax.second);
  EXPECT_DOUBLE_EQ(1, *minmax.first);
}

TEST(Containers, min_score_array) {
  const double v[] = {-2, 2, 1, 4};
  const double min = *common::min_score(v, [](const auto x) { return x * x; });
  EXPECT_DOUBLE_EQ(min, 1);
}

TEST(Containers, max_score_array) {
  const double v[] = {-2, 2, 1, 2};
  const double& max = *common::max_score(v, [](const auto x) { return x * x; });
  EXPECT_DOUBLE_EQ(-2, max);
}

TEST(Containers, minmax_score_array) {
  const double v[] = {-2, 2, 1, 2};
  const auto minmax = common::minmax_score(v, [](const auto x) { return x * x; });
  EXPECT_DOUBLE_EQ(-2, *minmax.second);
  EXPECT_DOUBLE_EQ(1, *minmax.first);
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
