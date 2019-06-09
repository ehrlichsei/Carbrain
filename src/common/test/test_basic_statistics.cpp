#include <common/macros.h>

THIRD_PARTY_HEADERS_BEGIN
#include <gtest/gtest.h>
#include <list>
THIRD_PARTY_HEADERS_END
#include "common/math.h"
#include "common/types.h"
#include "common/basic_statistics.h"

DISABLE_SUGGEST_OVERRIDE_WARNING

using common::squared;

// sum
TEST(BasicStatistics, sumVectorDouble) {
  const std::vector<double> v = {1, 2, 3, 4};
  const double sum = common::sum(v);
  EXPECT_DOUBLE_EQ(10.0, sum);
}

TEST(BasicStatistics, sumListFloat) {
  const std::list<float> v = {1, 2, 3, 4};
  const double sum = common::sum<double>(v);
  EXPECT_DOUBLE_EQ(10.0, sum);
}

TEST(BasicStatistics, sumSetInt) {
  const std::set<int> v = {1, 2, 3, 4};
  const double sum = common::sum<double>(v);
  EXPECT_DOUBLE_EQ(10.0, sum);
}

TEST(BasicStatistics, emptySumVector) {
  const std::vector<double> v = {};
  const double sum = common::sum(v);
  EXPECT_DOUBLE_EQ(0.0, sum);
}

TEST(BasicStatistics, emptySumList) {
  const std::list<double> v = {};
  const double sum = common::sum(v);
  EXPECT_DOUBLE_EQ(0.0, sum);
}

// mean
TEST(BasicStatistics, meanVectorDouble) {
  const std::vector<double> v = {1, 2, 3, 4};
  const double mean = common::mean(v);
  EXPECT_DOUBLE_EQ(10.0 / 4.0, mean);
}

TEST(BasicStatistics, meanListFloat) {
  const std::list<double> v = {1, 2, 3, 4};
  const double mean = common::mean(v);
  EXPECT_DOUBLE_EQ(10.0 / 4.0, mean);
}

TEST(BasicStatistics, meanSetInt) {
  const std::set<int> v = {1, 2, 3, 4};
  const double mean = common::mean<double>(v);
  EXPECT_DOUBLE_EQ(10.0 / 4.0, mean);
}

TEST(BasicStatistics, assertMeanEmptyVector) {
  const std::vector<int> vs = {};
  DISABLE_USED_BUT_MARKED_UNUSED_WARNING
  EXPECT_DEATH(common::mean(vs), "mean is not defined for less than 1 element");
  ENABLE_USED_BUT_MARKED_UNUSED_WARNING
}

TEST(BasicStatistics, assertMeanEmptyList) {
  const std::list<int> vs = {};
  DISABLE_USED_BUT_MARKED_UNUSED_WARNING
  EXPECT_DEATH(common::mean(vs), "mean is not defined for less than 1 element");
  ENABLE_USED_BUT_MARKED_UNUSED_WARNING
}

// squared sum
TEST(BasicStatistics, squaredSumVectorDouble) {
  const std::vector<double> v = {1, 2, 3, 4};
  const double ssum = common::squaredSum(v);
  EXPECT_DOUBLE_EQ(1 * 1 + 2 * 2 + 3 * 3 + 4 * 4, ssum);
}

TEST(BasicStatistics, squaredSumListFloat) {
  const std::list<double> v = {1, 2, 3, 4};
  const float ssum = common::squaredSum<float>(v);
  EXPECT_DOUBLE_EQ(1 * 1 + 2 * 2 + 3 * 3 + 4 * 4, ssum);
}

TEST(BasicStatistics, squaredSumSetInt) {
  const std::set<int> v = {1, 2, 3, 4};
  const float ssum = common::squaredSum<float>(v);
  EXPECT_DOUBLE_EQ(1 * 1 + 2 * 2 + 3 * 3 + 4 * 4, ssum);
}

// variance
TEST(BasicStatistics, variance) {
  const std::vector<double> v = {1, 2, 3, 4};
  const double mean = common::mean(v);
  const double variance = common::variance(v, mean);
  EXPECT_DOUBLE_EQ((squared(mean - 1.0) + squared(mean - 2.0) +
                    squared(mean - 3.0) + squared(mean - 4.0)) /
                       3.0,
                   variance);
}

TEST(BasicStatistics, assertVarianceSingleVector) {
  const std::vector<int> vs = {42};
  DISABLE_USED_BUT_MARKED_UNUSED_WARNING
  EXPECT_DEATH(common::variance(vs, 0),
               "variance is not defined for less than 2 elements");
  ENABLE_USED_BUT_MARKED_UNUSED_WARNING
}

TEST(BasicStatistics, assertVarianceSingleList) {
  const std::list<int> vs = {42};
  DISABLE_USED_BUT_MARKED_UNUSED_WARNING
  EXPECT_DEATH(common::variance(vs, 0),
               "variance is not defined for less than 2 elements");
  ENABLE_USED_BUT_MARKED_UNUSED_WARNING
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
