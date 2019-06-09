#include "common/minmax_element.h"
#include "common/macros.h"

THIRD_PARTY_HEADERS_BEGIN
#include <gtest/gtest.h>
#include <boost/range/algorithm/equal.hpp>
THIRD_PARTY_HEADERS_END

DISABLE_SUGGEST_OVERRIDE_WARNING

TEST(MinMaxElement, defaultPredicateConst) {
  const std::vector<double> v = {-2, 2, 1, 4};
  const auto minmax = common::minmax_element(v);
  EXPECT_DOUBLE_EQ(*minmax.first, -2);
  EXPECT_DOUBLE_EQ(*minmax.second, 4);
}

TEST(MinMaxElement, defaultPredicateMutable) {
  std::vector<double> v = {-2, 2, 1, 4};
  auto minmax = common::minmax_element(v);
  *minmax.first = -42.0;
  *minmax.second = 42.0;
  const std::vector<double> ref = {-42.0, 2, 1, 42.0};
  EXPECT_TRUE(boost::equal(ref, v));
}

TEST(Containers, customPredicateConst) {
  const std::vector<double> v = {-3, 2, 1, 2};
  const auto minmax = common::minmax_element(
      v, [](double a, double b) { return std::abs(a) < std::abs(b); });
  EXPECT_DOUBLE_EQ(*minmax.first, 1);
  EXPECT_DOUBLE_EQ(*minmax.second, -3);
}

TEST(MinMaxElement, customPredicateMutable) {
  std::vector<double> v = {-3, 2, 1, 2};
  auto minmax = common::minmax_element(
      v, [](double a, double b) { return std::abs(a) < std::abs(b); });
  *minmax.first = 42.0;
  *minmax.second = -42.0;
  const std::vector<double> ref = {-42.0, 2, 42.0, 2};
  EXPECT_TRUE(boost::equal(ref, v));
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
