#include "common/contains.h"
#include "common/macros.h"

THIRD_PARTY_HEADERS_BEGIN
#include <gtest/gtest.h>
#include <set>
#include <vector>
#include <map>
THIRD_PARTY_HEADERS_END

DISABLE_SUGGEST_OVERRIDE_WARNING

TEST(contains, has_find) {
  using namespace common::algorithm_detail;

  const bool vector_has_find = has_find<std::vector<int>, const int&>::type();
  EXPECT_FALSE(vector_has_find);

  const bool set_has_find = has_find<std::set<int>, int>::type();
  EXPECT_TRUE(set_has_find);
}

TEST(contains, set) {
  const std::set<double> v = {1, 2, 3, 4};
  EXPECT_TRUE(common::contains(v, 2));
  EXPECT_FALSE(common::contains(v, 5));
}

TEST(contains, vector) {
  const std::vector<double> v = {1, 2, 3, 4};
  EXPECT_TRUE(common::contains(v, 2));
  EXPECT_FALSE(common::contains(v, 5));
}

TEST(contains, map) {
  const std::map<int, int> v = {{1,1}, {2,2}, {3,3}, {4,4}};
  EXPECT_TRUE(common::contains(v, 2));
  EXPECT_FALSE(common::contains(v, 5));
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
