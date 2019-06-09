#include "common/indirect_sort.h"
#include "common/macros.h"

THIRD_PARTY_HEADERS_BEGIN
#include <gtest/gtest.h>
THIRD_PARTY_HEADERS_END

DISABLE_SUGGEST_OVERRIDE_WARNING

TEST(Containers, indirect_sort_identity) {
  std::vector<double> v = {1, 2, 3, 4, 5, 6};
  std::vector<double> score;

  auto identity = [](const auto& x) -> double { return x; };
  std::vector<size_t> indices = common::indirect_sort(v, identity, score);

  EXPECT_EQ(v.size(), indices.size());
  EXPECT_EQ(v.size(), score.size());

  // indices should be 0,1,2,...
  for (size_t i = 0; i < indices.size(); ++i) {
    EXPECT_EQ(i, indices[i]);
    EXPECT_EQ(v[i], score[i]);
  }
}

TEST(Containers, indirect_sort_reversed) {
  std::vector<double> v = {1, 2, 3, 4, 5, 6};
  std::vector<double> score;

  auto inverted = [](const auto& x) -> double { return -x; };
  std::vector<size_t> indices = common::indirect_sort(v, inverted, score);

  EXPECT_EQ(v.size(), indices.size());
  EXPECT_EQ(v.size(), score.size());

  for (size_t i = 0; i < indices.size(); ++i) {
    EXPECT_EQ(v.size() - i - 1, indices[i]);
    EXPECT_EQ(v[i], -score[i]);
  }
}

TEST(Containers, indirect_sort_shuffled) {
  std::vector<double> v = {1, 3, 2, 4, 6, 5};
  std::vector<size_t> order = {0, 2, 1, 3, 5, 4};
  std::vector<double> score;

  auto multiply = [](const auto& x) -> double { return x * x; };
  std::vector<size_t> indices = common::indirect_sort(v, multiply, score);

  EXPECT_EQ(v.size(), indices.size());
  EXPECT_EQ(v.size(), score.size());

  for (size_t i = 0; i < indices.size(); ++i) {
    EXPECT_EQ(order[i], indices[i]);
    EXPECT_EQ(v[i] * v[i], score[i]);
  }
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
