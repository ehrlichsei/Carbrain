#include "common/bresenham.h"
#include "common/macros.h"

THIRD_PARTY_HEADERS_BEGIN
#include <iostream>
#include <vector>
#include <algorithm>
#include <gtest/gtest.h>
#include <boost/static_assert.hpp>
THIRD_PARTY_HEADERS_END

DISABLE_SUGGEST_OVERRIDE_WARNING

using namespace common;

namespace {

class Result {
 public:
  void callback(const int x, const int y, const bool is_start, const bool is_end) {
    ints.push_back(x);
    ints.push_back(y);
  }

  std::vector<int> ints;
};

class BresenhamTest : public ::testing::Test {};

//! \todo except results in reverse order (eg. {1, 2, 0, 2} and {0, 2, 1, 2})

TEST_F(BresenhamTest, Octant1) {
  Result result;
  bresenham(3, 3, 5, 0, &Result::callback, &result);

  static const int arr[] = {5, 0, 4, 1, 4, 2, 3, 3};
  std::vector<int> expected(arr, arr + sizeof(arr) / sizeof(arr[0]));

  ASSERT_EQ(expected, result.ints);
}

TEST_F(BresenhamTest, Octant3) {
  Result result;
  bresenham(3, 3, 0, 2, &Result::callback, &result);

  static const int arr[] = {0, 2, 1, 2, 2, 3, 3, 3};
  std::vector<int> expected(arr, arr + sizeof(arr) / sizeof(arr[0]));

  ASSERT_EQ(expected, result.ints);
}

TEST_F(BresenhamTest, Octant4) {
  Result result;
  bresenham(3, 3, 0, 4, &Result::callback, &result);

  static const int arr[] = {0, 4, 1, 4, 2, 3, 3, 3};
  std::vector<int> expected(arr, arr + sizeof(arr) / sizeof(arr[0]));

  ASSERT_EQ(expected, result.ints);
}

TEST_F(BresenhamTest, Octant6) {
  Result result;
  bresenham(3, 3, 5, 6, &Result::callback, &result);

  static const int arr[] = {3, 3, 4, 4, 4, 5, 5, 6};
  std::vector<int> expected(arr, arr + sizeof(arr) / sizeof(arr[0]));

  ASSERT_EQ(expected, result.ints);
}

TEST_F(BresenhamTest, Diagonal) {
  Result result;
  bresenham(3, 3, 6, 6, &Result::callback, &result);

  static const int arr[] = {3, 3, 4, 4, 5, 5, 6, 6};
  std::vector<int> expected(arr, arr + sizeof(arr) / sizeof(arr[0]));

  ASSERT_EQ(expected, result.ints);
}

TEST_F(BresenhamTest, Horizontal) {
  Result result;
  bresenham(3, 3, 6, 3, &Result::callback, &result);

  static const int arr[] = {3, 3, 4, 3, 5, 3, 6, 3};
  std::vector<int> expected(arr, arr + sizeof(arr) / sizeof(arr[0]));

  ASSERT_EQ(expected, result.ints);
}

}  // namespace

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
