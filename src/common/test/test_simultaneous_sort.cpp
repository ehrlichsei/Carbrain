#include "common/simultaneous_sort.h"
#include "common/macros.h"

THIRD_PARTY_HEADERS_BEGIN
#include <gtest/gtest.h>
#include <vector>
#include <deque>
#include <array>
#include <boost/range/size.hpp>
THIRD_PARTY_HEADERS_END

DISABLE_SUGGEST_OVERRIDE_WARNING

TEST(Containers, simultaneous_sort) {
  std::vector<double> v = {1, 2, 3, 4, 5, 6};
  std::vector<char> a = {'a', 'b', 'c', 'd', 'e', 'f'};
  std::deque<std::string> b = {"A", "B", "C", "D", "E", "F"};
  std::array<int, 6> c = {{4, -4, 42, 8, 0, -48}};

  std::vector<char> a_reversed(a.size());
  std::reverse_copy(a.begin(), a.end(), a_reversed.begin());
  std::vector<std::string> b_reversed(b.size());
  std::reverse_copy(b.begin(), b.end(), b_reversed.begin());
  std::vector<int> c_reversed(c.size());
  std::reverse_copy(c.begin(), c.end(), c_reversed.begin());

  common::simultaneous_sort(v, std::greater<double>(), a, b, c);

  EXPECT_EQ(v.size(), a.size());
  EXPECT_EQ(v.size(), b.size());
  EXPECT_EQ(v.size(), c.size());

  for (size_t i = 0; i < a.size(); ++i) {
    EXPECT_DOUBLE_EQ(6 - i, v[i]);
    EXPECT_EQ(a_reversed[i], a[i]);
    EXPECT_EQ(b_reversed[i], b[i]);
    EXPECT_EQ(c_reversed[i], c[i]);
  }
}

TEST(Containers, simultaneous_sort_array) {
  double v[] = {1, 2, 3, 4, 5, 6};
  char a[] = {'a', 'b', 'c', 'd', 'e', 'f'};
  std::deque<std::string> b = {"A", "B", "C", "D", "E", "F"};
  std::array<int, 6> c = {{4, -4, 42, 8, 0, -48}};

  char a_reversed[sizeof(a)];
  std::reverse_copy(std::begin(a), std::end(a), std::begin(a_reversed));
  std::vector<std::string> b_reversed(b.size());
  std::reverse_copy(b.begin(), b.end(), b_reversed.begin());
  std::vector<int> c_reversed(c.size());
  std::reverse_copy(c.begin(), c.end(), c_reversed.begin());

  common::simultaneous_sort(v, std::greater<double>(), a, b, c);

  EXPECT_EQ(boost::size(v), boost::size(a));
  EXPECT_EQ(boost::size(v), b.size());
  EXPECT_EQ(boost::size(v), c.size());

  for (size_t i = 0; i < boost::size(a); ++i) {
    EXPECT_DOUBLE_EQ(6 - i, v[i]);
    EXPECT_EQ(a_reversed[i], a[i]);
    EXPECT_EQ(b_reversed[i], b[i]);
    EXPECT_EQ(c_reversed[i], c[i]);
  }
}

TEST(Containers, simultaneous_sort_identity) {
  std::vector<long long> v = {1, 2, 3, 4, 5, 6};
  std::vector<char> a = {'a', 'b', 'c', 'd', 'e', 'f'};
  std::deque<std::string> b = {"A", "B", "C", "D", "E", "F"};
  std::array<int, 6> c = {{4, -4, 42, 8, 0, -48}};

  std::vector<long long> v_check(a.size());
  std::copy(v.begin(), v.end(), v_check.begin());
  std::vector<char> a_check(a.size());
  std::copy(a.begin(), a.end(), a_check.begin());
  std::vector<std::string> b_check(b.size());
  std::copy(b.begin(), b.end(), b_check.begin());
  std::vector<int> c_check(c.size());
  std::copy(c.begin(), c.end(), c_check.begin());

  common::simultaneous_sort(v, std::less<long long>(), a, b, c);

  EXPECT_EQ(v.size(), a.size());
  EXPECT_EQ(v.size(), b.size());
  EXPECT_EQ(v.size(), c.size());

  for (size_t i = 0; i < a.size(); ++i) {
    EXPECT_EQ(v_check[i], v[i]);
    EXPECT_EQ(a_check[i], a[i]);
    EXPECT_EQ(b_check[i], b[i]);
    EXPECT_EQ(c_check[i], c[i]);
  }
}

TEST(Containers, simultaneous_sort_empty) {
  std::vector<unsigned int> v;
  std::vector<char> a;
  std::deque<std::string> b;

  EXPECT_NO_THROW(common::simultaneous_sort(v, std::less<unsigned int>(), a, b));

  EXPECT_TRUE(v.empty() && a.empty() && b.empty());
}


int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
