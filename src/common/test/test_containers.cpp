#include "common/container.h"
#include "common/macros.h"

THIRD_PARTY_HEADERS_BEGIN
#include <gtest/gtest.h>
#include <list>
#include <boost/range/algorithm/equal.hpp>
#include <boost/algorithm/string.hpp>
THIRD_PARTY_HEADERS_END

DISABLE_SUGGEST_OVERRIDE_WARNING

TEST(Containers, toSetVector) {
  const std::list<double> l = {1, 2, 3, 4};
  const std::set<double> s = common::toSet(l);
  EXPECT_TRUE(boost::equal(l, s));
}

TEST(Containers, EmptyRangeToStringIsEmpty) {
  const std::vector<double> v = {};
  const std::string s = common::toString(v);
  EXPECT_TRUE(s.empty());
}

TEST(Containers, ToStringOneElement) {
  const std::vector<int> v = {42};
  const std::string s = common::toString(v);
  EXPECT_EQ(s, std::to_string(42));
}

TEST(Containers, ToStringVectorOfInts) {
  const std::vector<double> v = {1, 1, 2, 0, 5};
  const std::string string = common::toString(v);
  EXPECT_TRUE(!string.empty());
  EXPECT_TRUE(string.back() != ' ');
  EXPECT_TRUE(string.front() != ' ');
  std::vector<std::string> strings;
  boost::split(strings, string, boost::algorithm::is_any_of(" "));
  EXPECT_TRUE(strings.size() == v.size());
  EXPECT_TRUE(boost::equal(
      v,
      strings,
      [](const auto& d, const auto& s) { return d == std::atoi(s.c_str()); }));
}

TEST(Containers, ToStringSetOfInts) {
  const std::set<double> v = {1, 1, 2, 0, 5};
  const std::string string = common::toString(v);
  EXPECT_TRUE(!string.empty());
  EXPECT_TRUE(string.back() != ' ');
  EXPECT_TRUE(string.front() != ' ');
  std::vector<std::string> strings;
  boost::split(strings, string, boost::algorithm::is_any_of(" "));
  EXPECT_TRUE(strings.size() == v.size());
  EXPECT_TRUE(boost::equal(
      v,
      strings,
      [](const auto& d, const auto& s) { return d == std::atoi(s.c_str()); }));
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
