#include "common/type_to_string.h"
#include "common/macros.h"

THIRD_PARTY_HEADERS_BEGIN
#include <gtest/gtest.h>
THIRD_PARTY_HEADERS_END

DISABLE_SUGGEST_OVERRIDE_WARNING

using common::toString;

TEST(TypeToString, basic_types) {
  EXPECT_EQ(toString<bool>(), "bool");
  EXPECT_EQ(toString<int>(), "int");
  EXPECT_EQ(toString<double>(), "double");
  EXPECT_EQ(toString<std::string>(), "std::string");
}

TEST(TypeToString, vector_types) {
  EXPECT_EQ(toString<std::vector<bool>>(), "std::vector<bool>");
  EXPECT_EQ(toString<std::vector<int>>(), "std::vector<int>");
  EXPECT_EQ(toString<std::vector<double>>(), "std::vector<double>");
  EXPECT_EQ(toString<std::vector<std::string>>(), "std::vector<std::string>");
}

// typedefs to avoid commatas in macro-calls
using bool_map = std::map<std::string,bool>;
using int_map = std::map<std::string,int>;
using double_map = std::map<std::string,double>;
using string_map = std::map<std::string,std::string>;

TEST(TypeToString, map_types) {
  EXPECT_EQ(toString<bool_map>(), "std::map<std::string, bool>");
  EXPECT_EQ(toString<int_map>(), "std::map<std::string, int>");
  EXPECT_EQ(toString<double_map>(), "std::map<std::string, double>");
  EXPECT_EQ(toString<string_map>(), "std::map<std::string, std::string>");
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
