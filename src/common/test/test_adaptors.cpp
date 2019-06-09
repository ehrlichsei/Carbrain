#include "common/adaptors.h"
#include "common/macros.h"

THIRD_PARTY_HEADERS_BEGIN
#include <gtest/gtest.h>
#include <boost/range/algorithm_ext/iota.hpp>
#include <boost/range/adaptor/map.hpp>
#include <list>
#include <map>
#include <boost/container/slist.hpp>
#include <common/basic_statistics.h>
THIRD_PARTY_HEADERS_END

DISABLE_SUGGEST_OVERRIDE_WARNING

struct P {
  double x;
  double y;
};

inline bool operator==(const P& a, const P& b) {
  return a.x == b.x && a.y == b.y;
}

template <class Range>
void test_x_values_custom_type(const Range ps) {
  const auto xs = ps | common::x_values;
  double i = 0.0;
  for (const auto x : xs) {
    EXPECT_EQ(i, x);
    i++;
  }
}

TEST(Adaptors, x_values_custom_type_const_vector) {
  const std::vector<P> ps = {{0, 0}, {1, 5}, {2, 3}};
  test_x_values_custom_type(ps);
}

TEST(Adaptors, x_values_custom_type_const_list) {
  const std::list<P> ps = {{0, 0}, {1, 5}, {2, 3}};
  test_x_values_custom_type(ps);
}

TEST(Adaptors, x_values_custom_type_const_slist) {
  const boost::container::slist<P> ps = {{0, 0}, {1, 5}, {2, 3}};
  test_x_values_custom_type(ps);
}

TEST(Adaptors, x_values_custom_type_const_map_values) {
  const std::map<int, P> pps = {{0, {0, 0}}, {1, {1, 0}}, {2, {2, 4}}};
  const auto ps = pps | boost::adaptors::map_values;
  test_x_values_custom_type(ps);
}

template <class Range>
void test_x_values_custiom_type_mutable(Range ps) {
  auto xs = ps | common::x_values;
  boost::iota(xs, 0.0);
  double i = 0.0;
  for (const auto& p : ps) {
    EXPECT_EQ(i, p.x);
    EXPECT_EQ(0.0, p.y);
    i++;
  }
}

TEST(Adaptors, x_values_custom_type_mutable_vector) {
  const std::vector<P> ps = {{0, 0}, {0, 0}, {0, 0}};
  test_x_values_custiom_type_mutable(ps);
}

TEST(Adaptors, x_values_custom_type_mutable_list) {
  const std::list<P> ps = {{0, 0}, {0, 0}, {0, 0}};
  test_x_values_custiom_type_mutable(ps);
}

TEST(Adaptors, x_values_custom_type_mutable_slist) {
  const boost::container::slist<P> ps = {{0, 0}, {0, 0}, {0, 0}};
  test_x_values_custiom_type_mutable(ps);
}

TEST(Adaptors, x_values_custom_type_mutable_map_values) {
  std::map<int, P> pps = {{0, {0, 0}}, {1, {1, 0}}, {2, {2, 0}}};
  auto ps = pps | boost::adaptors::map_values;
  test_x_values_custiom_type_mutable(ps);
}

template <class Range>
void test_y_values_custom_type(const Range ps) {
  const auto ys = ps | common::y_values;
  double i = 0.0;
  for (const auto y : ys) {
    EXPECT_EQ(i, y);
    i++;
  }
}

TEST(Adaptors, y_values_custom_type_const_vector) {
  const std::vector<P> ps = {{0, 0}, {5, 1}, {3, 2}};
  test_y_values_custom_type(ps);
}

TEST(Adaptors, y_values_custom_type_const_list) {
  const std::list<P> ps = {{0, 0}, {5, 1}, {3, 2}};
  test_y_values_custom_type(ps);
}

TEST(Adaptors, y_values_custom_type_const_slist) {
  const boost::container::slist<P> ps = {{0, 0}, {5, 1}, {3, 2}};
  test_y_values_custom_type(ps);
}

TEST(Adaptors, y_values_custom_type_const_map_values) {
  const std::map<int, P> pps = {{0, {0, 0}}, {1, {0, 1}}, {2, {4, 2}}};
  const auto ps = pps | boost::adaptors::map_values;
  test_y_values_custom_type(ps);
}

template <class Range>
void test_y_values_custiom_type_mutable(Range ps) {
  auto ys = ps | common::y_values;
  boost::iota(ys, 0.0);
  double i = 0.0;
  for (const auto& p : ps) {
    EXPECT_EQ(i, p.y);
    EXPECT_EQ(0.0, p.x);
    i++;
  }
}

TEST(Adaptors, y_values_custom_type_mutable_vector) {
  const std::vector<P> ps = {{0, 0}, {0, 0}, {0, 0}};
  test_y_values_custiom_type_mutable(ps);
}

TEST(Adaptors, y_values_custom_type_mutable_list) {
  const std::list<P> ps = {{0, 0}, {0, 0}, {0, 0}};
  test_y_values_custiom_type_mutable(ps);
}

TEST(Adaptors, y_values_custom_type_mutable_slist) {
  const boost::container::slist<P> ps = {{0, 0}, {0, 0}, {0, 0}};
  test_y_values_custiom_type_mutable(ps);
}

TEST(Adaptors, y_values_custom_type_mutable_map_values) {
  std::map<int, P> pps = {{0, {0, 0}}, {1, {0, 1}}, {2, {0, 1}}};
  auto ps = pps | boost::adaptors::map_values;
  test_y_values_custiom_type_mutable(ps);
}

TEST(Adaptors, members_size) {
  const std::vector<std::vector<double>> vvs = {{}, {0}, {0, 0}};
  const auto sizes = vvs | common::members(&std::vector<double>::size);
  std::size_t i = 0;
  for (const std::size_t s : sizes) {
    EXPECT_EQ(i, s);
    i++;
  }
}

TEST(Adaptors, static_casted_int_to_double) {
  const std::vector<int> is = {0, 1, 5, -1};
  const auto ds = is | common::static_casted<double>;
  EXPECT_EQ(5.0/4.0, common::mean(ds));
  EXPECT_NE(5.0/4.0, common::mean(is));
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
