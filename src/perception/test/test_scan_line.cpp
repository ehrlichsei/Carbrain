#include <common/macros.h>

THIRD_PARTY_HEADERS_BEGIN
#include <gtest/gtest.h>
#include <eigen3/Eigen/Dense>
THIRD_PARTY_HEADERS_END

#include "../include/scan_line.h"

DISABLE_SUGGEST_OVERRIDE_WARNING

using namespace common;

inline bool equals(const ScanLine& a, const ScanLine& b) {
  return (a.start == b.start) && (a.end == b.end);
}

TEST(ScanLineTests, clipNothingTest) {
  ScanLine reference_line({0, 0}, {0, 10});

  ScanLine clipped_line(reference_line);
  EXPECT_TRUE(ScanLine::clip(cv::Size{100, 100}, clipped_line));
  // line should still be the same
  EXPECT_TRUE(equals(reference_line, clipped_line));
}


TEST(ScanLineTests, clipTest) {
  ScanLine reference_line({0, -10}, {0, 10});

  ScanLine clipped_line(reference_line);
  EXPECT_TRUE(ScanLine::clip(cv::Size{100, 100}, clipped_line));
  // line should be clipped
  EXPECT_EQ(clipped_line.start, ImagePoint(0, 0));
}

TEST(ScanLineTests, trimTest) {
  ScanLine reference_line({0, 0}, {0, 10});

  ScanLine trimmed_line = reference_line.trim(0.1);
  EXPECT_TRUE(equals(trimmed_line, ScanLine({0, 1}, {0, 9})));
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
