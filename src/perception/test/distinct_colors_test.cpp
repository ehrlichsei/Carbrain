#include <common/macros.h>

THIRD_PARTY_HEADERS_BEGIN
#include <gtest/gtest.h>
#include <boost/range/algorithm/equal.hpp>
THIRD_PARTY_HEADERS_END

#include "distinct_colors.h"

DISABLE_SUGGEST_OVERRIDE_WARNING

using namespace perception;

TEST(DistinctColors, randomBrightColorIsValidAndBright) {
  for (int i = 0; i < 1000; i++) {
    const auto color = randomBrightColor();
    // blue
    EXPECT_GE(color[0], 0);
    EXPECT_LE(color[0], 255);
    // green
    EXPECT_GE(color[1], 0);
    EXPECT_LE(color[1], 255);
    // red
    EXPECT_GE(color[2], 0);
    EXPECT_LE(color[2], 255);
    // alpha
    EXPECT_EQ(color[3], 255);
    const double max = std::max(color[0], std::max(color[1], color[2]));
    const double min = std::min(color[0], std::min(color[1], color[2]));
    // Value
    EXPECT_GE(max, 200);
    // Saturation
    EXPECT_GE((max - min) / max, (200.0 - 1.0) / 255.0);
  }
}

TEST(DistincColors, randomBrightColorsWantZeroGetEmpty) {
  EXPECT_TRUE(randomBrightColors(0).empty());
}

TEST(DistincColors, randomBrightColorsWantNGetN) {
  EXPECT_EQ(1, randomBrightColors(1).size());
  EXPECT_EQ(10, randomBrightColors(10).size());
  EXPECT_EQ(42, randomBrightColors(42).size());
  EXPECT_EQ(1e3, randomBrightColors(1e3).size());
}

TEST(DistinctColors, randomBrightColorsAreValidAndBright) {
  for (const auto& color : randomBrightColors(1000)) {
    // blue
    EXPECT_GE(color[0], 0);
    EXPECT_LE(color[0], 255);
    // green
    EXPECT_GE(color[1], 0);
    EXPECT_LE(color[1], 255);
    // red
    EXPECT_GE(color[2], 0);
    EXPECT_LE(color[2], 255);
    // alpha
    EXPECT_EQ(color[3], 255);
    const double max = std::max(color[0], std::max(color[1], color[2]));
    const double min = std::min(color[0], std::min(color[1], color[2]));
    // Value
    EXPECT_GE(max, 200);
    // Saturation
    EXPECT_GE((max - min) / max, (200.0 - 1.0) / 255.0);
  }
}

TEST(DistincColors, nDistincColorsWantZeroGetEmpty) {
  EXPECT_TRUE(nDistinctColors(0).empty());
  EXPECT_TRUE(nDistinctColors(0, 255).empty());
  EXPECT_TRUE(nDistinctColors(0, 0).empty());
  EXPECT_TRUE(nDistinctColors(0, 42).empty());
}

TEST(DistincColors, nDistincColorsWantNGetN) {
  EXPECT_EQ(1, nDistinctColors(1).size());
  EXPECT_EQ(1, nDistinctColors(1, 255).size());
  EXPECT_EQ(1, nDistinctColors(1, 0).size());
  EXPECT_EQ(1, nDistinctColors(1, 42).size());

  EXPECT_EQ(42, nDistinctColors(42).size());
  EXPECT_EQ(42, nDistinctColors(42, 255).size());
  EXPECT_EQ(42, nDistinctColors(42, 0).size());
  EXPECT_EQ(42, nDistinctColors(42, 42).size());

  EXPECT_EQ(1e3, nDistinctColors(1e3).size());
  EXPECT_EQ(1e3, nDistinctColors(1e3, 255).size());
  EXPECT_EQ(1e3, nDistinctColors(1e3, 0).size());
  EXPECT_EQ(1e3, nDistinctColors(1e3, 42).size());
}

TEST(DistincColors, nDistinctColors) {
  const std::vector<cv::Scalar> colors = {{255, 0, 34, 255},
                                          {255, 192.973, 0, 255},
                                          {255, 124.054, 0, 255},
                                          {255, 55.1351, 0, 255},
                                          {13.7838, 255, 0, 255},
                                          {82.7027, 255, 0, 255},
                                          {151.622, 255, 0, 255},
                                          {220.541, 255, 0, 255},
                                          {0, 255, 220.726, 255},
                                          {0, 255, 152.177, 255},
                                          {0, 255, 83.6291, 255},
                                          {0, 255, 15.0807, 255},
                                          {0, 54.0489, 255, 255},
                                          {0, 123.342, 255, 255},
                                          {0, 192.636, 255, 255},
                                          {248.071, 0, 255, 255},
                                          {178.777, 0, 255, 255},
                                          {109.484, 0, 255, 255},
                                          {40.1902, 0, 255, 255},
                                          {255, 0, 220.353, 255}};
  const auto cs = nDistinctColors(colors.size());
  // for (const auto& c : cs) std::cout << c << std::endl;

  constexpr double eps = 5e-4;
  for (std::size_t i = 0; i < colors.size(); i++) {
    EXPECT_NEAR(colors[i][0], cs[i][0], eps);
    EXPECT_NEAR(colors[i][1], cs[i][1], eps);
    EXPECT_NEAR(colors[i][2], cs[i][2], eps);
    EXPECT_NEAR(colors[i][3], cs[i][3], eps);
  }
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
