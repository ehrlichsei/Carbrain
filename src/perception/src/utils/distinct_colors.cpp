#include "distinct_colors.h"
#include <random>
#include <array>
#include <algorithm>

namespace perception {

struct RainbowGistEntry {
  // the position in the color map. postion is in [0, 1].
  float position;
  cv::Vec3f color;
};

/* source: http://matplotlib.org/examples/color/colormaps_reference.html */
static const std::array<RainbowGistEntry, 8> gist_rainbow = {
    {{0.000f, {1.00f, 0.00f, 0.16f}},
     {0.030f, {1.00f, 0.00f, 0.00f}},
     {0.215f, {1.00f, 1.00f, 0.00f}},
     {0.400f, {0.00f, 1.00f, 0.00f}},
     {0.586f, {0.00f, 1.00f, 1.00f}},
     {0.770f, {0.00f, 0.00f, 1.00f}},
     {0.954f, {1.00f, 0.00f, 1.00f}},
     {1.000f, {1.00f, 0.00f, 0.75f}}}};

cv::Scalar withAlpha(const cv::Vec3f& c, float alpha) {
  return {c[0], c[1], c[2], alpha};
}

cv::Vec3f interpolate(const RainbowGistEntry& before, const RainbowGistEntry& after, float l) {
  const float d = after.position - before.position;
  const float w_before = (l - before.position) / d;
  const float w_after = (after.position - l) / d;
  return w_before * before.color + w_after * after.color;
}

std::vector<cv::Scalar> nDistinctColors(unsigned int n, float alpha) {
  std::vector<cv::Scalar> colors;
  colors.reserve(n);
  // cutting ends of gist_rainbow to use red as remaining debug color
  //  float a = 0.115;
  //  float b = 0.954;
  const float a = 0.0f;
  const float b = 1.0f;
  // interpolate RGB colors
  for (unsigned int i = 0; i < n; i++) {
    const float l =
        (((static_cast<float>(i) + 0.5f) / static_cast<float>(n)) * (b - a)) + a;
    const auto it = std::lower_bound(
        std::begin(gist_rainbow),
        std::end(gist_rainbow),
        l,
        [](const auto& x, const auto& y) { return x.position < y; });
    colors.emplace_back(withAlpha(interpolate(*std::prev(it), *it, l) * 255.f, alpha));
  }
  return colors;
}

cv::Scalar hsv2bgr(const cv::Scalar& hsv) {
  const cv::Mat3b hsvmat(cv::Vec3b(
      static_cast<uchar>(hsv[0]), static_cast<uchar>(hsv[1]), static_cast<uchar>(hsv[2])));
  cv::Mat3b bgrmat;
  cv::cvtColor(hsvmat, bgrmat, CV_HSV2BGR);
  return withAlpha(bgrmat.at<cv::Vec3b>(0, 0), static_cast<float>(hsv[3]));
}

class BrightColorGenerator {
 public:
  cv::Scalar operator()() {
    return hsv2bgr(cv::Scalar(uchar_dist(gen),          // Hue
                              uchar55_dist(gen) + 200,  // Saturation
                              uchar55_dist(gen) + 200,  // Value
                              255));                    // Alpha
  }

 private:
  std::mt19937 gen{std::random_device{}()};
  std::uniform_int_distribution<int> uchar_dist{0, 255};
  std::uniform_int_distribution<int> uchar55_dist{0, 55};
};

cv::Scalar randomBrightColor() { return BrightColorGenerator()(); }

std::vector<cv::Scalar> randomBrightColors(unsigned int n) {
  std::vector<cv::Scalar> colors(n);
  std::generate_n(std::begin(colors), n, BrightColorGenerator());
  return colors;
}

}  // namespace perception
