#ifndef DISTINCT_COLORS
#define DISTINCT_COLORS
#include <common/macros.h>

THIRD_PARTY_HEADERS_BEGIN
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
THIRD_PARTY_HEADERS_END

namespace perception {
// clang-format off
/*!
 * \brief nDistinctColors generates a vector containing n distinct colors similar to matlabs rainbow gist
 * <a href="https://matplotlib.org/examples/color/colormaps_reference.html"> (or matplotlibs)</a>.
 *
 * \param n the number of colors.
 * \param alpha the intransparency of the colors (<a href="https://en.wikipedia.org/wiki/Alpha_compositing"> alpha compositiong</a>):
 * \return a vector of colors.
 */
// clang-format on
std::vector<cv::Scalar> nDistinctColors(unsigned int n, float alpha = 255);

/*!
 * \brief randomBrightColor creates a randomly choosen 'bright' color.
 * 'bright' means high value and saturation: value >= 200 and saturation > 200.
 * \return a vector of colors.
 */
cv::Scalar randomBrightColor();

/*!
 * \brief randomBrightColors creates an vector of randomly choosen 'bright'
 *  colors.
 *
 * 'bright' means high value and saturation: value >= 200 and saturation > 200.
 *
 * \param n the number of colors.
 * \return a vector of colors.
 */
std::vector<cv::Scalar> randomBrightColors(unsigned int n);

}  // namespace perception
#endif  // DISTINCT_COLORS
