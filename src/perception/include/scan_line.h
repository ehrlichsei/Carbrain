#ifndef SCAN_LINE_H
#define SCAN_LINE_H
#include <common/macros.h>

THIRD_PARTY_HEADERS_BEGIN
#include "opencv2/imgproc.hpp"
THIRD_PARTY_HEADERS_END

#include "opencv_eigen_conversions.h"
#include "perception_types.h"

/**
 * @brief The ScanLine struct
 *  specifies a scan line
 */
struct ScanLine {
  ScanLine() {}

  ScanLine(const ImagePoint& start, const ImagePoint& end)
      : start(start), end(end) {}
  /**
   * @brief start start point of the scan line
   */
  ImagePoint start;
  /**
   * @brief end end point of the scan line
   */
  ImagePoint end;

  ScanLine swapped() const { return ScanLine(end, start); }

  /*!
   * \brief trim trims the line by 'trim_factor' on both sides;
   * A trim_factor of 0.2 for example would shorten the line by 0.4 in total;
   * left and right can be interchanged
   * \param trim_factor element of (0, 1)
   * \return trimmed scan line
   */
  ScanLine trim(const double trim_factor) const {
    assert(trim_factor >= 0.0 && trim_factor < 0.5 &&
           "trim factor has to be in [0.0,0.5)");
    const ImagePointExact diff = (end - start).cast<double>();

    const auto trimmed_left = start.cast<double>() + trim_factor * diff;
    const auto trimmed_right = start.cast<double>() + (1 - trim_factor) * diff;

    return ScanLine(trimmed_left.cast<int>(), trimmed_right.cast<int>());
  }

  /** \brief Clips the line against the image rectangle.
   * The function clip calculates a part of the line segment that is entirely
   * within the specified rectangle. For further information, see \a
   *cv::clipLine().
   *
   * \param size the size. The image rectangle is
   *  Rect(0, 0, imgSize.width, imgSize.height).
   * \param scan_line [in|out] ScanLine that will be clipped in place
   * \return false if scan line is completely outside the rectangle. Otherwise,
   *  it returns true
   */
  template <typename T>
  static bool clip(const T& size, ScanLine& scan_line) {
    cv::Point p_start = toCV(scan_line.start);
    cv::Point p_end = toCV(scan_line.end);

    if (!cv::clipLine(size, p_start, p_end))
      return false;

    scan_line.start = toEigen(p_start);
    scan_line.end = toEigen(p_end);
    return true;
  }
};

/**
 * Type definition of a list of scan lines.
 */
using ScanLines = std::vector<ScanLine>;

#endif  // SCAN_LINE_H
