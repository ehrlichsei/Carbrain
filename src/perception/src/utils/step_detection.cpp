#include "step_detection.h"
#include <common/macros.h>

THIRD_PARTY_HEADERS_BEGIN
#include <opencv2/imgproc.hpp>
THIRD_PARTY_HEADERS_END

#include "opencv_eigen_conversions.h"
#include "common/basic_statistics.h"

namespace step_detection {

ImagePoints detectStep(const cv::Mat &img,
            const ScanLine& scan_line,
            const std::vector<float> &reference_function,
            const float threshold,
            const bool use_abs) {
  /*** Method: Cross correlation with reference function, predefined length ***/
  ImagePoints feature_points;

  const std::size_t range_length = reference_function.size();
  const std::size_t range_mid = (range_length - 1) / 2;

  // fill vector with complete signal using line iterator
  cv::LineIterator signal_iterator(img, toCV(scan_line.start), toCV(scan_line.end), 4);
  if (signal_iterator.count <= static_cast<int>(range_length)) {
    return feature_points;
  }

  std::vector<std::size_t> signal(static_cast<std::size_t>(signal_iterator.count));
  ImagePoints signal_pos(static_cast<std::size_t>(signal_iterator.count));
  for (std::size_t j = 0; j < signal.size(); j++, signal_iterator++) {
    signal[j] = **signal_iterator;
    signal_pos[j] = toEigen(signal_iterator.pos());
  }

  // iterate through scan line points and check correlation of each local
  // signal (j, j+range) with reference
  float best_correlation_in_range = 0.0;
  std::size_t best_correlation_in_range_j = 0;
  bool best_correlation_waiting = false;

  for (std::size_t j = 0; j < signal.size() - range_length - 1; j++) {

    const float range_mean = common::mean<float>(&signal[j], &signal[j + range_length]);

    // compute cross correlation
    float cross_correlation = 0;
    for (std::size_t k = 0; k < range_length; k++) {
      cross_correlation += (float(signal[j + k]) - range_mean) * reference_function[k];
    }
    cross_correlation /= static_cast<float>(range_length);
    if (use_abs) {
      cross_correlation = std::abs(cross_correlation);
    }

    // in the following: non maxima suppression within 'range'
    // do net add point direcly but remember it for range_length points: if it
    // has the highest
    // correlation of these points, add it; if not do the same for the point
    // with the new
    // highest correlation
    if (best_correlation_waiting && (j - best_correlation_in_range_j > range_length)) {
      // if not already exited range
      // add best_correlation as we have exited the range
      feature_points.push_back(signal_pos[best_correlation_in_range_j + range_mid]);
      best_correlation_waiting = false;
      best_correlation_in_range = 0.0;
    } else if (cross_correlation > threshold && best_correlation_in_range < cross_correlation) {
      best_correlation_in_range = cross_correlation;
      best_correlation_in_range_j = j;
      best_correlation_waiting = true;
    }
  }  // end range for loop
  return feature_points;
}


std::vector<float> createReferenceFunction(const std::size_t size) {
  // tanh function, scaled so that values contain tanh(x) values for x in
  //    [-1.5, 1.5]

  std::vector<float> reference_function(size);
  const double abs_tanh_max = 1.5;
  const double half_size = floor(static_cast<double>(size) / 2.0);
  for (std::size_t i = 0; i < size; i++) {
    const double x = (static_cast<double>(i) - half_size) * (abs_tanh_max / half_size);
    reference_function[i] = static_cast<float>(tanh(x));
  }
  return reference_function;
}

std::vector<float> invertReferenceFunction(std::vector<float> reference_function) {
  std::reverse(reference_function.begin(), reference_function.end());
  return reference_function;
}
} // namespace step_detection
