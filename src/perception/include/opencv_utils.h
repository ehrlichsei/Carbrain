#ifndef PERCEPTION_OPENCV_UTILS_H
#define PERCEPTION_OPENCV_UTILS_H

#include<opencv2/core/mat.hpp>

template <typename Container>
cv::_InputArray toInputArray(Container &c) {
  return cv::_InputArray(c.data(), c.size());
}

#endif // PERCEPTION_OPENCV_UTILS_H
