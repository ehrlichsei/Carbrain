#ifndef STEP_DETECTION_H
#define STEP_DETECTION_H
#include <common/macros.h>

THIRD_PARTY_HEADERS_BEGIN
#include <opencv2/core/core.hpp>
THIRD_PARTY_HEADERS_END

#include "perception_types.h"
#include "scan_line.h"

namespace step_detection {
/**
 * \param use_abs use absolute value for comparison with threshold
 */
ImagePoints detectStep(const cv::Mat& img,
                       const ScanLine& scan_line,
                       const std::vector<float>& reference_function,
                       const float threshold,
                       const bool use_abs);

/*!
 * \brief createReferenceFunction returns discrete values from reference
 * function (currently tanh for mode==1) to be correlated with in
 * function 'apply1dGradientDetector'
 * \param size number of descrete values to be retured; has to be odd!
 * see function 'apply1dGradientDetector'
 */
std::vector<float> createReferenceFunction(const std::size_t size);

std::vector<float> invertReferenceFunction(std::vector<float> reference_function);
};

#endif  // STEP_DETECTION_H
