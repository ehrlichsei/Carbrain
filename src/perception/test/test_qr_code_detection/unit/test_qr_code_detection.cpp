#include <common/camera_transformation_parameters.h>
#include <common/macros.h>
#include <common/test/dummy_parameter_handler.h>

#include "../src/qr_code_detection/qr_code_detection.h"

THIRD_PARTY_HEADERS_BEGIN
#include <gtest/gtest.h>
#include <ros/package.h>
#include <opencv2/core.hpp>
#include <opencv2/opencv.hpp>
THIRD_PARTY_HEADERS_END

DISABLE_SUGGEST_OVERRIDE_WARNING

using namespace common;

namespace perception {
namespace qr_code_detection {
namespace testing {

class QrCodeDetectionTest : public ::testing::Test {};

cv::Mat loadTestImage() {
  const std::string test_data_dir = ros::package::getPath("perception") +
                                    "/test/test_qr_code_detection/unit/data/";
  return cv::imread(test_data_dir + "qr_code_stop.png", cv::IMREAD_GRAYSCALE);
}

class MyDummyParameterHandler : public DummyParameterHandler {
 public:
  MyDummyParameterHandler() {
    addParam(CAMERA_FOCAL_LENGTH_X, 383.456503);
    addParam(CAMERA_FOCAL_LENGTH_Y, 382.843594);
    addParam(CAMERA_OPTICAL_CENTER_X, 634.13738);
    addParam(CAMERA_OPTICAL_CENTER_Y, 142.480292);

    addParam(CAMERA_R11, 0.000454787);
    addParam(CAMERA_R12, -0.999948);
    addParam(CAMERA_R13, 0.0102272);
    addParam(CAMERA_R21, -0.186011);
    addParam(CAMERA_R22, -0.0101333);
    addParam(CAMERA_R23, -0.982495);
    addParam(CAMERA_R31, 0.982548);
    addParam(CAMERA_R32, -0.00145555);
    addParam(CAMERA_R33, -0.186006);

    addParam(CAMERA_T1, 0.126102);
    addParam(CAMERA_T2, 263.795);
    addParam(CAMERA_T3, 40.1794);

    addParam(QrCodeDetection::MARKER_SIZE, 0.15);
  }
};

TEST_F(QrCodeDetectionTest, QrCodeStopTest) {

  // The test image shows a QR Code with label 'STOP'
  const cv::Mat test_img = loadTestImage();
  EXPECT_FALSE(test_img.empty());

  EXPECT_GT(test_img.size().area(), 0);

  MyDummyParameterHandler parameter_handler;
  QrCodeDetection qr_code_detection(&parameter_handler);

  const auto qr_codes = qr_code_detection.detect(test_img);

  EXPECT_EQ(qr_codes.size(), 1);

  if (qr_codes.empty()) {
    return;
  }

  EXPECT_EQ(qr_codes.front().data, "STOP");
}

}  // namespace testing
}  // namespace qr_code_detection
}  // namespace perception

using namespace perception::qr_code_detection::testing;

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
