#ifndef QR_CODE_DETECTION_H
#define QR_CODE_DETECTION_H
#include <common/macros.h>

THIRD_PARTY_HEADERS_BEGIN
#include <stddef.h>  // needed for 16.04
#include <zbar.h>
#include <Eigen/Geometry>
#include <opencv2/core/mat.hpp>
THIRD_PARTY_HEADERS_END

#include "common/camera_transformation.h"
#include "common/parameter_interface.h"

namespace perception {
namespace qr_code_detection {


struct QrCode {
  std::string type;  //! Type of code. Here, it should be always 'QR-CODE'
  std::string data;  //! code of the QR code e.g. 'STOP'
  std::vector<cv::Point2f> location;  //! polygon of points describing the contour in the image
  /*!
   * \brief the QR code pose in vehicle frame
   *
   *  The coordinate frame is as follows
   *
   *                 ^ x
   *                 |
   *                 |
   *       ▄▄▄▄▄▄▄  ▄ ▄▄ ▄▄▄▄▄▄▄
   *       █ ▄▄▄ █ ██ ▀▄ █ ▄▄▄ █
   *       █ ███ █ ▄▀ ▀▄ █ ███ █
   *       █▄▄▄▄▄█ █ ▄▀█ █▄▄▄▄▄█
   *  y     ▄▄ ▄  ▄▄▀██▀▀ ▄▄▄ ▄▄
   *  <--   ▀▀██▄ ▄█▀▀▄ ▄▄▀  ▀█▄
   *       █▀▄   ▄█▀█ ▀▄ ▄ ▄ ▀▀█
   *       ▄▄▄▄▄▄▄ █▄ ▀ ▄ █▀█ █
   *       █ ▄▄▄ █  █▄█▀▀▄██▀▄██
   *       █ ███ █ ▀██▀▄   ▄▀▀▀█
   *       █▄▄▄▄▄█ █▄▄▄▄▀▀ █▄█ ▀

   *
   */
  Eigen::Affine3f pose;
};

using QrCodes = std::vector<QrCode>;

/*!
 * \brief Node to detect QR codes like the 'STOP' signal
 */
class QrCodeDetection {
 public:
  /*!
   * \brief QrCodeDetection is the consstructor. A ros indipendent functionality
   * containing class needs a pointer to a ParameterInterface (in fact a
   * ParameterHandler) to get access to parameters.
   *
   * This is highly inspired by
   * https://www.learnopencv.com/barcode-and-qr-code-scanner-using-zbar-and-opencv/
   */
  QrCodeDetection(common::ParameterInterface *parameters);

  /*!
   * \brief detects QR codes in the image
   * \param img grayscale image
   * \return a list of detected QR codes
   */
  QrCodes detect(const cv::Mat &img);

  //! side length of the QR code needed to estimate the pose
  const static common::ParameterString<double> MARKER_SIZE;

 private:
  /*!
   * \brief estimatePose based on the detected QR code corner locations in the
   * image estimates the 3D pose of the marker.
   * \param qr_code qr-code, that has been detected
   */
  void estimatePose(QrCode &qr_code);

  /*!
   * \brief parameters_ptr_ is needed for parameter access
   */
  common::ParameterInterface *parameter_interface_;
  common::CameraTransformation camera_transformation_;

  zbar::ImageScanner scanner;
};

}  // namespace qr_code_detection
}  // namespace perception

#endif  // QR_CODE_DETECTION_H
