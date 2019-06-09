#include "qr_code_detection.h"
#include <common/container.h>

THIRD_PARTY_HEADERS_BEGIN
#include <ros/console.h>
#include <boost/range/algorithm/for_each.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/core/eigen.hpp>
THIRD_PARTY_HEADERS_END

namespace ph = std::placeholders;

namespace perception {
namespace qr_code_detection {

const common::ParameterString<double> QrCodeDetection::MARKER_SIZE(
    "marker_size");

QrCodeDetection::QrCodeDetection(ParameterInterface *parameters)
    : parameter_interface_(parameters), camera_transformation_(parameters) {
  // register params
  parameter_interface_->registerParam(MARKER_SIZE);

  // Configure scanner
  scanner.set_config(zbar::ZBAR_NONE, zbar::ZBAR_CFG_ENABLE, 0);
  scanner.set_config(zbar::ZBAR_QRCODE, zbar::ZBAR_CFG_ENABLE, 1);

  scanner.set_config(zbar::ZBAR_QRCODE, zbar::ZBAR_CFG_X_DENSITY, 1);
  scanner.set_config(zbar::ZBAR_QRCODE, zbar::ZBAR_CFG_X_DENSITY, 1);
}

QrCode convert(const zbar::Symbol &symbol) {
  QrCode qr_code;

  qr_code.type = symbol.get_type_name();
  qr_code.data = symbol.get_data();

  // Obtain location
  qr_code.location.reserve(static_cast<size_t>(symbol.get_location_size()));
  for (int i = 0; i < symbol.get_location_size(); i++) {
    qr_code.location.emplace_back(symbol.get_location_x(static_cast<unsigned int>(i)),
                                  symbol.get_location_y(static_cast<unsigned int>(i)));
  }

  return qr_code;
}

QrCodes convertSymbols(zbar::SymbolSet symbols) {
  QrCodes qr_codes;
  qr_codes.reserve(static_cast<size_t>(symbols.get_size()));

  std::transform(
      symbols.symbol_begin(), symbols.symbol_end(), std::back_inserter(qr_codes), convert);

  return qr_codes;
}

void QrCodeDetection::estimatePose(QrCode &qr_code) {
  cv::Mat camera_matrix;
  cv::eigen2cv(camera_transformation_.getIntrinsicCalibrationMatrix(), camera_matrix);

  // TODO failsafe?
  assert(qr_code.location.size() == 4);

  cv::Mat rvec, tvec;

  const float w2 = static_cast<float>(parameter_interface_->getParam(MARKER_SIZE)) / 2;

  // For a QR Code, there will always be 4 points (logically) ordered counter-clockwise from the upper-left corner of the symbol.
  const std::vector<cv::Point3f> objectPoints{
      {w2, w2, 0.0f}, {-w2, w2, 0.0f}, {-w2, -w2, 0.0f}, {w2, -w2, 0.0f}};

  // TODO make sure that the qr_code.location points are in the correct order
  cv::solvePnP(objectPoints, qr_code.location, camera_matrix, {}, rvec, tvec);

  cv::Mat R;
  cv::Rodrigues(rvec, R);

  Eigen::Vector3f translation;
  Eigen::Matrix3f rotation;
  cv::cv2eigen(R, rotation);
  cv::cv2eigen(tvec, translation);

  qr_code.pose.setIdentity();
  qr_code.pose.translation() = translation;
  qr_code.pose.linear() = rotation;


  Eigen::Affine3f camera_T_vehicle;
  camera_T_vehicle.setIdentity();
  camera_T_vehicle.linear() = camera_transformation_.getRotationMatrix().cast<float>();
  camera_T_vehicle.translation() =
      camera_transformation_.getTranslationVector().cast<float>();

  qr_code.pose = camera_T_vehicle.inverse() * qr_code.pose;
}

QrCodes QrCodeDetection::detect(const cv::Mat &img) {
  assert(img.cols >= 0);
  assert(img.rows >= 0);

  // Wrap image data in a zbar image
  zbar::Image zbar_img(static_cast<unsigned int>(img.cols),
                       static_cast<unsigned int>(img.rows),
                       "Y800",
                       static_cast<const uchar *>(img.data),
                       static_cast<unsigned int>(img.cols * img.rows));

  // Scan the image for QR Codes
  scanner.scan(zbar_img);

  auto qr_codes = convertSymbols(zbar_img.get_symbols());

  // TODO ros_debug!
  ROS_INFO_STREAM("Found " << qr_codes.size() << " QR codes.");

  boost::for_each(qr_codes, std::bind(&QrCodeDetection::estimatePose, this, ph::_1));
  return qr_codes;
}

}  // namespace qr_code_detection
}  // namespace perception
