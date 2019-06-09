#include "qr_code_detection_node_debug.h"
#include "common/macros.h"

THIRD_PARTY_HEADERS_BEGIN
#include <cv_bridge/cv_bridge.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <boost/range/algorithm.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/core/eigen.hpp>
THIRD_PARTY_HEADERS_END

namespace perception {
namespace qr_code_detection {
namespace debug {

QrCodeDetectionNodeDebug::QrCodeDetectionNodeDebug(ros::NodeHandle &node_handle)
    : QrCodeDetectionNode(node_handle), camera_transformation_(&parameter_handler_) {}

void QrCodeDetectionNodeDebug::startModule() {
  QrCodeDetectionNode::startModule();
  debug_img_publisher_ =
      node_handle_.advertise<sensor_msgs::Image>("qr_code_img", 1);
}

void QrCodeDetectionNodeDebug::stopModule() {
  QrCodeDetectionNode::stopModule();
  debug_img_publisher_.shutdown();
}

void QrCodeDetectionNodeDebug::imageCallback(const sensor_msgs::ImageConstPtr &image_msg) {
  current_img_ = image_msg;

  QrCodeDetectionNode::imageCallback(image_msg);
}

void QrCodeDetectionNodeDebug::publishQrCodes(const QrCodes &qr_codes,
                                              const std_msgs::Header &header) const {

  auto img = createDebugImg(current_img_, qr_codes);
  publishDebugImg(img, current_img_->header);

  QrCodeDetectionNode::publishQrCodes(qr_codes, header);
}

void QrCodeDetectionNodeDebug::drawQrCode(const QrCode &qr_code, cv::Mat &img) const {
  // draw bounding rect
  std::vector<cv::Point> point_locs;
  point_locs.reserve(qr_code.location.size());

  boost::transform(
      qr_code.location, std::back_inserter(point_locs), [](const auto &pt) -> cv::Point {
        return {static_cast<int>(pt.x), static_cast<int>(pt.y)};
      });

  std::vector<std::vector<cv::Point>> pts{point_locs};

  cv::polylines(img, pts, true, cv::Scalar(255, 0, 0), 2);

  // draw pose marker

  Eigen::Affine3f camera_T_vehicle;
  camera_T_vehicle.setIdentity();
  camera_T_vehicle.linear() = camera_transformation_.getRotationMatrix().cast<float>();
  camera_T_vehicle.translation() =
      camera_transformation_.getTranslationVector().cast<float>();

  Eigen::Affine3f camera_T_qr_code = camera_T_vehicle * qr_code.pose;
  cv::Mat R, tvec;
  cv::eigen2cv(Eigen::Matrix3f{camera_T_qr_code.linear()}, R);
  cv::eigen2cv(Eigen::Vector3f{camera_T_qr_code.translation()}, tvec);

  cv::Mat rvec;
  cv::Rodrigues(R, rvec);

  cv::Mat cam_calib;
  cv::eigen2cv(camera_transformation_.getIntrinsicCalibrationMatrix(), cam_calib);

  cv::aruco::drawAxis(
      img, cam_calib, {}, rvec, tvec, static_cast<float>(parameter_handler_.getParam(QrCodeDetection::MARKER_SIZE)));
}

cv::Mat QrCodeDetectionNodeDebug::createDebugImg(const sensor_msgs::ImageConstPtr &image_msg,
                                                 const QrCodes &qr_codes) const {

  auto cv_img_ptr = cv_bridge::toCvCopy(image_msg, "bgr8");

  boost::for_each(
      qr_codes,
      boost::bind(
          &QrCodeDetectionNodeDebug::drawQrCode, this, _1, boost::ref(cv_img_ptr->image)));

  return cv_img_ptr->image;
}

void QrCodeDetectionNodeDebug::publishDebugImg(const cv::Mat &img,
                                               const std_msgs::Header &header) const {
  cv_bridge::CvImage cv_img(header, "bgr8", img);
  debug_img_publisher_.publish(cv_img.toImageMsg());
}

}  // namespace debug
}  // namespace qr_code_detection
}  // namespace perception
