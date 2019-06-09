#ifndef QR_CODE_DETECTION_NODE_DEBUG_H
#define QR_CODE_DETECTION_NODE_DEBUG_H

#include "../qr_code_detection_node.h"
#include "common/camera_transformation.h"
#include "common/macros.h"

THIRD_PARTY_HEADERS_BEGIN
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
THIRD_PARTY_HEADERS_END

namespace perception {
namespace qr_code_detection {
namespace debug {

/*!
 * \brief The QrCodeDetectionNodeDebug class publishes an image with the detected QR codes
 */
class QrCodeDetectionNodeDebug : public QrCodeDetectionNode {
 public:
  QrCodeDetectionNodeDebug(ros::NodeHandle &node_handle);
  virtual ~QrCodeDetectionNodeDebug() override = default;

  virtual void startModule() override;
  virtual void stopModule() override;

  // QrCodeDetectionNode interface
 private:
  virtual void imageCallback(const sensor_msgs::ImageConstPtr &image_msg) override;
  virtual void publishQrCodes(const QrCodes &qr_codes,
                              const std_msgs::Header &header) const override;
  void drawQrCode(const QrCode &qr_code, cv::Mat &img) const;

  /*!
   * \brief createDebugImg draws the QR codes into the image
   * \param image_msg the input image with arbitrary color format
   * \param qr_codes a list of QR codes
   * \return image with overlayed QR codes
   */
  cv::Mat createDebugImg(const sensor_msgs::ImageConstPtr &image_msg,
                         const QrCodes &qr_codes) const;
  /*!
   * \brief publishDebugImg publishDebugImg publishs the overlayed color image
   * \param img the image to publish
   * \param header corresponding header
   */
  void publishDebugImg(const cv::Mat &img, const std_msgs::Header &header) const;

  sensor_msgs::ImageConstPtr current_img_;  //! updated via imageCallback

  ros::Publisher debug_img_publisher_;

  common::CameraTransformation camera_transformation_;
};

}  // namespace debug
}  // namespace qr_code_detection
}  // namespace perception

#endif  // QR_CODE_DETECTION_NODE_DEBUG_H
