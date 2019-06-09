#include "extrinsic_calibration_node.h"
#include "extrinsic_calibration.h"

//#include <opencv2/core.hpp>
//#include <opencv2/calib3d.hpp>
#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/calib3d/calib3d.hpp"

#include "common/node_creation_makros.h"

ExtrinsicCalibrationNode::ExtrinsicCalibrationNode(ros::NodeHandle& node_handle)
    : NodeBase(node_handle), extrinsic_calibration_(&parameter_handler_) {
}

void ExtrinsicCalibrationNode::startModule() {
  // sets your node in running mode. Activate publishers, subscribers, service
  // servers, etc here.
  image_transport::ImageTransport img_trans(node_handle_);
  image_raw_sub_ = img_trans.subscribe(
      "image_raw", 1, &ExtrinsicCalibrationNode::handleImages, this);
  rospub_img_debug_ = node_handle_.advertise<sensor_msgs::Image>("img_debug", 1);
}

void ExtrinsicCalibrationNode::stopModule() {
  // sets your node in idle mode. Deactivate publishers, subscribers, service
  // servers, etc here.
  image_raw_sub_.shutdown();
  rospub_img_debug_.shutdown();
}

void ExtrinsicCalibrationNode::handleImages(const sensor_msgs::ImageConstPtr& image_raw_msg) {
  // converting from ros image type to cv::Mat
  cv_bridge::CvImageConstPtr cv_ptr_raw;
  try {
    cv_ptr_raw = cv_bridge::toCvShare(image_raw_msg, sensor_msgs::image_encodings::MONO8);
  } catch (cv_bridge::Exception& e) {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }
  // Datentyp OpenCV --> mit cv_ptr_raw->image (Zeiger auf Adresse vom Bild)
  cv::Mat image = cv_ptr_raw->image;
  cv::Mat img_debug = image.clone();
  // Converts an image from one color space to another.
  cv::cvtColor(img_debug, img_debug, CV_GRAY2BGR);

  extrinsic_calibration_.printExtrinsicParams(image, img_debug);
  // Funktionsaufruf zur Implementierung hier

  // publish img_debug
  cv_bridge::CvImage debug_msg;
  debug_msg.header = std_msgs::Header();
  debug_msg.encoding = sensor_msgs::image_encodings::BGR8;
  debug_msg.image = img_debug;
  rospub_img_debug_.publish(debug_msg.toImageMsg());
}

const std::string ExtrinsicCalibrationNode::getName() {
  return std::string("extrinsic_calibration");
}

CREATE_NODE(ExtrinsicCalibrationNode)
