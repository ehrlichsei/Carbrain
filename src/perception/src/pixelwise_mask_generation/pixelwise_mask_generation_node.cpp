#include "pixelwise_mask_generation_node.h"
#include <common/macros.h>

THIRD_PARTY_HEADERS_BEGIN
#include <cv_bridge/cv_bridge.h>
#include <ros/ros.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/opencv.hpp>
#include "perception/PreprocessingPixelwiseMaskConfig.h"
THIRD_PARTY_HEADERS_END

#include "common/node_creation_makros.h"


namespace enc = sensor_msgs::image_encodings;

namespace pixelwise_mask_generation {

const ParameterString<std::string> PixelwiseMaskGenerationNode::STANDARD_MASK_DIRECTORY(
    "/perception/ego_vehicle/pixelwise_mask/standard_mask_directory");
const ParameterString<std::string> PixelwiseMaskGenerationNode::STANDARD_MASK_NAME(
    "/perception/ego_vehicle/pixelwise_mask/standard_mask_name");

PixelwiseMaskGenerationNode::PixelwiseMaskGenerationNode(ros::NodeHandle &node_handle)
    : NodeBase(node_handle), pixelwise_mask_generation_(&parameter_handler_) {
  parameter_handler_.registerParam(STANDARD_MASK_DIRECTORY);
  parameter_handler_.registerParam(STANDARD_MASK_NAME);

  show_old_mask = true;
  mask_loaded = false;
}

void PixelwiseMaskGenerationNode::startModule() {
  // sets your node in running mode. Activate publishers, subscribers, service
  // servers, etc here.
  image_transport::ImageTransport img_trans(node_handle_);
  rossub_image = img_trans.subscribe(
      "image_raw", 1, &PixelwiseMaskGenerationNode::handleImage, this);
  rospub_mask_image = node_handle_.advertise<sensor_msgs::Image>("mask_image", 1);
  rospub_image_mask_contour =
      node_handle_.advertise<sensor_msgs::Image>("image_mask_contour", 1);

  serv_save_mask = node_handle_.advertiseService(
      "save_mask", &pixelwise_mask_generation::PixelwiseMaskGenerationNode::saveMaskCallback, this);
  serv_median_blur = node_handle_.advertiseService(
      "median_blur",
      &pixelwise_mask_generation::PixelwiseMaskGenerationNode::medianBlurCallback,
      this);
  serv_automatic_mask_gen = node_handle_.advertiseService(
      "automatic_mask_gen",
      &pixelwise_mask_generation::PixelwiseMaskGenerationNode::automaticMaskGenCallback,
      this);
  serv_take_pictures = node_handle_.advertiseService(
      "take_pictures",
      &pixelwise_mask_generation::PixelwiseMaskGenerationNode::takePicturesCallback,
      this);
  serv_dilation = node_handle_.advertiseService(
      "dilation", &pixelwise_mask_generation::PixelwiseMaskGenerationNode::dilationCallback, this);
  serv_closing = node_handle_.advertiseService(
      "closing", &pixelwise_mask_generation::PixelwiseMaskGenerationNode::closingCallback, this);
  serv_bilateral_filter = node_handle_.advertiseService(
      "bilateral_filter",
      &pixelwise_mask_generation::PixelwiseMaskGenerationNode::bilateralFilterCallback,
      this);
  serv_set_starting_point = node_handle_.advertiseService(
      "set_starting_point",
      &pixelwise_mask_generation::PixelwiseMaskGenerationNode::setStartingPointCallback,
      this);
}

void PixelwiseMaskGenerationNode::stopModule() {
  // sets your node in idle mode. Deactivate publishers, subscribers, service
  // servers, etc here.
  rossub_image.shutdown();
  rospub_mask_image.shutdown();
  rospub_image_mask_contour.shutdown();
  serv_save_mask.shutdown();
  serv_median_blur.shutdown();
  serv_automatic_mask_gen.shutdown();
  serv_take_pictures.shutdown();
  serv_dilation.shutdown();
  serv_closing.shutdown();
  serv_bilateral_filter.shutdown();
  serv_set_starting_point.shutdown();
}



void PixelwiseMaskGenerationNode::handleImage(const sensor_msgs::ImageConstPtr &msg) {

  cv_bridge::CvImageConstPtr cv_ptr;
  try {
    cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::MONO8);
  } catch (cv_bridge::Exception &e) {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }

  cv_bridge::CvImage out_msg_mask;
  cv_bridge::CvImage out_msg_mask_contour;

  out_msg_mask_contour.header = msg->header;
  out_msg_mask_contour.encoding = enc::BGR8;

  if (pixelwise_mask_generation_.generateMask(cv_ptr->image, out_msg_mask.image)) {

    out_msg_mask.header = msg->header;
    out_msg_mask.encoding = enc::MONO8;

    sensor_msgs::ImagePtr out_ptr_mask_image(out_msg_mask.toImageMsg());
    rospub_mask_image.publish(out_ptr_mask_image);

    const cv::Scalar colour(0, 0, 255);
    pixelwise_mask_generation_.generate_image_mask_contour(
        cv_ptr->image, out_msg_mask.image, out_msg_mask_contour.image, colour);
    cv::putText(out_msg_mask_contour.image, "new mask", cv::Point(44, 44), cv::FONT_HERSHEY_PLAIN, 3, colour);
    sensor_msgs::ImagePtr out_ptr_image_mask_contour(out_msg_mask_contour.toImageMsg());
    rospub_image_mask_contour.publish(out_ptr_image_mask_contour);
    show_old_mask = false;
  } else if (show_old_mask) {

    if (!mask_loaded) {
      std::string mask_directory =
          parameter_handler_.getParam(PixelwiseMaskGeneration::STANDARD_MASK_DIRECTORY);
      std::string mask_name =
          parameter_handler_.getParam(PixelwiseMaskGeneration::STANDARD_MASK_NAME);

      if (!mask_directory.empty() && mask_directory.front() == '~' && getenv("HOME")) {
        mask_directory.replace(0, 1, getenv("HOME"));
      }

      if (mask_directory.back() != '/') {
        mask_directory = mask_directory + "/";
      }

      old_mask = cv::imread(mask_directory + mask_name, 0);
      cv::bitwise_not(old_mask, old_mask);
      mask_loaded = true;
    }

    if (old_mask.data) {
      const cv::Scalar colour(0, 255, 0);
      pixelwise_mask_generation_.generate_image_mask_contour(
          cv_ptr->image, old_mask, out_msg_mask_contour.image, colour);
      cv::putText(
          out_msg_mask_contour.image, "mask file", cv::Point(44, 44), cv::FONT_HERSHEY_PLAIN, 3, colour);
      sensor_msgs::ImagePtr out_ptr_image_mask_contour(out_msg_mask_contour.toImageMsg());
      rospub_image_mask_contour.publish(out_ptr_image_mask_contour);
    }
  }
}

const std::string PixelwiseMaskGenerationNode::getName() {
  return std::string("pixelwise_mask_generation");
}

bool PixelwiseMaskGenerationNode::takePicturesCallback(
    pixelwise_mask_generation_msgs::TakePictures::Request &req,
    pixelwise_mask_generation_msgs::TakePictures::Response &) {
  if (req.numberOfPictures < 1 || req.numberOfPictures > 200) {
    ROS_ERROR("numberOfPictures has to be bigger than 0 and smaller than 201");
    return true;
  }
  pixelwise_mask_generation_.setTakePictures(req.numberOfPictures);
  return true;
}

bool PixelwiseMaskGenerationNode::saveMaskCallback(
    pixelwise_mask_generation_msgs::SaveMask::Request &,
    pixelwise_mask_generation_msgs::SaveMask::Response &) {
  pixelwise_mask_generation_.setSaveMask();
  return true;
}

bool PixelwiseMaskGenerationNode::automaticMaskGenCallback(
    pixelwise_mask_generation_msgs::AutomaticMaskGen::Request &,
    pixelwise_mask_generation_msgs::AutomaticMaskGen::Response &) {
  pixelwise_mask_generation_.setAutomaticMaskGen();
  return true;
}

bool PixelwiseMaskGenerationNode::medianBlurCallback(
    pixelwise_mask_generation_msgs::MedianBlur::Request &req,
    pixelwise_mask_generation_msgs::MedianBlur::Response &) {
  if (req.ksize % 2 == 0) {
    ROS_ERROR("ksize for medianBlur has to be odd");
    return true;
  }
  if (req.ksize > 80) {
    ROS_ERROR("ksize has to be smaller than 80");
    return true;
  }
  pixelwise_mask_generation_.setMedianBlur(req.ksize);
  return true;
}

bool PixelwiseMaskGenerationNode::dilationCallback(
    pixelwise_mask_generation_msgs::Dilation::Request &req,
    pixelwise_mask_generation_msgs::Dilation::Response &) {
  pixelwise_mask_generation_.setDilation(req.dilation_size);
  return true;
}

bool PixelwiseMaskGenerationNode::closingCallback(
    pixelwise_mask_generation_msgs::Closing::Request &req,
    pixelwise_mask_generation_msgs::Closing::Response &) {
  pixelwise_mask_generation_.setClosing(req.closing_size);
  return true;
}

bool PixelwiseMaskGenerationNode::bilateralFilterCallback(
    pixelwise_mask_generation_msgs::BilateralFilter::Request &req,
    pixelwise_mask_generation_msgs::BilateralFilter::Response &) {

  if (req.bilateral_filter_size < 2 || req.bilateral_filter_size > 10) {
    ROS_ERROR(
        "the bilateral filter size has to be bigger than 1 and smaller than "
        "11");
    return true;
  }
  pixelwise_mask_generation_.setBilateralFilter(req.bilateral_filter_size,
                                                req.bilateral_filter_sigma);
  return true;
}

bool PixelwiseMaskGenerationNode::setStartingPointCallback(
    pixelwise_mask_generation_msgs::SetStartingPoint::Request &req,
    pixelwise_mask_generation_msgs::SetStartingPoint::Response &) {
  pixelwise_mask_generation_.setStartingPoint(req.x, req.y);
  return true;
}

} // namespace pixelwise_mask_generation

CREATE_NODE(pixelwise_mask_generation::PixelwiseMaskGenerationNode)
