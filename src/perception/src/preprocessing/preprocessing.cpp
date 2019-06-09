#include "preprocessing.h"
#include <common/macros.h>

THIRD_PARTY_HEADERS_BEGIN
#include <opencv2/opencv.hpp>
#include <ros/ros.h>
THIRD_PARTY_HEADERS_END

#define WHITE_PIXEL 255  // Value of white pixels in cv::Mat

const ParameterString<int> Preprocessing::BINARISTATION_MODE(
    "binarisation/mode");
const ParameterString<double> Preprocessing::BINARISTATION_SIGMA(
    "binarisation/sigma");
const ParameterString<int> Preprocessing::BINARISTATION_BLOCK_SIZE(
    "binarisation/blocksize");
const ParameterString<double> Preprocessing::BINARISTATION_C("binarisation/c");
const ParameterString<int> Preprocessing::MASK_MODE(
    "/perception/ego_vehicle/mask_mode");
const ParameterString<std::string> Preprocessing::STANDARD_MASK_DIRECTORY(
    "/perception/ego_vehicle/pixelwise_mask/standard_mask_directory");
const ParameterString<std::string> Preprocessing::STANDARD_MASK_NAME(
    "/perception/ego_vehicle/pixelwise_mask/standard_mask_name");
const ParameterString<std::string> Preprocessing::MODIFIED_MASK_PATH(
    "/perception/ego_vehicle/pixelwise_mask/modified_mask_path");
const ParameterString<int> Preprocessing::ROI_START_LOC_X("roi/start_loc_x");
const ParameterString<int> Preprocessing::ROI_START_LOC_Y("roi/start_loc_y");
const ParameterString<int> Preprocessing::ROI_END_LOC_X("roi/end_loc_x");
const ParameterString<int> Preprocessing::ROI_END_LOC_Y("roi/end_loc_y");
const ParameterString<int> Preprocessing::MEDIAN_BLUR_SIZE(
    "median_blur/median_blur_size");

void Preprocessing::registerPreprocessingParameters(ParameterInterface* parameter_interface) {
  parameter_interface->registerParam(BINARISTATION_MODE);
  parameter_interface->registerParam(BINARISTATION_SIGMA);
  parameter_interface->registerParam(BINARISTATION_BLOCK_SIZE);
  parameter_interface->registerParam(BINARISTATION_C);
  parameter_interface->registerParam(ROI_START_LOC_X);
  parameter_interface->registerParam(ROI_START_LOC_Y);
  parameter_interface->registerParam(ROI_END_LOC_X);
  parameter_interface->registerParam(ROI_END_LOC_Y);
  parameter_interface->registerParam(MASK_MODE);
  parameter_interface->registerParam(STANDARD_MASK_DIRECTORY);
  parameter_interface->registerParam(STANDARD_MASK_NAME);
  parameter_interface->registerParam(MODIFIED_MASK_PATH);
  parameter_interface->registerParam(MEDIAN_BLUR_SIZE);
}

Preprocessing::Preprocessing(ParameterInterface* parameters)
    : parameters_ptr_(parameters), ego_vehicle(parameters) {
  registerPreprocessingParameters(parameters);
  mask_mode = parameters_ptr_->getParam(MASK_MODE);
  refreshMask();
}

void Preprocessing::preprocessImage(const cv::Mat& image_gray, cv::Mat& preprocessed_image) {

  // Updating image limits and cut mask to ROI
  updateImageLimits();
  ego_vehicle.update(image_limits);

  // Updating mask mode and reloading mask file if necessary
  if (mask_mode != parameters_ptr_->getParam(MASK_MODE)) {
    mask_mode = parameters_ptr_->getParam(MASK_MODE);
    refreshMask();
  }

  // cutting out region of interest
  cv::Mat image_roi = image_gray(image_limits);

  // (optional) binarization
  const int binarisation_mode = parameters_ptr_->getParam(BINARISTATION_MODE);

  if (binarisation_mode == 0) {
    // no binarization
    image_roi.copyTo(preprocessed_image);
  } else if (binarisation_mode == 1) {
    binarizeImageThresh(image_roi, parameters_ptr_->getParam(BINARISTATION_SIGMA), preprocessed_image);
  } else if (binarisation_mode == 2 || binarisation_mode == 3) {
    // 3 case if for compatibility with older rosbags
    binarizeAdaptiveThreshold(image_roi,
                              parameters_ptr_->getParam(BINARISTATION_BLOCK_SIZE),
                              parameters_ptr_->getParam(BINARISTATION_SIGMA),
                              parameters_ptr_->getParam(BINARISTATION_C),
                              preprocessed_image);
  } else {
    binarizeImageOtsu(image_roi, preprocessed_image);
  }

  // set parts of car black
  if (use_trapezoid_mask_mode) {
    cv::fillConvexPoly(
        preprocessed_image, ego_vehicle.asInputArray(), cv::Scalar::all(0), 8, 0);
  } else {
    try {
      // set the parts of car black, which are 1 in the mask file (mask_roi)
      preprocessed_image.setTo(0, mask_roi);
    } catch (cv::Exception& e) {
      ROS_ERROR("exception while applying pixelwise mask to image: %s", e.what());
      ROS_WARN("Preprocessing continues by using trapezoid mask mode.");
      use_trapezoid_mask_mode = true;
      preprocessed_image.setTo(0);
    }
  }

  //(optional) median blur
  if (parameters_ptr_->getParam(MEDIAN_BLUR_SIZE) > 1) {
    cv::medianBlur(preprocessed_image,
                   preprocessed_image,
                   parameters_ptr_->getParam(MEDIAN_BLUR_SIZE));
  }
}

const cv::Rect& Preprocessing::getRegionOfInterest() { return image_limits; }


void Preprocessing::refreshMask() {
  switch (parameters_ptr_->getParam(MASK_MODE)) {
    case MASK_MODE_STANDARD_PATH:
      if (!readMask(parameters_ptr_->getParam(STANDARD_MASK_DIRECTORY),
                    parameters_ptr_->getParam(STANDARD_MASK_NAME))) {
        ROS_WARN(
            "Reading of the standard mask file failed. Check if the file \n "
            "\"%s\"\n exists, is valid and accessible. \n Preprocessing "
            "continues by using trapezoid mask mode. \n You can try to use a "
            "with "
            "dynamic reconfigure modified mask file path instead.",
            (parameters_ptr_->getParam(STANDARD_MASK_DIRECTORY) +
             parameters_ptr_->getParam(STANDARD_MASK_NAME)).c_str());
        use_trapezoid_mask_mode = true;
      } else {
        ROS_INFO(
            "mask mode set to pixelwise mask mode using the mask file at the "
            "standard mask path");
        use_trapezoid_mask_mode = false;
      }
      break;
    case MASK_MODE_MODIFIED_PATH:
      if (!readMask(parameters_ptr_->getParam(MODIFIED_MASK_PATH))) {
        ROS_WARN(
            "Reading of the modified mask file failed. Check if the file \n "
            "\"%s\"\n exists, is valid and accessible. \n Preprocessing "
            "continues by using trapezoid mask mode. \n You can try to use the "
            "standard mask file instead.",
            (parameters_ptr_->getParam(MODIFIED_MASK_PATH)).c_str());
        use_trapezoid_mask_mode = true;
      } else {
        ROS_INFO(
            "mask mode set to pixelwise mask mode using the mask file \n "
            "\"%s\"",
            (parameters_ptr_->getParam(MODIFIED_MASK_PATH)).c_str());
        use_trapezoid_mask_mode = false;
      }
      break;
    case MASK_MODE_TRAPEZOID:
    default:
      ROS_INFO("mask mode set to trapezoid mask mode");
      use_trapezoid_mask_mode = true;
  }
}


inline void Preprocessing::binarizeAdaptiveThreshold(const cv::Mat& image_grey,
                                                     int blocksize,
                                                     double sigma,
                                                     double c,
                                                     cv::Mat& image_binary) {
  double stddev = 0;
  if (sigma != 0) {
    cv::Scalar mean_scalar, stddev_scalar;
    cv::meanStdDev(image_grey, mean_scalar, stddev_scalar);
    stddev = stddev_scalar.val[0];
  }
  cv::adaptiveThreshold(image_grey,
                        image_binary,
                        WHITE_PIXEL,
                        cv::ADAPTIVE_THRESH_MEAN_C,
                        cv::THRESH_BINARY,
                        blocksize,
                        -sigma * stddev - c);
}

inline void Preprocessing::binarizeImageOtsu(const cv::Mat& image_grey, cv::Mat& image_binary) {
  cv::threshold(image_grey, image_binary, 0, WHITE_PIXEL, cv::THRESH_BINARY | cv::THRESH_OTSU);
}

inline void Preprocessing::binarizeImageThresh(const cv::Mat& image_grey,
                                               double sigma,
                                               cv::Mat& image_binary) {
  cv::Scalar mean_scalar, stddev_scalar;
  cv::meanStdDev(image_grey, mean_scalar, stddev_scalar);
  double mean = mean_scalar.val[0];
  double stddev = stddev_scalar.val[0];
  cv::threshold(image_grey, image_binary, mean + sigma * stddev, WHITE_PIXEL, cv::THRESH_BINARY);
}

inline bool Preprocessing::readMask(std::string mask_path) {
  if (!mask_path.empty() && mask_path[0] == '~') {
    if (!getenv("HOME")) {
      ROS_ERROR(
          "unable to resolve '~' character in standard mask directory "
          "specified in the preprocessing parameter YAML file \n because the "
          "HOME environment variable is not set");

      return false;
    }

    mask_path.replace(0, 1, getenv("HOME"));
  }

  mask = cv::imread(mask_path, 0);

  return mask.data && cutMaskToROI();
}

inline bool Preprocessing::readMask(std::string mask_directory, const std::string& mask_name) {
  if (mask_directory.back() != '/') {
    mask_directory = mask_directory + "/";
  }

  return readMask(mask_directory + mask_name);
}

inline void Preprocessing::updateImageLimits() {
  if (image_limits.x != parameters_ptr_->getParam(ROI_START_LOC_X) ||
      image_limits.y != parameters_ptr_->getParam(ROI_START_LOC_Y) ||
      image_limits.width != parameters_ptr_->getParam(ROI_END_LOC_X) - image_limits.x ||
      image_limits.height != parameters_ptr_->getParam(ROI_END_LOC_Y) - image_limits.y) {
    image_limits.x = parameters_ptr_->getParam(ROI_START_LOC_X);
    image_limits.y = parameters_ptr_->getParam(ROI_START_LOC_Y);
    image_limits.width = parameters_ptr_->getParam(ROI_END_LOC_X) - image_limits.x;
    image_limits.height = parameters_ptr_->getParam(ROI_END_LOC_Y) - image_limits.y;

    if (!isUsingTrapezoidMaskMode()) {
      if (!cutMaskToROI()) {
        use_trapezoid_mask_mode = true;
        ROS_ERROR(
            "The ROI and the mask file dimensions are incompatible. \n "
            "Preprocessing "
            "continues by using trapezoid mask mode.");
      }
    }
  }
}


bool Preprocessing::cutMaskToROI() {
  ROS_DEBUG("cut mask to ROI");
  if (mask.rows >= parameters_ptr_->getParam(ROI_END_LOC_Y) &&
      mask.cols >= parameters_ptr_->getParam(ROI_END_LOC_X)) {
    cv::Range col_range_mask(parameters_ptr_->getParam(ROI_START_LOC_X),
                             parameters_ptr_->getParam(ROI_END_LOC_X));
    cv::Range row_range_mask(parameters_ptr_->getParam(ROI_START_LOC_Y),
                             parameters_ptr_->getParam(ROI_END_LOC_Y));
    mask_roi = mask(row_range_mask, col_range_mask);
    return true;
  } else {
    ROS_ERROR(
        "cutMaskToROI was not able to cut the mask from the maskfile to the "
        "ROI, because the ROI end locations are bigger then the mask file "
        "dimensions");
    return false;
  }
}

bool Preprocessing::isUsingTrapezoidMaskMode() {
  return use_trapezoid_mask_mode;
}
