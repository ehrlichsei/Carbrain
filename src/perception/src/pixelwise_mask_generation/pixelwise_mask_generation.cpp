#include "pixelwise_mask_generation.h"
#include <common/macros.h>

THIRD_PARTY_HEADERS_BEGIN
#include <fstream>
#include <opencv2/opencv.hpp>
#include <ros/ros.h>
THIRD_PARTY_HEADERS_END

#include "common/assert_fail.h"

#include "pixelwise_mask_generation.h"

const ParameterString<std::string> PixelwiseMaskGeneration::STANDARD_MASK_DIRECTORY(
    "/perception/ego_vehicle/pixelwise_mask/standard_mask_directory");
const ParameterString<std::string> PixelwiseMaskGeneration::STANDARD_MASK_NAME(
    "/perception/ego_vehicle/pixelwise_mask/standard_mask_name");
const ParameterString<int> PixelwiseMaskGeneration::LOWER_DIFF(
    "segmentation/max_color_diff/lower_diff");
const ParameterString<int> PixelwiseMaskGeneration::UPPER_DIFF(
    "segmentation/max_color_diff/upper_diff");
const ParameterString<int> PixelwiseMaskGeneration::MIN_SEGMENT_SIZE(
    "segmentation/min_segment_size");
const ParameterString<int> PixelwiseMaskGeneration::NUMBER_OF_SEGMENTATION_ATTEMPTS(
    "segmentation/number_of_segmentation_attempts");
const ParameterString<int> PixelwiseMaskGeneration::STARTING_POINT_MAX_Y(
    "segmentation/starting_point/random/max_y");
const ParameterString<std::string> PixelwiseMaskGeneration::OLD_FILE_NAME(
    "old_mask_names/old_file_name");
const ParameterString<std::string> PixelwiseMaskGeneration::OLD_MASKS_FOLDER_NAME(
    "old_mask_names/old_masks_folder_name");
const ParameterString<int> PixelwiseMaskGeneration::ARCHIVE_WARNING_SIZE(
    "max_archive_directory_size/warning_size");
const ParameterString<int> PixelwiseMaskGeneration::ARCHIVE_ERROR_SIZE(
    "max_archive_directory_size/error_size");
const ParameterString<int> PixelwiseMaskGeneration::AUTOMATIC_MASK_GEN_NUMBER_OF_PICTURES(
    "automatic_mask_gen/number_of_pictures");
const ParameterString<int> PixelwiseMaskGeneration::AUTOMATIC_MASK_GEN_BILATERAL_FILTER_SIZE(
    "automatic_mask_gen/bilateral_filter_size");
const ParameterString<double> PixelwiseMaskGeneration::AUTOMATIC_MASK_GEN_SEGMENTATION_TARGET_SIZE(
    "automatic_mask_gen/segmentation_target_size");
const ParameterString<int> PixelwiseMaskGeneration::AUTOMATIC_MASK_GEN_BILATERAL_FILTER_MAX_SIGMA(
    "automatic_mask_gen/bilateral_filter_max_sigma");
const ParameterString<double> PixelwiseMaskGeneration::AUTOMATIC_MASK_GEN_MEDIAN_BLUR_SEGMENTATION_ARTIFACT_SIZE_THRESHOLD(
    "automatic_mask_gen/median_blur_segmentation_artifact_size_threshold");
const ParameterString<int> PixelwiseMaskGeneration::AUTOMATIC_MASK_GEN_MEDIAN_BLUR_MAX_KERNEL_SIZE(
    "automatic_mask_gen/median_blur_max_kernel_size");
const ParameterString<int> PixelwiseMaskGeneration::AUTOMATIC_MASK_GEN_CLOSING_MAX_NUMBER_OF_SEGMENTS(
    "automatic_mask_gen/closing_max_number_of_segments");
const ParameterString<int> PixelwiseMaskGeneration::AUTOMATIC_MASK_GEN_CLOSING_MAX_SIZE(
    "automatic_mask_gen/closing_max_size");
const ParameterString<int> PixelwiseMaskGeneration::AUTOMATIC_MASK_GEN_DILATION_SIZE(
    "automatic_mask_gen/dilation_size");
const ParameterString<bool> PixelwiseMaskGeneration::AUTOMATIC_MASK_GEN_AUTO_SAVE(
    "automatic_mask_gen/auto_save");
const ParameterString<bool> PixelwiseMaskGeneration::ASSUME_CONNECTED_MASK(
    "assume_connected_mask");


PixelwiseMaskGeneration::PixelwiseMaskGeneration(ParameterInterface *parameterI)
    : parameters_ptr_(parameterI) {
  picture_counter = 0;
  take_pictures = false;
  save_mask = false;
  apply_median_blur = false;
  dilate = false;
  apply_bilateral_filter = false;
  apply_closing = false;
  number_of_pictures = 0;
  dilation_size = 0;
  closing_size = 0;
  filter_size_bilateral_filter = 0;
  sigma_bilateral_filter = 0;
  reset_vars = true;
  automatic_mask_gen_stage = 0;
  automatic_mask_gen = false;
  draw_starting_point = false;
  starting_point_x = 0;
  starting_point_y = 0;
  random_starting_point = true;


  parameterI->registerParam(STANDARD_MASK_DIRECTORY);
  parameterI->registerParam(STANDARD_MASK_NAME);
  parameterI->registerParam(LOWER_DIFF);
  parameterI->registerParam(UPPER_DIFF);
  parameterI->registerParam(MIN_SEGMENT_SIZE);
  parameterI->registerParam(NUMBER_OF_SEGMENTATION_ATTEMPTS);
  parameterI->registerParam(STARTING_POINT_MAX_Y);
  parameterI->registerParam(OLD_FILE_NAME);
  parameterI->registerParam(OLD_MASKS_FOLDER_NAME);
  parameterI->registerParam(ARCHIVE_WARNING_SIZE);
  parameterI->registerParam(ARCHIVE_ERROR_SIZE);
  parameterI->registerParam(AUTOMATIC_MASK_GEN_NUMBER_OF_PICTURES);
  parameterI->registerParam(AUTOMATIC_MASK_GEN_BILATERAL_FILTER_SIZE);
  parameterI->registerParam(AUTOMATIC_MASK_GEN_SEGMENTATION_TARGET_SIZE);
  parameterI->registerParam(AUTOMATIC_MASK_GEN_BILATERAL_FILTER_MAX_SIGMA);
  parameterI->registerParam(AUTOMATIC_MASK_GEN_MEDIAN_BLUR_SEGMENTATION_ARTIFACT_SIZE_THRESHOLD);
  parameterI->registerParam(AUTOMATIC_MASK_GEN_MEDIAN_BLUR_MAX_KERNEL_SIZE);
  parameterI->registerParam(AUTOMATIC_MASK_GEN_CLOSING_MAX_NUMBER_OF_SEGMENTS);
  parameterI->registerParam(AUTOMATIC_MASK_GEN_CLOSING_MAX_SIZE);
  parameterI->registerParam(AUTOMATIC_MASK_GEN_DILATION_SIZE);
  parameterI->registerParam(AUTOMATIC_MASK_GEN_AUTO_SAVE);
  parameterI->registerParam(ASSUME_CONNECTED_MASK);
}


bool PixelwiseMaskGeneration::generateMask(const cv::Mat &image, cv::Mat &mask_image) {  // generateMask is executed for every incoming image
  // by handleImage in pixelwise_mask_generation_node

  if (reset_vars) {  // resets variables if in the last cycle the mask was saved
    reset_vars = false;
    resetVars(image);
  }


  // if one of the booleans is set by a service callback function (to be exact
  // by the function of the same name in pixelwise_mask_generation called by the
  // service callback function in the node) the corrosponding action is executed
  // (e.g applying an median filter on the mask, saving the mask etc.)
  if (take_pictures) {  // take numberOfPictures pictures and generate for each
                        // one a segmentation and generate a mask by applying a
                        // threshold on the sum of the segmentations
    cv::Mat bilateral_filtered_image;
    if (picture_counter < number_of_pictures) {
      if (apply_bilateral_filter) {  // apply bilateral filter
        if (image.channels() == 3) {
          bilateral_filtered_image = cv::Mat(
              image.size().height, image.size().width, CV_8UC3, cv::Scalar(0));  // because image.data must not be the same as bilateral_filtered_image.data for bilateralFilter
        } else {
          bilateral_filtered_image = cv::Mat(
              image.size().height, image.size().width, CV_8UC1, cv::Scalar(0));  // because image.data must not be the same as bilateral_filtered_image.data for bilateralFilter
        }
        cv::bilateralFilter(image,
                            bilateral_filtered_image,
                            filter_size_bilateral_filter,
                            sigma_bilateral_filter,
                            sigma_bilateral_filter);

      } else {  // or don't apply bilateral filter
        bilateral_filtered_image = image;
      }
      cv::Mat processed_image =
          processImage(bilateral_filtered_image);  // generate the mask
      if (cv::countNonZero(processed_image) >
          0) {  // if mask generation sucessfull add mask to other masks
        picture_counter++;
        component_image += processed_image;
      } else {  // if mask generation not successfull abort component mask
                // generation
        ROS_WARN("Received empty segmentation. Image too noisy?");
        resetVars(image);
      }
      return false;
    } else {

      segmented_image = cv::Mat::zeros(image.size().height, image.size().width, CV_8UC1);
      segmented_image = component_image >=
                        (number_of_pictures / 2.0);  // sets all values either
                                                     // to 255 or 0 based on how
                                                     // often these areas were
                                                     // segmented as car or not
                                                     // car (threshold ca. 50%)
      segmented_image = segmented_image / 255;
      mask_image =
          255 *
          segmented_image;  // because in the rqt image viewer white is 255
      picture_counter = 0;

      if (apply_bilateral_filter) {
        ROS_INFO(
            "Built mask out of the segmentation of %i bilateral filtered "
            "images",
            number_of_pictures);
      } else {
        ROS_INFO(
            "Built mask out of the segmentation of %i images (no bilateral "
            "filtering)",
            number_of_pictures);
      }

      take_pictures = false;
      apply_bilateral_filter = false;
      component_image = cv::Mat::zeros(image.size().height, image.size().width, CV_8UC1);
      return true;
    }

  } else if (apply_median_blur) {  // apply medianBlur
    apply_median_blur = false;
    cv::medianBlur(segmented_image, segmented_image, kernel_size);
    mask_image =
        255 * segmented_image;  // because in the rqt image viewer white is 255
    ROS_INFO("Applied median blur with kernel size %d on the mask", kernel_size);
    return true;
  } else if (apply_closing) {  // erodes the not-mask-area => mask gets bigger
    apply_closing = false;
    cv::morphologyEx(segmented_image,
                     segmented_image,
                     cv::MORPH_OPEN,
                     cv::getStructuringElement(
                         cv::MORPH_ELLIPSE,
                         cv::Size(2 * closing_size + 1, 2 * closing_size + 1),  // because closing only works with odd erosion_sizes (geometry of filter mask)
                         cv::Point(closing_size, closing_size)));
    mask_image =
        255 * segmented_image;  // because in the rqt image viewer white is 255
    ROS_INFO("Applied Closing Morphology Operation on mask");
    return true;
  }

  else if (dilate) {
    dilate = false;
    // erodes the not-mask-area => mask gets bigger
    cv::erode(segmented_image,
              segmented_image,
              cv::getStructuringElement(cv::MORPH_RECT,
                                        cv::Size(2 * dilation_size + 1, 2 * dilation_size + 1),  // because erode only works with odd erosion_sizes (geometry of filter mask)
                                        cv::Point(dilation_size, dilation_size)));
    mask_image =
        255 * segmented_image;  // because in the rqt image viewer white is 255
    ROS_INFO("Dilated mask by %i pixels", dilation_size);
    return true;
  } else if (save_mask) {  // save Mask
    save_mask = false;
    // if parameter assume_connected_mask is set to true only use the mask
    // element that encloses the point at (width/2, height)
    // if no such mask-element is found, the hole mask is used
    if (parameters_ptr_->getParam(ASSUME_CONNECTED_MASK)) {
      std::vector<std::vector<cv::Point> > contours;
      cv::Mat contour_mat;
      cv::bitwise_not(255 * segmented_image, contour_mat);
      findContours(contour_mat, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);
      for (std::size_t i = 0; i < contours.size(); i++) {
        if (cv::pointPolygonTest(contours[i],
                                 cv::Point(segmented_image.size().width / 2,
                                           segmented_image.size().height - 8),
                                 false) > 0) {
          segmented_image =
              cv::Mat::ones(segmented_image.size(), segmented_image.type());
          cv::drawContours(segmented_image, contours, int(i), cv::Scalar(0, 0, 0), CV_FILLED);
          break;
        }
      }
    }
    cv::Mat test_mat =
        segmented_image(cv::Range(0, image.size().height / 2), cv::Range::all());  // half the image size
    int n_mask_pixels_upper_half = test_mat.size().area() - cv::countNonZero(test_mat);
    ROS_INFO(
        "mask quality evaluation: mask pixels in the upper half of the "
        "image: %i",
        n_mask_pixels_upper_half);
    const double percentage_mask_pixels = std::round(
        (image.size().area() - static_cast<double>(cv::countNonZero(segmented_image))) /
        (image.size().area()) * 100.0);
    ROS_INFO(
        "mask quality evaluation: percentage of the image area covered by "
        "the mask (not considering ROI): %i%%",
        static_cast<int>(percentage_mask_pixels));

    if (!saveMask(segmented_image)) {
      ROS_ERROR("Saving of mask unsuccessfull.");
    } else {
      ROS_INFO("Mask saved");
    }
    mask_image =
        255 * segmented_image;  // because in the rqt image viewer white is 255
    reset_vars = true;  // to reset the variables when the next image arrives
    return true;        // true => publish image
  } else if (automatic_mask_gen) {
    cv::Mat contour_image;
    std::vector<std::vector<cv::Point> > contours;

    switch (automatic_mask_gen_stage) {
      case 0: {
        take_pictures = true;
        number_of_pictures =
            parameters_ptr_->getParam(AUTOMATIC_MASK_GEN_NUMBER_OF_PICTURES);
        sigma_bilateral_filter = 0;
        filter_size_bilateral_filter =
            parameters_ptr_->getParam(AUTOMATIC_MASK_GEN_BILATERAL_FILTER_SIZE);
        closing_size = 1;
        automatic_mask_gen_stage = 1;
      }

      break;
      case 1: {
        contour_image =
            segmented_image
                .clone();  // because findContours changes the input image
        findContours(contour_image, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);
        // apply bilateral filter until the segmentation area is large enough
        if (contours.empty() ||
            (cv::contourArea(contours[0]) < parameters_ptr_->getParam(AUTOMATIC_MASK_GEN_SEGMENTATION_TARGET_SIZE) *
                                                segmented_image.size().area() &&
             sigma_bilateral_filter <
                 parameters_ptr_->getParam(AUTOMATIC_MASK_GEN_BILATERAL_FILTER_MAX_SIGMA))) {
          sigma_bilateral_filter++;
          apply_bilateral_filter = true;
          take_pictures = true;
        } else {
          automatic_mask_gen_stage = 2;
        }
      } break;
      case 2: {
        cv::Mat inverted_image;
        cv::bitwise_not(255 * segmented_image, inverted_image);
        cv::Mat test_mat = inverted_image(cv::Range::all(), cv::Range::all());
        findContours(test_mat, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);
        int max_size = 0;
        // find size of the largest segmentation artifact
        if (contours.size() > 1) {
          for (const auto &contour : contours) {
            const int size = std::max(cv::boundingRect(contour).width,
                                      cv::boundingRect(contour).height);
            if (size < segmented_image.size().width *
                           parameters_ptr_->getParam(AUTOMATIC_MASK_GEN_MEDIAN_BLUR_SEGMENTATION_ARTIFACT_SIZE_THRESHOLD) &&
                size > max_size) {
              max_size = size;
            }
          }
          if (max_size > 1) {
            kernel_size = max_size * 2 + 1;
            if (kernel_size > parameters_ptr_->getParam(AUTOMATIC_MASK_GEN_MEDIAN_BLUR_MAX_KERNEL_SIZE)) {
              kernel_size = parameters_ptr_->getParam(AUTOMATIC_MASK_GEN_MEDIAN_BLUR_MAX_KERNEL_SIZE);
            }
            apply_median_blur = true;
          }
        }
        automatic_mask_gen_stage = 3;
      } break;
      case 3: {
        cv::Mat inverted_image;
        cv::bitwise_not(255 * segmented_image, inverted_image);
        cv::Mat test_mat = inverted_image(
            cv::Range(segmented_image.size().height / 2, segmented_image.size().height),
            cv::Range::all());
        findContours(test_mat, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);
        if (contours.size() > std::size_t(parameters_ptr_->getParam(
                                  AUTOMATIC_MASK_GEN_CLOSING_MAX_NUMBER_OF_SEGMENTS)) &&
            closing_size < 88) {  // clustering instead of number of segments?
                                  // //!TODO remove magic number!
          closing_size += 2;
          apply_closing = true;
        } else {
          automatic_mask_gen_stage = 4;
        }
        break;
      }
      case 4: {
        dilation_size = parameters_ptr_->getParam(AUTOMATIC_MASK_GEN_DILATION_SIZE);
        dilate = true;
        automatic_mask_gen_stage = 5;
      } break;
      case 5: {
        save_mask = parameters_ptr_->getParam(AUTOMATIC_MASK_GEN_AUTO_SAVE);
        automatic_mask_gen = false;
        automatic_mask_gen_stage = 0;
        break;
      }
      default: { assert_fail("invalid automatic_mask_gen_stage"); }
    }
    mask_image =
        255 * segmented_image;  // because in the rqt image viewer white is 255
    return true;
  } else if (draw_starting_point) {
    draw_starting_point = false;
    mask_image = 255 * segmented_image;
    return true;
  }

  return false;
}

cv::Mat PixelwiseMaskGeneration::processImage(const cv::Mat &image) {

  int ix;
  int iy;
  cv::Mat pixelwise_mask =
      cv::Mat::zeros(image.size().height + 2, image.size().width + 2, CV_8UC1);
  if (random_starting_point) {
    int count = 0;
    ix = 0;
    iy = 0;

    cv::Rect bound_rect = cv::Rect(
        0, 0, 2, 2);  // bound_rect is the bounding rectangle of the segmented
    // area - segmentation is discarded if bound_rect (and thus
    // the segmented area) is  too small
    do {
      cv::floodFill(image,
                    pixelwise_mask,
                    cv::Point(ix, iy),
                    cv::Scalar(0, 0, 0),
                    &bound_rect,
                    cv::Scalar(parameters_ptr_->getParam(LOWER_DIFF),
                               parameters_ptr_->getParam(LOWER_DIFF),
                               parameters_ptr_->getParam(LOWER_DIFF)),
                    cv::Scalar(parameters_ptr_->getParam(UPPER_DIFF),
                               parameters_ptr_->getParam(UPPER_DIFF),
                               parameters_ptr_->getParam(UPPER_DIFF)),
                    4 | cv::FLOODFILL_MASK_ONLY | (1 << 8));
      ix = rand() %
           image.cols;  // determine starting position of segmentation randomly
      iy = rand() % parameters_ptr_->getParam(STARTING_POINT_MAX_Y);
      if (count >= parameters_ptr_->getParam(NUMBER_OF_SEGMENTATION_ATTEMPTS)) {  // if the segmentation fails multiple times in a row the image is probably too noisy
        ROS_WARN(
            "not able to generate mask because all segments found are too "
            "small");
        break;
      }
      count++;
    } while (bound_rect.area() < parameters_ptr_->getParam(MIN_SEGMENT_SIZE));
  } else {
    ix = starting_point_x;
    iy = starting_point_y;
    cv::floodFill(image,
                  pixelwise_mask,
                  cv::Point(ix, iy),
                  cv::Scalar(0, 0, 0),
                  std::nullptr_t(),
                  cv::Scalar(parameters_ptr_->getParam(LOWER_DIFF),
                             parameters_ptr_->getParam(LOWER_DIFF),
                             parameters_ptr_->getParam(LOWER_DIFF)),
                  cv::Scalar(parameters_ptr_->getParam(UPPER_DIFF),
                             parameters_ptr_->getParam(UPPER_DIFF),
                             parameters_ptr_->getParam(UPPER_DIFF)),
                  4 | cv::FLOODFILL_MASK_ONLY | (1 << 8));
  }
  pixelwise_mask = pixelwise_mask(
      cv::Range(1, image.size().height + 1), cv::Range(1, image.size().width + 1));  // rescaling pixelwiseMask to image
  // size, because the mask has to be
  // bigger than the image for the
  // floodFill
  return pixelwise_mask;
}



void PixelwiseMaskGeneration::generate_image_mask_contour(const cv::Mat &image,
                                                          const cv::Mat &mask,
                                                          cv::Mat &image_mask_contour,
                                                          const cv::Scalar& colour) {
  std::vector<std::vector<cv::Point> > display_contours;
  std::vector<cv::Vec4i> hierarchy;
  cv::Mat inverted_mask;
  cv::bitwise_not(255 * mask, inverted_mask);

  cv::findContours(inverted_mask, display_contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);


  if (image.channels() == 1) {
    cv::cvtColor(image, image_mask_contour, CV_GRAY2BGR);
  } else {
    image_mask_contour = image.clone();
  }

  if (!(random_starting_point)) {
    cv::line(image_mask_contour,
             cv::Point(starting_point_x - 20, starting_point_y),
             cv::Point(starting_point_x + 20, starting_point_y),
             cv::Scalar(255, 0, 0));
    cv::line(image_mask_contour,
             cv::Point(starting_point_x, starting_point_y - 20),
             cv::Point(starting_point_x, starting_point_y + 20),
             cv::Scalar(255, 0, 0));
  }
  cv::drawContours(image_mask_contour, display_contours, -1, colour, 2);
}


// these functions set the bools in generateMask and are called by the callbacks
// of the services

void PixelwiseMaskGeneration::setTakePictures(int nop) {
  take_pictures = true;
  number_of_pictures = nop;
}

void PixelwiseMaskGeneration::setSaveMask() { save_mask = true; }

void PixelwiseMaskGeneration::setAutomaticMaskGen() {
  automatic_mask_gen = true;
}

void PixelwiseMaskGeneration::setMedianBlur(int ks) {
  kernel_size = ks;
  apply_median_blur = true;
}

void PixelwiseMaskGeneration::setDilation(int es) {
  dilation_size = es;
  dilate = true;
}

void PixelwiseMaskGeneration::setClosing(int cs) {
  closing_size = cs;
  apply_closing = true;
}

void PixelwiseMaskGeneration::setBilateralFilter(int size, int sigma) {
  filter_size_bilateral_filter = size;
  sigma_bilateral_filter = sigma;
  apply_bilateral_filter = true;
}

void PixelwiseMaskGeneration::setStartingPoint(int x, int y) {

  if (x >= 0 && y >= 0) {
    starting_point_x = x;
    starting_point_y = y;
    draw_starting_point = true;
    random_starting_point = false;
  } else {
    random_starting_point = true;
  }
}


bool PixelwiseMaskGeneration::saveMask(const cv::Mat &mask) {
  std::string old_mask_file_name = parameters_ptr_->getParam(OLD_FILE_NAME);
  std::string old_masks_directory_name =
      parameters_ptr_->getParam(OLD_MASKS_FOLDER_NAME) + "/";
  std::string mask_file_extension;
  std::string current_mask_path_string;
  fs::path current_mask_path;
  fs::path current_mask_directory_path;
  fs::path old_mask_directory_path;
  try {
    if ((parameters_ptr_->getParam(
            STANDARD_MASK_DIRECTORY))[(parameters_ptr_->getParam(STANDARD_MASK_DIRECTORY)).length() - 1] !=
        '/') {
      current_mask_path_string = parameters_ptr_->getParam(STANDARD_MASK_DIRECTORY) +
                                 "/" + parameters_ptr_->getParam(STANDARD_MASK_NAME);  // convert the standardMaskPath parameter to a boost path
    } else {
      current_mask_path_string = parameters_ptr_->getParam(STANDARD_MASK_DIRECTORY) +
                                 parameters_ptr_->getParam(STANDARD_MASK_NAME);  // convert the standardMaskPath parameter to a boost path
    }
    if (!current_mask_path_string.empty() && current_mask_path_string[0] == '~') {

      if (getenv("HOME")) {
        current_mask_path_string.replace(0, 1, getenv("HOME"));
      } else {
        ROS_ERROR(
            "unable to resolve '~' character in standard mask directory "
            "specified in the preprocessing parameter YAML file \n because the "
            "HOME environment variable is not set");
        return false;
      }
    }

    current_mask_path = fs::path(current_mask_path_string);
    mask_file_extension = fs::extension(current_mask_path);
    current_mask_directory_path = current_mask_path.parent_path();
    current_mask_directory_path += "/";
    old_mask_directory_path = current_mask_directory_path;
    old_mask_directory_path += old_masks_directory_name;


    if (current_mask_directory_path.empty()) {
      ROS_ERROR(
          "The current mask path defined in the preprocessing parameter YAML "
          "file is empty");
      return false;
    }

    if (current_mask_directory_path.is_relative()) {
      ROS_ERROR(
          "The current mask path defined in the preprocessing parameter YAML "
          "file has to be an absolute path");
      return false;
    }

    if (!fs::exists(current_mask_directory_path)) {
      if (fs::create_directory(current_mask_directory_path)) {
        ROS_INFO(
            "created the standard mask directory as specified in the "
            "preprocessing YAML parameter file \n path: %s",
            current_mask_directory_path.string().c_str());
      } else {
        ROS_ERROR(
            "mask generation not able to generate mask because the "
            "directory \n %s \n does not exist and the mask generation was not "
            "able to generate this directory",
            current_mask_directory_path.string().c_str());
        return false;
      }
    }


    if (!fs::exists(old_mask_directory_path)) {
      if (!fs::create_directory(old_mask_directory_path)) {
        ROS_ERROR(
            "mask generation was not able to create the subdirectory for "
            "the old mask files %s",
            old_mask_directory_path.string().c_str());
        return false;
      }
    }

    if (!(mask_file_extension == ".pbm" || mask_file_extension == ".bmp")) {
      ROS_ERROR(
          "mask generation not able to generate mask because the file "
          "extension %s for the mask name is not supported \n"
          "supported file extensions: .pbm and .bmp",
          mask_file_extension.c_str());
      return false;
    }

    uintmax_t size_directory = 0;
    int file_index = 0;
    std::string temp_file_name;
    fs::directory_iterator di(old_mask_directory_path);
    for (; di != fs::directory_iterator();
         di++) {  // search directory for the highest used file index
      temp_file_name = (*di).path().filename().string();
      if (!temp_file_name.compare(0, old_mask_file_name.length(), old_mask_file_name)) {  // if file in the directory begins with the template of the old file name check and save its index
        if (fs::exists(*di) && fs::is_regular_file(*di)) {
          size_directory += fs::file_size(*di);
        }
        int temp_file_index = 0;
        std::stringstream convert1(temp_file_name.substr(old_mask_file_name.length()));
        convert1 >> temp_file_index;
        if (temp_file_index > file_index) {
          file_index = temp_file_index;
        }
      }
    }
    file_index += 1;

    // checking, if the pixelwise mask archive directory gets too big
    if (size_directory >
        uintmax_t(convertMegabyteToByte(parameters_ptr_->getParam(ARCHIVE_ERROR_SIZE)))) {
      ROS_ERROR(
          "The pixelwise mask archive directory %s reached the error size of "
          "%i mB",
          old_mask_directory_path.string().c_str(),
          parameters_ptr_->getParam(ARCHIVE_ERROR_SIZE));
      ROS_ERROR("Please delete mask files that are outdated");
      ROS_ERROR(
          "You will not be able to save any new masks as long as the mask "
          "archive directory is bigger than %i mB",
          parameters_ptr_->getParam(ARCHIVE_ERROR_SIZE));
      ROS_INFO(
          "the warning size and the error size can be changed in the pixelwise "
          "mask generation parameter files");
      return false;
    } else if (size_directory > uintmax_t(convertMegabyteToByte(
                                    parameters_ptr_->getParam(ARCHIVE_WARNING_SIZE)))) {
      ROS_WARN(
          "The pixelwise mask archive directory %s reached the warning size of "
          "%i mB",
          old_mask_directory_path.string().c_str(),
          parameters_ptr_->getParam(ARCHIVE_WARNING_SIZE));
      ROS_WARN("Please delete mask files that are outdated");
      ROS_WARN(
          "If this folder reaches the error size of %i mB no new mask files "
          "can be created",
          parameters_ptr_->getParam(ARCHIVE_ERROR_SIZE));
      ROS_INFO(
          "the warning size and the error size can be changed in the pixelwise "
          "mask generation parameter files");
    }

    // save old mask file in the archive with correct index
    if (fs::exists(current_mask_path) && fs::is_regular_file(current_mask_path)) {
      std::stringstream convert2;
      convert2 << file_index;
      std::string file_index_str = convert2.str();
      fs::path old_mask_path = old_mask_directory_path;
      old_mask_path += old_mask_file_name + file_index_str + mask_file_extension;
      fs::copy_file(current_mask_path, old_mask_path);
    }
  } catch (const fs::filesystem_error &ex) {
    ROS_ERROR("%s", ex.what());
    return false;
  }

  try {
    if (mask_file_extension == ".pbm") {  // save as .pbm

      std::ofstream maskFileStream;
      maskFileStream.open(current_mask_path.string().c_str(),
                          std::ios::trunc | std::ios::binary);
      if (maskFileStream.is_open()) {
        maskFileStream << "P4"
                       << "\n" << mask.cols << " " << mask.rows << "\n";
        uint32_t data_size =
            (mask.rows * mask.cols) / 8 + (((mask.rows * mask.cols) % 8) ? 0 : 1);
        std::vector<char> data(data_size, 0);
        for (int y = 0; y < mask.rows; y++) {
          for (int x = 0; x < mask.cols; x++) {
            data[(x + y * mask.cols) / 8] |= mask.at<uchar>(y, x)
                                             << (7 - (x + y * mask.cols) % 8);
          }
        }

        maskFileStream.write(data.data(), data.size());
        maskFileStream.close();
        return true;
      } else {
        throw std::runtime_error("not able to open mask file");
      }
    } else if (mask_file_extension == ".bmp") {  // save as .bmp
      cv::bitwise_not(mask, mask);
      mask -= 254;
      cv::imwrite(current_mask_path.string(), mask);
      return true;
    }

  } catch (std::runtime_error &ex) {
    ROS_ERROR(
        "mask generation unable to write the new mask file to the "
        "harddrive \n %s",
        ex.what());
    return false;
  }
  return false;
}

int PixelwiseMaskGeneration::convertMegabyteToByte(int Megabyte) {
  return Megabyte * 1024 * 1024;
}

void PixelwiseMaskGeneration::resetVars(cv::Mat image) {
  picture_counter = 0;
  take_pictures = false;
  save_mask = false;
  apply_median_blur = false;
  dilate = false;
  apply_bilateral_filter = false;
  apply_closing = false;
  number_of_pictures = 0;
  dilation_size = 0;
  closing_size = 0;
  filter_size_bilateral_filter = 0;
  sigma_bilateral_filter = 0;
  automatic_mask_gen_stage = 0;
  draw_starting_point = false;
  starting_point_x = 0;
  starting_point_y = 0;

  segmented_image = cv::Mat::zeros(image.size().height, image.size().width, CV_8UC1);
  component_image = cv::Mat::zeros(image.size().height, image.size().width, CV_8UC1);
}
