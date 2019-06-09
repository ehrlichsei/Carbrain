#ifndef PIXELWISE_MASK_GENERATION_H
#define PIXELWISE_MASK_GENERATION_H
#include <common/macros.h>
#define BOOST_NO_CXX11_SCOPED_ENUMS

THIRD_PARTY_HEADERS_BEGIN
#include <boost/filesystem.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
THIRD_PARTY_HEADERS_END

#include "common/parameter_interface.h"



namespace fs = boost::filesystem;

class PixelwiseMaskGeneration {

 public:
  /*!
   * \brief PixelwiseMaskGeneration
   *
   * The constructor you should usually use.
   *
   * \param parameterI The ParameterInterface for the preprocessing parameters
   */
  PixelwiseMaskGeneration(ParameterInterface* parameterI);

  /*!
   * \brief generateMask
   *
   * generates pixelwise mask
   *
   * \param image the source image
   * \param mask_image the generated mask
   */
  bool generateMask(const cv::Mat &image, cv::Mat &mask_image);


  /*!
   * \brief setTakePictures
   *
   * takes multiple pictures for the mask generation
   *
   */
  void setTakePictures(int nop);

  /*!
   * \brief setSaveMask
   *bool
   * calls function saveMask to save the mask
   *
   */
  void setSaveMask();

  /*!
   * \brief setAutomaticMaskGen
   *
   * generates mask automatically
   *
   */
  void setAutomaticMaskGen();

  /*!
   * \brief setMedianBlur
   *
   * takes median blur
   *
   */
  void setMedianBlur(int ks);

  /*!
   * \brief setDilation
   *
   * dilates the mask => mask gets bigger
   *
   */
  void setDilation(int es);

  /*!
   * \brief setClosing
   *
   * closes mask
   *
   */
  void setClosing(int cs);

  /*!
   * \brief setBilateralFilter
   *
   * bilateral filter
   *
   */
  void setBilateralFilter(int size, int sigma);

  /*!
   * \brief setSetStartingPoint
   *
   * set the starting point of the segmentation
   *
   */
  void setStartingPoint(int x, int y);

  /*!
   * \brief generateMask
   *
   * paints the contour of the mask in the raw image
   *
   * \param image the source image
   * \param image_mask_contour the contour of the mask painted in the raw image
   */
  void generate_image_mask_contour(const cv::Mat &image, const cv::Mat &mask, cv::Mat &image_mask_contour, const cv::Scalar& colour);

  /*!
   * \brief STANDARD_MASK_DIRECTORY
   *
   * directory of the standard mask file for the pixelwise mask in standard mask mode (folder location without file name; '~' for HOME directory)
   */
  static const ParameterString<std::string> STANDARD_MASK_DIRECTORY;
  /*!
   * \brief STANDARD_MASK_NAME
   *
   * file name for the standard mask file for the pixelwise mask in standard mask mode
   */
  static const ParameterString<std::string> STANDARD_MASK_NAME;
  static const ParameterString<int> LOWER_DIFF;
  static const ParameterString<int> UPPER_DIFF;
  static const ParameterString<int> MIN_SEGMENT_SIZE;
  static const ParameterString<int> NUMBER_OF_SEGMENTATION_ATTEMPTS;
  static const ParameterString<int> STARTING_POINT_MAX_Y;
  static const ParameterString<std::string> OLD_FILE_NAME;
  static const ParameterString<std::string> OLD_MASKS_FOLDER_NAME;
  static const ParameterString<int> ARCHIVE_WARNING_SIZE;
  static const ParameterString<int> ARCHIVE_ERROR_SIZE;
  static const ParameterString<int> AUTOMATIC_MASK_GEN_NUMBER_OF_PICTURES;
  static const ParameterString<int> AUTOMATIC_MASK_GEN_BILATERAL_FILTER_SIZE;
  static const ParameterString<double> AUTOMATIC_MASK_GEN_SEGMENTATION_TARGET_SIZE;
  static const ParameterString<int> AUTOMATIC_MASK_GEN_BILATERAL_FILTER_MAX_SIGMA;
  static const ParameterString<double> AUTOMATIC_MASK_GEN_MEDIAN_BLUR_SEGMENTATION_ARTIFACT_SIZE_THRESHOLD;
  static const ParameterString<int> AUTOMATIC_MASK_GEN_CLOSING_MAX_NUMBER_OF_SEGMENTS;
  static const ParameterString<int> AUTOMATIC_MASK_GEN_CLOSING_MAX_SIZE;
  static const ParameterString<int> AUTOMATIC_MASK_GEN_DILATION_SIZE;
  static const ParameterString<bool> AUTOMATIC_MASK_GEN_AUTO_SAVE;
  static const ParameterString<int> AUTOMATIC_MASK_GEN_MEDIAN_BLUR_MAX_KERNEL_SIZE;
  static const ParameterString<bool> ASSUME_CONNECTED_MASK;


private:
  /*!
   * \brief processImage
   *
   * generate mask for single image
   *
   * \param image the source image
   */
  cv::Mat processImage(const cv::Mat &image);

  /*!
   * \brief saveMask
   *
   * saves pixelwise mask
   *
   * \param mask the mask to save
   */
  bool saveMask(const cv::Mat &mask);

  /*!
   * \brief resetVars
   *
   * resets the Variables of the pixelwise mask generation
   *
   */
  inline void resetVars(cv::Mat image);

  /*!
   * \brief params number of Megabytes
   * converts Megabytes to Bytes
   */
  inline int convertMegabyteToByte(int Megabyte);

  /*!
   * \brief counter of images for mask generation with multiple pictures
   */
  int picture_counter;

  /*!
   * \brief take Pictures if true
   */
  bool take_pictures;

  /*!
   * \brief save Mask if true
   */
  bool save_mask;

  /*!
   * \brief apply median blur if true
   */
  bool apply_median_blur;

  /*!
   * \brief erode if true
   */
  bool dilate;

  /*!
   * \brief apply bilateral filter if true
   */
  bool apply_bilateral_filter;

  /*!
   * \brief apply closing filter if true
   */
  bool apply_closing;

  /*!
   * \brief use automatic mask generation if true
   */
  bool automatic_mask_gen;

  /*!
   * \brief number of pictures
   */
  int number_of_pictures;

  /*!
   * \brief size of dilation
   */
  int dilation_size;

  /*!
   * \brief size of closing
   */
  int closing_size;

  /*!
   * \brief kernel size for the median blur
   */
  int kernel_size = 1;

  /*!
   * \brief sigma for the bilateral filter
   */
  int sigma_bilateral_filter;

  /*!
   * \brief filter size for the bilateral filter
   */
  int filter_size_bilateral_filter;

  /*!
   * \brief current stage of the automatic mask generation
   */
  int automatic_mask_gen_stage;


  /*!
   * \brief x coordinate of the starting point of the segmentation
   */

  int starting_point_x;

  /*!
   * \brief y coordinate of the starting point of the segmentation
   */
  int starting_point_y;

  /*!
   * \brief determines whether the segmentation starting point is drawn
   */
  bool draw_starting_point;

  /*!
   * \brief determines whether the segmentation starting point is chosen randomly
   */
  bool random_starting_point;

  /*!
   * \brief determines if mask generation is executed the first time since the
   * start of the node or Variables need to be reset after the saving of a mask
   */
  bool reset_vars;

  /*!
   * \brief Image constructed out of multiple camera images to generate the mask
   */
  cv::Mat segmented_image;

  /*!
   * \brief image composed of multiple camera images
   */
  cv::Mat component_image;

  /*!
  * \brief parameters_ptr_ is needed for parameter access
  */
 const ParameterInterface * parameters_ptr_;
};

#endif  // PIXELWISE_MASK_GENERATION_H
