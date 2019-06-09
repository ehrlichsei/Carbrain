#ifndef PREPROCESSING_H
#define PREPROCESSING_H
#include <common/macros.h>

THIRD_PARTY_HEADERS_BEGIN
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
THIRD_PARTY_HEADERS_END

#include "common/parameter_interface.h"
#include "../utils/ego_vehicle.h"

/*!
 * \brief This node performs preprocessing steps on a raw camera image.
 */
class Preprocessing {
 public:
  /*!
  * \brief Preprocessing is the constructor. A ros indipendent functionality
  * containing
  * class needs a pointer to a ParameterInterface (in fact a ParameterHandler)
  * to get access to parameters.
  * \param parameters the ParameterInterface
  */
  Preprocessing(ParameterInterface* parameters);

  /*!
   * \brief preprocessImage
   *
   * perfroms image preprocessing (grayscale, binarisation, masking car parts in
   *the image, cutting out region of interest).
   *
   * @param image_gray the source image
   * @param preprocessed_image the output image
   */
  void preprocessImage(const cv::Mat& image_gray, cv::Mat& preprocessed_image);

  const cv::Rect& getRegionOfInterest();

  /*!
   * \brief isUsingTrapezoidMaskMode
   * \returns if trapezoid mask mode is used
   */
  bool isUsingTrapezoidMaskMode();

 private:
  enum MaskMode {
    MASK_MODE_TRAPEZOID = 0,
    MASK_MODE_STANDARD_PATH = 1,
    MASK_MODE_MODIFIED_PATH = 2
  };

  /*!
   * \brief BINARISTATION_MODE
   *
   * Chooses the binraisation method: 0 no binarisation, 1 Threshold, 2+3
   *adaptive Threshold, >=4 Otsu
   */
  static const ParameterString<int> BINARISTATION_MODE;
  /*!
   * \brief BINARISTATION_SIGMA
   *
   * sigma for Threshold binarisation
   */
  static const ParameterString<double> BINARISTATION_SIGMA;
  /*!
   * \brief BINARISTATION_BLOCK_SIZE
   *
   * the blocksize of the adaptive threshold binarisation (must be odd and
   *positiv!)*
   */
  static const ParameterString<int> BINARISTATION_BLOCK_SIZE;
  /*!
   * \brief BINARISTATION_C
   *
   * a constant substrahend for threshold binarisation;
   */
  static const ParameterString<double> BINARISTATION_C;
  /*!
   * \brief MASK_MODE
   *
   * mask mode: 0 for trapezoid mask; 1 for pixelwise mask from standard mask
   *file; 2 for pixelwise mask from modified mask path
   */
  static const ParameterString<int> MASK_MODE;
  /*!
   * \brief STANDARD_MASK_DIRECTORY
   *
   * directory of the standard mask file for the pixelwise mask in standard mask
   *mode (folder location without file name; '~' for HOME directory)
   */
  static const ParameterString<std::string> STANDARD_MASK_DIRECTORY;
  /*!
   * \brief STANDARD_MASK_NAME
   *
   * file name for the standard mask file for the pixelwise mask in standard
   *mask mode
   */
  static const ParameterString<std::string> STANDARD_MASK_NAME;
  /*!
   * \brief MODIFIED_MASK_PATH
   *
   * path of the modified mask file for the pixelwise mask in modified mask mode
   *(full path including file name; '~' for HOME directory)
   */
  static const ParameterString<std::string> MODIFIED_MASK_PATH;
  /*!
   * \brief ROI_START_LOC_X
   *
   * Start location (x-axis) of the ROI area relative to recorded image size.
   */
  static const ParameterString<int> ROI_START_LOC_X;
  /*!
   * \brief ROI_START_LOC_Y
   *
   * Start location (y-axis) of the ROI area relative to recorded image size.
   */
  static const ParameterString<int> ROI_START_LOC_Y;
  /*!
   * \brief ROI_END_LOC_X
   *
   * End location (x-axis) of the ROI area relative to recorded image size.
   */
  static const ParameterString<int> ROI_END_LOC_X;
  /*!
   * \brief ROI_END_LOC_Y
   *
   * End location (y-axis) of the ROI area relative to recorded image size.
   */
  static const ParameterString<int> ROI_END_LOC_Y;

  /*!
   * \brief MEDIAN_BLUR_SIZE
   *
   * size of the median blur kernel: has to be odd, zero to deactivate median
   *blur
   */
  static const ParameterString<int> MEDIAN_BLUR_SIZE;


  static void registerPreprocessingParameters(ParameterInterface* parameter_interface);

  /*!
   * \brief refreshMask
   *
   * after changing the maskMode parameter, you have to refresh the mask.
   *refreshing the mask loads the mask from the specified mask file in pixelwise
   *Mask Mode
   *
   */
  void refreshMask();

  /*!
   * \brief cutMaskToROI
   *
   * cuts pixelwise mask to ROI (Range Of Interest). You have to call
   *cutMaskToROI after every change of the ROI-Parameters, after loading a new
   *mask and after every start of the perception node to avoid errors. Therefore
   *readMask and callbackDynamicReconfigureRoi call cutMaskToROI.
   *
   */
  bool cutMaskToROI();

  /**
     * @brief binarizeImageOtsu
     *
     * performs binarization using Otsu's Method.
     *
     * @param image_grey the source image
     * @param image_binary the target image
     */
  inline void binarizeImageOtsu(const cv::Mat& image_grey, cv::Mat& image_binary);

  /**
   * @brief binarizeImageThresh
   *
   * performs binarization using a threshold. The threshold is based on mean and
   *standard deviation of source image.
   *
   * @param image_grey the source image
   * @param sigma the weight of the standard derivation to use.
   * @param image_binary the target image
   */
  inline void binarizeImageThresh(const cv::Mat& image_grey, double sigma, cv::Mat& image_binary);

  /**
   * @brief binarizeAdaptiveThresholdWithC
   *
   *  performs binarization using an adaptive threshold. the threshold is based
   *an the mean of the points neighbourhood minus C plus weighted global
   *standard derivation.
   * @param image_grey the input image
   * @param blocksize Size of a pixel neighborhood (must be odd!)
   * @param sigma wheight ot the global standard derivation
   * @param c the subrtahend
   * @param image_binary the putput image
   */
  inline void binarizeAdaptiveThreshold(const cv::Mat& image_grey,
                                        int blocksize,
                                        double sigma,
                                        double c,
                                        cv::Mat& image_binary);

  /*!
   * \brief readMask
   *
   * Reads mask from harddrive and modifies mask to the ROI set in the
   *parameters. If readMask() fails to read the file (e.g. because the file is
   *missing), it returns 0, otherwise 1.
   * \param mask_path the directory of the mask file (with name)
   */
  inline bool readMask(std::string mask_path);

  /*!
   * \brief readMask
   *
   * Reads mask from harddrive and modifies mask to the ROI set in the
   *parameters. If readMask() fails to read the file (e.g. because the file is
   *missing), it returns 0, otherwise 1.
   * \param mask_directory the directory of the mask file (without name)
   * \param mask_name the name of the mask file
   */
  inline bool readMask(std::string mask_directory, const std::string& mask_name);

  /*!
   * \brief readMask
   *
   * updates ROI and mask if the ROI changed
   */
  inline void updateImageLimits();

  /*!
  * \brief parameters_ptr_ is needed for parameter access
  */
  const ParameterInterface* parameters_ptr_;

  EgoVehicle ego_vehicle;

  cv::Rect image_limits;

  /**
   * @brief params The parameters of the preprocessing.
   */

  /*!
   * \brief the mask mode from the parameter server. In difference to
   * use_trapezoid_mask_mode mask_mode does NOT take into consideration whether
   * the loading of the mask file failed.
   */
  int mask_mode;

  /*!
   * \brief use_trapezoid_mask_mode determines, whether trapezoid mask mode is
   * used (true) or the pixelwise mask mode (false)
   */
  bool use_trapezoid_mask_mode = true;

  /*!
   * \brief mask (if pixelwise mask is used)
   */
  cv::Mat mask;

  /*!
   * \brief mask cut to roi (if pixelwise mask is used)
   */
  cv::Mat mask_roi;
};

#endif  // PREPROCESSING_H
