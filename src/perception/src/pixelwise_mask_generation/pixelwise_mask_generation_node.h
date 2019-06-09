#ifndef PIXELWISE_MASK_GENERATION_NODE_H
#define PIXELWISE_MASK_GENERATION_NODE_H
#include <common/macros.h>

THIRD_PARTY_HEADERS_BEGIN
#include <dynamic_reconfigure/server.h>
#include <image_transport/image_transport.h>
#include <ros/ros.h>
#include <opencv2/imgproc/imgproc.hpp>

#include "pixelwise_mask_generation.h"
#include "pixelwise_mask_generation_msgs/BilateralFilter.h"
#include "pixelwise_mask_generation_msgs/Dilation.h"
#include "pixelwise_mask_generation_msgs/Closing.h"
#include "pixelwise_mask_generation_msgs/MedianBlur.h"
#include "pixelwise_mask_generation_msgs/SaveMask.h"
#include "pixelwise_mask_generation_msgs/AutomaticMaskGen.h"
#include "pixelwise_mask_generation_msgs/TakePictures.h"
#include "pixelwise_mask_generation_msgs/SetStartingPoint.h"
THIRD_PARTY_HEADERS_END

#include "common/node_base.h"

namespace pixelwise_mask_generation {
/*!
 * \brief The PixelwiseMaskGenerationNode class
 *
 * This class serves as the interface between ROS and the standalone
 * PixelwiseMaskGeneration implementation.
 *
 */
class PixelwiseMaskGenerationNode : public NodeBase {
 public:
  /*!
   * \brief PreprocessingNode the constructor meant to be used in the context of
   * the
   * Nodelet, because the NodeHandle has to obtained otherways in the context of
   * Nodelets.
   * \param node_handle the NodeHandle to be used.
   */
   PixelwiseMaskGenerationNode(ros::NodeHandle& node_handle);
  /*!
   * \brief returns the name of the node. It is needed in main and onInit
   * (nodelet) method.
   * \return the name of the node
   */
  static const std::string getName();


  /*!
   * \brief takePictures
   *
   * takes multiple pictures for the mask generation
   *
   */
  bool takePicturesCallback(pixelwise_mask_generation_msgs::TakePictures::Request &req,
                            pixelwise_mask_generation_msgs::TakePictures::Response &res);

  /*!
   * \brief saveMaskCallback
   *
   * calls function saveMask to save the mask
   *
   */
  bool saveMaskCallback(pixelwise_mask_generation_msgs::SaveMask::Request &req,
                        pixelwise_mask_generation_msgs::SaveMask::Response &res);

  /*!
   * \brief automaticMaskGenCallback
   *
   * generates mask automatically
   *
   */
  bool automaticMaskGenCallback(pixelwise_mask_generation_msgs::AutomaticMaskGen::Request &req,
                               pixelwise_mask_generation_msgs::AutomaticMaskGen::Response &res);

  /*!
   * \brief medianBlurv
   *
   * takes multiple pictures for the mask generation
   *
   */
  bool medianBlurCallback(pixelwise_mask_generation_msgs::MedianBlur::Request &req,
                          pixelwise_mask_generation_msgs::MedianBlur::Response &res);

  /*!
   * \brief dilationCallback
   *
   * erodes the not-mask-area => mask gets bigger
   *
   */
  bool dilationCallback(pixelwise_mask_generation_msgs::Dilation::Request &req,
                       pixelwise_mask_generation_msgs::Dilation::Response &res);

  /*!
   * \brief closingCallback
   *
   * closes the mask
   *
   */
  bool closingCallback(pixelwise_mask_generation_msgs::Closing::Request &req,
                       pixelwise_mask_generation_msgs::Closing::Response &res);

  /*!
   * \brief bilateralFilterCallback
   *
   * bilateral filter
   *
   */
  bool bilateralFilterCallback(pixelwise_mask_generation_msgs::BilateralFilter::Request &req,
                               pixelwise_mask_generation_msgs::BilateralFilter::Response &res);


  /*!
   * \brief setStartingPointCallback
   *
   * set the starting point of the segmentation
   *
   */
  bool setStartingPointCallback(pixelwise_mask_generation_msgs::SetStartingPoint::Request &req,
                                pixelwise_mask_generation_msgs::SetStartingPoint::Response &res);



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

 private:
  // NodeBase interface
  /*!
   * \brief startModule is called, if the node shall be turned active. In here
   * the subrscribers an publishers are started.
   */
  void startModule() override;
  /*!
   * \brief stopModule is called, if the node shall be turned inactive. In this
   * function subscribers and publishers are shutdown.
   */
  void stopModule() override;

  /**
   * @brief handleImage
   *
   * Call-back for incomming images.
   * Recives image from Camera; publishes preprocessed image.
   *
   * @param msg the message
   */
  void handleImage(const sensor_msgs::ImageConstPtr& msg);

  /*!
   * \brief rossub_image
   *
   * Suscribes camera image.
   */
  image_transport::Subscriber rossub_image;


  /*!
   * \brief rospub_preprocessed_image
   *
   * Publisher for the mask image
   */
  ros::Publisher rospub_mask_image;

  /*!
   * \brief rospub_preprocessed_image
   *
   * Publisher for the mask contour image
   */
  ros::Publisher rospub_image_mask_contour;

  /*!
   * \brief serv_save_mask
   *
   * Service Server for mask saving.
   */
  ros::ServiceServer serv_save_mask;

  /*!
   * \brief serv_median_blur
   *
   * Service Server for median blur.
   */
  ros::ServiceServer serv_median_blur;

  /*!
   * \brief serv_automatic_mask_gen
   *
   * Service Server for automatic mask generation.
   */
  ros::ServiceServer serv_automatic_mask_gen;

  /*!
   * \brief serv_take_pictures
   *
   * Service Server for taking pictures for mask generation.
   */
  ros::ServiceServer serv_take_pictures;

  /*!
   * \brief servDilation
   *
   * Service Server for the dilation of the mask
   */
  ros::ServiceServer serv_dilation;

  /*!
   * \brief serv_closing
   *
   * Service Server for the closing of the mask
   */
  ros::ServiceServer serv_closing;

  /*!
   * \brief serv_bilateral_filter
   *
   * Service Server for the bilateral filtering of the mask
   */
  ros::ServiceServer serv_bilateral_filter;

  /*!
   * \brief serv_set_starting_point
   *
   * Service Server for setting the starting point for the segmentation
   */
  ros::ServiceServer serv_set_starting_point;

  /*!
   * \brief old_mask
   *
   * mask from the mask file
   */
  cv::Mat old_mask;

  /*!
   * \brief show_old_mask
   *
   * true if no mask segmentation was published yet
   */
  bool show_old_mask;

  /*!
   * \brief mask_loaded
   *
   * true if mask file loaded
   */
  bool mask_loaded;


  /*!
   * \brief preprocessing contains the ROS-indipendent implementation of this
   * node.
   */
  PixelwiseMaskGeneration pixelwise_mask_generation_;

};
}
#endif  // PIXELWISE_MASK_GENERATION_NODE_H
