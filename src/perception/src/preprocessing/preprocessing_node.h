#ifndef PREPROCESSING_NODE_H
#define PREPROCESSING_NODE_H
#include <common/macros.h>

THIRD_PARTY_HEADERS_BEGIN
#include <image_transport/image_transport.h>
THIRD_PARTY_HEADERS_END

#include "common/node_base.h"

#include "preprocessing.h"

/*!
 * \brief This node performs preprocessing steps on a raw camera image.
 */
class PreprocessingNode : public NodeBase {
 public:
  /*!
 * \brief PreprocessingNode the constructor.
 * \param node_handle the NodeHandle to be used.
 */
  PreprocessingNode(ros::NodeHandle& node_handle);
  /*!
 * \brief returns the name of the node. It is needed in main and onInit
 * (nodelet) method.
 * \return the name of the node
 */
  static const std::string getName();

 private:
  static const ParameterString<int> INPUT_QUEUE_SIZE;
  static const ParameterString<int> OUTPUT_QUEUE_SIZE;

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

  /**
     * @brief rossub_image
     *
     * Subscribes camera image.
     */
  image_transport::Subscriber rossub_image;
  /**
   * @brief rospub_preprocessed_image
   *
   * Publisher for output images.
   */
  ros::Publisher rospub_preprocessed_image;

  ros::Publisher rospub_region_of_interest;

  /*!
 * \brief preprocessing contains the ROS-independent implementation of this
 * node.
 */
  Preprocessing preprocessing_;
};

#endif  // PREPROCESSING_NODE_H
