#ifndef EXTRINSIC_CALIBRATION_NODE_H
#define EXTRINSIC_CALIBRATION_NODE_H
#include <common/macros.h>

THIRD_PARTY_HEADERS_BEGIN
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <message_filters/subscriber.h>
THIRD_PARTY_HEADERS_END

#include "common/node_base.h"

#include "extrinsic_calibration.h"

/*!
 * \brief Publishes extrinsic parameters
 */
class ExtrinsicCalibrationNode : public NodeBase {
 public:
  /*!
   * \brief ExtrinsicCalibrationNode the constructor.
   * \param node_handle the NodeHandle to be used.
   */
  ExtrinsicCalibrationNode(ros::NodeHandle& node_handle);
  /*!
   * \brief returns the name of the node. It is needed in main and onInit
   * (nodelet) method.
   * \return the name of the node
   */
  static const std::string getName();

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

  void handleImages(const sensor_msgs::ImageConstPtr& image_raw_msg);

  /*!
   * \brief extrinsic_calibration contains the ROS-indipendent implementation of
   * this node.
   */
  ExtrinsicCalibration extrinsic_calibration_;

  image_transport::Subscriber image_raw_sub_;

  ros::Publisher rospub_img_debug_;
};

#endif  // EXTRINSIC_CALIBRATION_NODE_H
