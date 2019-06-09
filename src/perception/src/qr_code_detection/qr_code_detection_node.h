#ifndef QR_CODE_DETECTION_NODE_H
#define QR_CODE_DETECTION_NODE_H

#include "common/macros.h"
#include "common/node_base.h"
#include "qr_code_detection.h"

THIRD_PARTY_HEADERS_BEGIN
#include <image_transport/image_transport.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <std_srvs/SetBool.h>
THIRD_PARTY_HEADERS_END

namespace perception {
namespace qr_code_detection {


/*!
 * \brief Node to detect QR codes like the 'STOP' signal
 */
class QrCodeDetectionNode : public NodeBase {
 public:
  /*!
   * \brief QrCodeDetectionNode the constructor.
   * \param node_handle the NodeHandle to be used.
   */
  QrCodeDetectionNode(ros::NodeHandle& node_handle);

  virtual ~QrCodeDetectionNode() override = default;

  /*!
   * \brief returns the name of the node. It is needed in main and onInit
   * (nodelet) method.
   * \return the name of the node
   */
  static const std::string getName();

 protected:
  /*!
   * \brief imageCallback processes an incoming image message. The detected QR
   * codes are published afterwards.
   * \param image_msg incoming message
   */
  void virtual imageCallback(const sensor_msgs::ImageConstPtr& image_msg);

  /*!
   * \brief publishQrCodes
   * \param qr_codes list of detected QR codes
   * \param header corresponding header from input image message
   */
  void virtual publishQrCodes(const QrCodes& qr_codes, const std_msgs::Header& header) const;

  /*!
   * \brief called by the state machine to activate and deactivate the node
   * \param req contains bool which determines if the node should be activated
   * (true) or deactivated (false)
   * \param res not used response
   */
  bool activateQrCodeDetection(std_srvs::SetBoolRequest& req, std_srvs::SetBoolResponse& res);


 protected:
  // NodeBase interface
  /*!
   * \brief startModule is called, if the node shall be turned active. In here
   * the subrscribers an publishers are started.
   */
  virtual void startModule() override;
  /*!
   * \brief stopModule is called, if the node shall be turned inactive. In this
   * function subscribers and publishers are shutdown.
   */
  virtual void stopModule() override;


 private:
  /*!
   * \brief qr_code_detection contains the ROS-indipendent implementation of this node.
   */
  QrCodeDetection qr_code_detection_;

  image_transport::Subscriber image_subscriber_;  //! subscribes to arbitrary image topic
  image_transport::ImageTransport image_transport_;

  ros::Publisher qr_code_publisher_;  //! publisher for QR codes

  ros::ServiceServer activation_server_;
};


}  // namespace qr_code_detection
}  // namespace perception


#endif  // QR_CODE_DETECTION_NODE_H
