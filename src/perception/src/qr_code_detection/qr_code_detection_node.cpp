#include "qr_code_detection_node.h"
#include "perception_message_conversions.h"

#include <perception_msgs/QrCodes.h>
#include "common/node_creation_makros.h"

THIRD_PARTY_HEADERS_BEGIN
#include <cv_bridge/cv_bridge.h>
#include <eigen_conversions/eigen_msg.h>
#include <boost/range/algorithm/transform.hpp>
THIRD_PARTY_HEADERS_END

// Needed for fancy debug
#include FANCY_DEBUG_INCLUDE("debug/qr_code_detection_node_debug.h")

namespace perception {
namespace qr_code_detection {

QrCodeDetectionNode::QrCodeDetectionNode(ros::NodeHandle &node_handle)
    : NodeBase(node_handle),
      qr_code_detection_(&parameter_handler_),
      image_transport_(node_handle) {
  // Abb initializations of class member here. This function is meant to be
  // called at construction time an shall be called ONLY in the constructors.
}

void QrCodeDetectionNode::startModule() {
  // sets your node in running mode. Activate publishers, subscribers, service
  // servers, etc here.
  qr_code_publisher_ =
      node_handle_.advertise<perception_msgs::QrCodes>("qr_codes", 5);

  image_subscriber_ = image_transport_.subscribe(
      "image", 1, boost::bind(&QrCodeDetectionNode::imageCallback, this, _1));

  activation_server_ = node_handle_.advertiseService(
      "activate_qr_code_detection_service", &QrCodeDetectionNode::activateQrCodeDetection, this);
}

void QrCodeDetectionNode::stopModule() {
  // sets your node in idle mode. Deactivate publishers, subscribers, service
  // servers, etc here.
  image_subscriber_.shutdown();
  qr_code_publisher_.shutdown();
}

void QrCodeDetectionNode::imageCallback(const sensor_msgs::ImageConstPtr &image_msg) {
  cv_bridge::CvImageConstPtr img_ptr = cv_bridge::toCvShare(image_msg, "mono8");

  auto qr_codes = qr_code_detection_.detect(img_ptr->image);

  publishQrCodes(qr_codes, img_ptr->header);
}


perception_msgs::QrCode toMsg(const QrCode &qr_code, const std_msgs::Header &header) {
  perception_msgs::QrCode qr_code_msg;
  qr_code_msg.data.data = qr_code.data;
  qr_code_msg.type.data = qr_code.type;
  qr_code_msg.location = perception::message_conversion::toMsg(qr_code.location);
  qr_code_msg.pose_stamped.header = header;
  tf::poseEigenToMsg(qr_code.pose.cast<double>(), qr_code_msg.pose_stamped.pose);

  return qr_code_msg;
}


void QrCodeDetectionNode::publishQrCodes(const QrCodes &qr_codes,
                                         const std_msgs::Header &header) const {
  perception_msgs::QrCodes qr_codes_msg;
  qr_codes_msg.header = header;

  std_msgs::Header pose_header = header;
  pose_header.frame_id = "vehicle";

  qr_codes_msg.qr_codes.reserve(qr_codes.size());
  boost::transform(qr_codes,
                   std::back_inserter(qr_codes_msg.qr_codes),
                   boost::bind(toMsg, _1, boost::cref(pose_header)));

  qr_code_publisher_.publish(qr_codes_msg);
}

bool QrCodeDetectionNode::activateQrCodeDetection(std_srvs::SetBoolRequest &req,
                                                  std_srvs::SetBoolResponse & /*res*/) {
  if (req.data) {
    image_subscriber_ = image_transport_.subscribe(
        "image", 1, boost::bind(&QrCodeDetectionNode::imageCallback, this, _1));
  } else {
    image_subscriber_.shutdown();
  }
  return true;
}

const std::string QrCodeDetectionNode::getName() {
  return std::string("qr_code_detection");
}


}  // namespace qr_code_detection
}  // namespace perception

using namespace perception::qr_code_detection;

CREATE_NODE_WITH_FANCY_DEBUG(QrCodeDetectionNode, debug::QrCodeDetectionNodeDebug)
