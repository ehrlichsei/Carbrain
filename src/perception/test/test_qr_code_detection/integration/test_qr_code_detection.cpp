#include <common/macros.h>

THIRD_PARTY_HEADERS_BEGIN
#include <gtest/gtest.h>
#include <ros/callback_queue.h>
#include <ros/ros.h>
#include <tf2_eigen/tf2_eigen.h>
#include <boost/optional.hpp>
#include <memory>
#include <opencv2/imgproc.hpp>

#include "perception_msgs/QrCodes.h"
THIRD_PARTY_HEADERS_END

#include "common/testee_module.h"
#include "perception_types.h"

DISABLE_SUGGEST_OVERRIDE_WARNING

namespace perception {
namespace qr_code_detection {
namespace testing {


class QrCodeDetectionTest : public ::testing::Test {

 protected:
  ros::NodeHandle nh;

 private:
  ros::Subscriber qr_code_subscriber;

 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  QrCodeDetectionTest()
      : nh("~"),
        timeout(nh.param("timeout", 1.0)),
        name(nh.param("name", std::string("STOP"))) {
    EXPECT_TRUE(nh.hasParam("timeout"));
  }

 protected:
  const ros::Duration timeout;
  bool finished = false;  //! flag indicates successful test run

  const std::string name;  //! the expected name / code


  // Test interface
  void SetUp() override {
    qr_code_subscriber =
        nh.subscribe("qr_codes", 1, &QrCodeDetectionTest::handleQrCodes, this);
  }

  void TearDown() override { qr_code_subscriber.shutdown(); }

 private:
  void handleQrCodes(const perception_msgs::QrCodesConstPtr& msg) {
    ROS_INFO("Received message!");

    if (msg->qr_codes.empty()) {
      ROS_WARN("Message is empty!");
      return;
    }


    if (msg->qr_codes.front().data.data != name) {
      ROS_INFO_STREAM("The name is " << msg->qr_codes.front().data.data
                                     << " but should be " << name);
      return;
    }

    // OK, the QR code seems to be valid
    ROS_INFO("Valid QR code received!");
    finished = true;
  }
};

TEST_F(QrCodeDetectionTest, testQrCode) {
  TesteeModule UNUSED qr_code_detection(nh, "/perception/qr_code_detection");
  TesteeModule UNUSED rosbag_nodebase(nh, "/common/rosbag_nodebase");

  const ros::Time start = ros::Time::now();
  do {
    ros::getGlobalCallbackQueue()->callAvailable(ros::WallDuration(0.1));
  } while (!finished && ((ros::Time::now() - start) < timeout));
  EXPECT_TRUE(finished);
}

}  // namespace testing
}  // namespace qr_code_detection
}  // namespace perception

using namespace perception::qr_code_detection::testing;

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "test_qr_code_detection");
  return RUN_ALL_TESTS();
}
