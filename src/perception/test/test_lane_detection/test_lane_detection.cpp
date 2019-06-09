#include <common/macros.h>

THIRD_PARTY_HEADERS_BEGIN
#include <memory>
#include <boost/make_unique.hpp>
#include <gtest/gtest.h>
#include <ros/ros.h>
#include <ros/callback_queue.h>

#include <boost/optional.hpp>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <cv_bridge/cv_bridge.h>

#include <sensor_msgs/Image.h>
#include <nav_msgs/Path.h>
THIRD_PARTY_HEADERS_END

#include "common/parameter_handler.h"
#include "common/camera_transformation.h"
#include "common/path_conversion.h"

#include "perception_types.h"
#include "common/testee_module.h"

DISABLE_SUGGEST_OVERRIDE_WARNING

class LaneDetectionTest : public testing::Test {
 public:
  LaneDetectionTest()
      : node_handle("~"),
        last_image(boost::none),
        timeout(node_handle.param("timeout", 1.0)),
        image_count(0),
        no_passing_line_present(node_handle.param("no_passing_line_existent", false)),
        island_existent(node_handle.param("island_existent", false)),
        eps(node_handle.param("epsilon_set_image", 1)),
        image_to_check(node_handle.param("image_to_check", 1)),
        parameter_handler(node_handle),
        view_transformation(boost::make_unique<common::CameraTransformation>(&parameter_handler)),
        timesync(image_sub, left_line_sub, middle_line_sub, right_line_sub, middle_line_no_passing_sub, 10) {

    timesync.registerCallback(boost::bind(
        &LaneDetectionTest::handleImageAndLane, this, _1, _2, _3, _4, _5));
  }
  ros::NodeHandle node_handle;

 protected:
  boost::optional<ros::Time> last_image;
  const ros::Duration timeout;
  unsigned int image_count;
  bool no_passing_line_present;
  bool island_existent;
  double eps;
  unsigned int image_to_check;

  // Test interface
  void SetUp() override {
    image_sub.subscribe(node_handle, "labled_image", 10);
    right_line_sub.subscribe(node_handle, "road_lane_right", 10);
    middle_line_sub.subscribe(node_handle, "road_lane_middle", 10);
    left_line_sub.subscribe(node_handle, "road_lane_left", 10);
    middle_line_no_passing_sub.subscribe(
        node_handle, "road_lane_middle_no_passing", 10);
  }

 private:
  bool checkCorrectness(const cv::Vec3b& pixel, const cv::Vec3b set) {
    return cv::norm(pixel, set, cv::NORM_L2) <= eps;
  }


  void handleImageAndLane(const sensor_msgs::ImageConstPtr& img_msg,
                          const nav_msgs::PathConstPtr& left_line_msg,
                          const nav_msgs::PathConstPtr& middle_line_msg,
                          const nav_msgs::PathConstPtr& right_line_msg,
                          const nav_msgs::PathConstPtr& middle_no_passing_msg) {
    image_count++;
    ROS_INFO("callback on image %d", image_count);

    std::cout << image_count << std::endl;


    //    if (!node_handle.getParam("middle_line_existent",middle_line_present))
    //    {
    //      ROS_ERROR("parameter 'middle_line_existent' not setted.");
    //      std::exit(-1);
    //    }

    if (image_to_check == 0 || image_to_check == image_count) {
      cv_bridge::CvImageConstPtr cv_ptr =
          cv_bridge::toCvShare(img_msg, sensor_msgs::image_encodings::BGR8);

      const unsigned int correct_left =
          checkLine(cv_ptr->image, *left_line_msg, cv::Vec3b(255, 0, 0));
      const unsigned int correct_middle =
          checkLine(cv_ptr->image, *middle_line_msg, cv::Vec3b(0, 255, 0));
      const unsigned int correct_right =
          checkLine(cv_ptr->image, *right_line_msg, cv::Vec3b(0, 0, 255));
      const unsigned int correct_middle_no_passing =
          checkLine(cv_ptr->image, *middle_no_passing_msg, cv::Vec3b(255, 0, 255));

      const unsigned int max_fails = 1U;

      EXPECT_LE(left_line_msg->poses.size() - correct_left, max_fails);
      EXPECT_LE(middle_line_msg->poses.size() - correct_middle, max_fails);
      EXPECT_LE(right_line_msg->poses.size() - correct_right, max_fails);

      if (!island_existent) {
        EXPECT_LE(middle_no_passing_msg->poses.size() - correct_middle_no_passing, max_fails);
        EXPECT_EQ(no_passing_line_present, !middle_no_passing_msg->poses.empty());
      }

      const unsigned int min_correct = 100;
      EXPECT_GE(correct_left + correct_middle + correct_right + correct_middle_no_passing,
                min_correct);

      last_image = ros::Time::now();
    }
  }

  unsigned int checkLine(const cv::Mat& img,
                         const nav_msgs::Path& path_msg,
                         const cv::Vec3b& correct_color) {

    VehiclePoints points;
    tf2::fromMsg(path_msg, points);

    ImagePoints image_points;
    view_transformation->transformVehicleToImage(points, &image_points);
    unsigned int correct = 0;
    for (const ImagePoint& p : image_points) {
      if (checkCorrectness(img.at<cv::Vec3b>(p(1), p(0)), correct_color)) {
        //      if (img.at<cv::Vec3b>(p(1), p(0)) == correct_color) {
        correct++;
      } else {
        ROS_WARN_STREAM("in image " << image_count
                                    << " color: " << img.at<cv::Vec3b>(p(1), p(0))
                                    << " at (" << p(0) << "," << p(1)
                                    << "). Expected: " << correct_color);
        ROS_WARN_STREAM("in image "
                        << image_count << " norm: "
                        << cv::norm(img.at<cv::Vec3b>(p(1), p(0)), correct_color, cv::NORM_L2)
                        << " at (" << p(0) << "," << p(1) << "). Expected: " << eps);
      }
    }
    return correct;
  }

  common::ParameterHandler parameter_handler;
  std::unique_ptr<common::CameraTransformation> view_transformation;

  message_filters::Subscriber<sensor_msgs::Image> image_sub;
  message_filters::Subscriber<nav_msgs::Path> right_line_sub;
  message_filters::Subscriber<nav_msgs::Path> middle_line_sub;
  message_filters::Subscriber<nav_msgs::Path> left_line_sub;
  message_filters::Subscriber<nav_msgs::Path> middle_line_no_passing_sub;
  message_filters::TimeSynchronizer<sensor_msgs::Image, nav_msgs::Path, nav_msgs::Path, nav_msgs::Path, nav_msgs::Path> timesync;
};

unsigned int getParam(const std::string& name) {
  int x;
  if (!ros::NodeHandle("~").getParam(name, x)) {
    ROS_ERROR("parameter '%s' not setted.", name.c_str());
    std::exit(-1);
  }
  if (x >= 0) {
    return static_cast<unsigned int>(x);
  } else {
    ROS_ERROR("parameter '%s' should be unsigned but is: %d", name.c_str(), x);
    std::exit(-1);
  }
}

TEST_F(LaneDetectionTest, testLaneDetection) {
  TesteeModule UNUSED lane_detection(node_handle, "/perception/lane_detection");
  TesteeModule UNUSED preprocessing(node_handle, "/perception/preprocessing");
  TesteeModule UNUSED rosbag_nodebase(node_handle, "/common/rosbag_nodebase");
  ROS_INFO("entered test!");
  ros::Time current = ros::Time::now();
  const unsigned int expected_images = getParam("expected_images");
  ROS_INFO("Value of expected_images parameter is %u", expected_images);
  do {
    //    ROS_INFO("entered do while loop!");
    ros::getGlobalCallbackQueue()->callAvailable(ros::WallDuration(0.1));
    current = ros::Time::now();
  } while ((expected_images > image_count) &&
           (!last_image || (current - last_image.get()) < timeout));
  EXPECT_EQ(static_cast<unsigned int>(expected_images), image_count);
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "test_lane_detection");
  return RUN_ALL_TESTS();
}
