#ifndef LANE_DETECTION_NODE_H
#define LANE_DETECTION_NODE_H
#include <common/macros.h>

THIRD_PARTY_HEADERS_BEGIN
#include <memory>
#include <image_transport/image_transport.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Empty.h>
#include <nav_msgs/Path.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/subscriber.h>
#include <std_srvs/Empty.h>

#include "perception_msgs/RegionOfInterestStamped.h"
#include "perception_msgs/SetMiddleLine.h"
THIRD_PARTY_HEADERS_END

#include "common/node_base.h"
#include "lane_detection.h"

#include "roi_birds_view_transformation.h"
#include "../utils/ego_vehicle.h"

/*!
 * \brief This class perfroms the lane detection.
 */
class LaneDetectionNode : public NodeBase {
 public:
  /*!
   * \brief LaneDetectionNode the constructor.
   * \param node_handle the NodeHandle to be used.
   */
  LaneDetectionNode(ros::NodeHandle &node_handle);
  /*!
   * \brief returns the name of the node. It is needed in main and onInit
   * (nodelet) method.
   * \return the name of the node
   */
  static const std::string getName();

 protected:
  /*!
   * \brief Process an image provided by the camera.
   *
   * This function is used as call-back for subscribed preprocessed
   * images in ROS.
   *
   * \param image_msg the pointer to the raw camera image.
   * \param roi_msg the to the region of interest message.
   */
  virtual void handleImage(const sensor_msgs::ImageConstPtr &image_msg,
                           const perception_msgs::RegionOfInterestStampedConstPtr &roi_msg);

  // NodeBase interface
  /*!
   * \brief startModule is called, if the node shall be turned active. In here
   * the subscribers and publishers are started.
   */
  virtual void startModule() override;
  /*!
   * \brief stopModule is called, if the node shall be turned inactive. In this
   * function subscribers and publishers are shutdown.
   */
  virtual void stopModule() override;

  std::unique_ptr<FeatureExtraction> feature_extraction_;
  std::unique_ptr<LaneDetection> lane_detection_;
  bool reset(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);

  std::unique_ptr<EgoVehicle> ego_vehicle_;

 private:
  static const ParameterString<int> INPUT_QUEUE_SIZE;
  static const ParameterString<int> OUTPUT_QUEUE_SIZE;

  /*!
   * @brief Resets the node.
   */
  void handleReset(const std_msgs::Empty);
  /*!
   * \brief handleSetStartBucht
   * \param msg_start_bucht the boolean message.
   */
  void handleSetStartBucht(const std_msgs::BoolConstPtr &msg_start_bucht);

  bool setMiddleLine(perception_msgs::SetMiddleLine::Request &req,
                     perception_msgs::SetMiddleLine::Response &);

  /**
   * ROS publisher for detected left line (points).
   */
  ros::Publisher rospub_lane_left;

  /**
   * ROS publisher for detected middle line (points).
   */
  ros::Publisher rospub_lane_middle;

  /*!
   * ROS publisher for detected middle line (points) when
   * car is driving in no passing zone
   *
   */
  ros::Publisher rospub_lane_middle_no_passing;

  /**
   * ROS publisher for detected right line (points).
   */
  ros::Publisher rospub_lane_right;
  /**
   * ROS subscriber for reset messanges.
   */
  ros::Subscriber rossub_reset;

  ros::Subscriber rossub_set_start_box;

  ros::ServiceServer rosserv_set_middle_line;
  ros::ServiceServer reset_server_;

  message_filters::Subscriber<sensor_msgs::Image> image_sub;
  message_filters::Subscriber<perception_msgs::RegionOfInterestStamped> roi_sub;
  message_filters::TimeSynchronizer<sensor_msgs::Image, perception_msgs::RegionOfInterestStamped> timesync_img;

  std::unique_ptr<ROIBirdsViewTransformation> roi_view_transformation;
};

#endif  // LANE_DETECTION_NODE_H
