#ifndef PERPENDICULAR_PARKING_NODE_H
#define PERPENDICULAR_PARKING_NODE_H

#include <common/macros.h>
#include <common/node_base.h>

THIRD_PARTY_HEADERS_BEGIN
#include <memory>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <nav_msgs/Path.h>
#include <perception_msgs/ParkingLotFound.h>
#include <perception_msgs/PerpendicularParkingSpots.h>
#include <perception_msgs/SearchParkingSpots.h>
#include <sensor_msgs/Image.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
THIRD_PARTY_HEADERS_END


#include "../road_object_detection/road_object_visitors/message_handler.h"
#include "../utils/tf_helper.h"
#include "perpendicular_parking.h"

/*!
 * \brief Node for detection and localization of parking spots on the left part
 * of the lane, that can be used for perpendicular parking
 */
class PerpendicularParkingNode : public NodeBase {
 public:
  /*!
   * \brief PerpendicularParkingNode the constructor.
   * \param node_handle the NodeHandle to be used.
   */
  PerpendicularParkingNode(ros::NodeHandle& node_handle);
  virtual ~PerpendicularParkingNode() override = default;
  /*!
   * \brief returns the name of the node. It is needed in main and onInit
   * (nodelet) method.
   * \return the name of the node
   */
  static const std::string getName();

 protected:
  // NodeBase interface
  // protected for interface to debug
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


  virtual void handleLeftLaneAndImage(const sensor_msgs::ImageConstPtr& image_raw_msg,
                                      const nav_msgs::PathConstPtr& left_lane_msg,
                                      const nav_msgs::PathConstPtr& middle_lane_msg,
                                      const nav_msgs::PathConstPtr& right_lane_msg,
                                      const nav_msgs::PathConstPtr& no_passing_lane_msg);

  tf2_ros::Buffer tf2_buffer_;
  std::unique_ptr<common::CameraTransformation> camera_transformation_;

  std::unique_ptr<perpendicular_parking::PerpendicularParking> perpendicular_parking_;

 private:
  static const ParameterString<int> INPUT_QUEUE_SIZE;
  static const tf_helper::FrameIDs FRAME_IDS;

  static uint32_t getInputQueueSize(ParameterInterface* const parameter_ptr);

  bool activateService(perception_msgs::SearchParkingSpots::Request& request,
                       perception_msgs::SearchParkingSpots::Response& response);


  void createMsg(perception_msgs::PerpendicularParkingSpots& free_parking_spots,
                 const ros::Time& timestamp);

  const perception_msgs::PerpendicularParkingSpots toPerpendicularParkingSpots(
      perpendicular_parking::ParkingSpotsConstRef free_spots);

  const perception_msgs::ParkingLotFound toParkingLotFound(const bool found,
                                                           const ros::Time& timestamp);

  tf2_ros::TransformListener tf2_listener_;
  tf_helper::TFHelper<double> world_coordinates_helper_;

  ros::ServiceServer parking_service;

  road_object_detection::MessageHelper<perception_msgs::PerpendicularParkingSpots, perception_msgs::PerpendicularParkingSpot> parking_spot_helper;
  ros::Publisher parking_lot_found_pub;

  message_filters::Subscriber<sensor_msgs::Image> image_raw_sub;
  message_filters::Subscriber<nav_msgs::Path> lane_left_sub;
  message_filters::Subscriber<nav_msgs::Path> lane_middle_sub;
  message_filters::Subscriber<nav_msgs::Path> lane_right_sub;
  message_filters::Subscriber<nav_msgs::Path> lane_no_passing_sub;
  message_filters::TimeSynchronizer<sensor_msgs::Image, nav_msgs::Path, nav_msgs::Path, nav_msgs::Path, nav_msgs::Path> synchronizer;

  // std::unique_ptr<ROIBirdsViewTransformation> roi_birdsview_transformation_;
};

#endif  // PERPENDICULAR_PARKING_NODE_H
