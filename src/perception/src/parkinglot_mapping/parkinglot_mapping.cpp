#include "parkinglot_mapping.h"
#include <common/macros.h>

THIRD_PARTY_HEADERS_BEGIN
#include <sensor_msgs/Image.h>
#include <nav_msgs/Path.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
THIRD_PARTY_HEADERS_END

#include "parkinglot_map.h"

const ParameterString<double> ParkinglotMapping::PARKINGLOT_MAP_WIDTH("width");
const ParameterString<double> ParkinglotMapping::PARKINGLOT_MAP_HEIGHT(
    "height");
const ParameterString<double> ParkinglotMapping::PARKINGLOT_MAP_RESOLUTION(
    "resolution");

void ParkinglotMapping::registerParams(ParameterInterface &parameters) {
  parameters.registerParam(ParkinglotMapping::PARKINGLOT_MAP_WIDTH);
  parameters.registerParam(ParkinglotMapping::PARKINGLOT_MAP_HEIGHT);
  parameters.registerParam(ParkinglotMapping::PARKINGLOT_MAP_RESOLUTION);
}

std::unique_ptr<ParkinglotMapping> ParkinglotMapping::create(ParameterInterface *parameters,
                                                             const tf2_ros::Buffer &tf2_buffer) {
  return std::make_unique<ParkinglotMapping>(parameters, tf2_buffer);
}

ParkinglotMapping::ParkinglotMapping(ParameterInterface *parameters,
                                     const tf2_ros::Buffer &tf2_buffer)
    : parameters_ptr_(parameters),
      parkinglot_map_(parameters->getParam(PARKINGLOT_MAP_WIDTH),
                      parameters->getParam(PARKINGLOT_MAP_HEIGHT),
                      parameters->getParam(PARKINGLOT_MAP_RESOLUTION),
                      0.5f),
      birdsview_transformation_(parameters),
      tf2_buffer_(tf2_buffer) {
  ROS_INFO("Initialized map with width: %f, height: %f, resolution %f",
           parameters->getParam(PARKINGLOT_MAP_WIDTH),
           parameters->getParam(PARKINGLOT_MAP_HEIGHT),
           parameters->getParam(PARKINGLOT_MAP_RESOLUTION));
}

void ParkinglotMapping::mapRangeMeasurement(const sensor_msgs::Range::ConstPtr &range_measurement) {
  last_map_update_ = range_measurement->header.stamp;

  geometry_msgs::PoseStamped start_pose_in_ir_;
  start_pose_in_ir_.header.frame_id = range_measurement->header.frame_id;
  start_pose_in_ir_.header.stamp = range_measurement->header.stamp;
  start_pose_in_ir_.pose.position.x = 0;
  start_pose_in_ir_.pose.position.y = 0;
  start_pose_in_ir_.pose.position.z = 0;
  start_pose_in_ir_.pose.orientation.w = 1.0;

  geometry_msgs::PoseStamped end_pose_in_ir_ = start_pose_in_ir_;
  end_pose_in_ir_.pose.position.x = range_measurement->range;

  geometry_msgs::TransformStamped transform_to_parkinglot;
  try {
    transform_to_parkinglot =
        tf2_buffer_.lookupTransform("parkinglot",
                                    ros::Time(0),
                                    range_measurement->header.frame_id,
                                    range_measurement->header.stamp,
                                    "world",
                                    ros::Duration(0.1));
  } catch (const tf2::ConnectivityException &ex) {
   ROS_WARN(
       "Lookup of transform from parkinglot to ir sensor failed "
       "(ConnectivityException): %s",
       ex.what());
   return;
 } catch (const tf2::TransformException &ex) {
    ROS_WARN(
        "Lookup of transform from parkinglot to ir sensor failed "
        "(TransformException): %s",
        ex.what());
    return;
  }

  geometry_msgs::PoseStamped start_pose_in_parkinglot;
  geometry_msgs::PoseStamped end_pose_in_parkinglot;
  tf2::doTransform(start_pose_in_ir_, start_pose_in_parkinglot, transform_to_parkinglot);
  tf2::doTransform(end_pose_in_ir_, end_pose_in_parkinglot, transform_to_parkinglot);

  parkinglot_map_.updateDataOnRay(start_pose_in_parkinglot.pose.position.x,
                                  start_pose_in_parkinglot.pose.position.y,
                                  end_pose_in_parkinglot.pose.position.x,
                                  end_pose_in_parkinglot.pose.position.y,
                                  0.0f,
                                  0.7f);
}

const nav_msgs::OccupancyGrid ParkinglotMapping::getMap() {
  return parkinglot_map_.createMessage("parkinglot", last_map_update_, 1.0f);
}
