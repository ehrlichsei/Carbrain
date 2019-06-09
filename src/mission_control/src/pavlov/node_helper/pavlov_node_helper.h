#ifndef PAVLOV_NODE_HELPER_H
#define PAVLOV_NODE_HELPER_H

#include <common/macros.h>

THIRD_PARTY_HEADERS_BEGIN
#include <limits>
#include <functional>
#include <ros/callback_queue.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <std_msgs/Time.h>
#include <nav_msgs/Path.h>
#include <navigation_msgs/DrivingCorridor.h>
#include <navigation_msgs/Obstacles.h>
#include <navigation_msgs/NoPassingZones.h>
#include <navigation_msgs/Crosswalks.h>
#include <perception_msgs/Junctions.h>
#include <navigation_msgs/RoadClosures.h>
#include <perception_msgs/SpeedLimitMarkings.h>
#include <perception_msgs/PerpendicularParkingSpots.h>
#include <perception_msgs/Unidentifieds.h>
#include <perception_msgs/QrCodes.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PolygonStamped.h>
#include <perception_msgs/StartLines.h>
#include <perception_msgs/ParkingLotFound.h>
#include <std_msgs/UInt64.h>
#include <boost/range/algorithm/sort.hpp>
#include <boost/bind.hpp>
THIRD_PARTY_HEADERS_END

#include "../fsm_utils/events.h"
#include <common/node_base.h>
#include "driving_corridor_checker.h"
#include "turn_checker.h"
#include "passing_point_checker.h"
#include "common/node_creation_makros.h"
#include "common/tf2_eigen_addon.h"
#include "common/eigen_utils.h"
#include "common/math.h"


class PavlovNodeHelper {
 public:
  PavlovNodeHelper();
  template <class T>
  void periodicTimerCallback(const std::shared_ptr<T>& state_machine, const ros::TimerEvent&) {
    updatePosition(state_machine);
    updateTime(state_machine);
  }

  template <class T>
  void handleStoppingCompleted(const std::shared_ptr<T>& state_machine,
                               const std_msgs::UInt64::ConstPtr& stop_completed_msg) {
    state_machine->process_event(EventCarHasStopped(stop_completed_msg->data));
  }

  template <class T>
  void startModule(const std::shared_ptr<T>& state_machine, ros::NodeHandle& node_handle) {
    periodic_timer_ = node_handle.createTimer(
        ros::Duration(0.0166),
        boost::bind(&PavlovNodeHelper::periodicTimerCallback<T>, this, boost::ref(state_machine), _1));  // 0.0166=60hz
    stopping_completed_subscriber = node_handle.subscribe<std_msgs::UInt64>(
        "stopping_completed",
        20,
        boost::bind(&PavlovNodeHelper::handleStoppingCompleted<T>, this, boost::ref(state_machine), _1));

    qr_code_subscriber = node_handle.subscribe<perception_msgs::QrCodes>(
        "qr_codes",
        20,
        boost::bind(&PavlovNodeHelper::handleQRCodes<T>, this, boost::ref(state_machine), _1));
  }
  void stopModule();

  template <class T>
  void updatePosition(const std::shared_ptr<T>& state_machine) {
    Eigen::Affine3d vehicle_pose;
    if (!vehicleToWorld(Eigen::Affine3d::Identity(), vehicle_pose)) {
      return;
    }
    if (previous_vehicle_pos != Eigen::Vector3d::Zero()) {
      double drove_distance = (vehicle_pose.translation() - previous_vehicle_pos).norm();
      state_machine->process_event(EventDroveDistance(drove_distance));
    }
    previous_vehicle_pos = vehicle_pose.translation();
  }

  template <class T>
  void updateTime(const std::shared_ptr<T>& state_machine) {
    state_machine->process_event(EventTimeUpdate(ros::Time::now()));
  }

  bool worldToVehicle(const Eigen::Vector3d& world_pos, Eigen::Vector3d& vehicle_pos);
  bool vehicleToWorld(const Eigen::Affine3d& vehicle_pose, Eigen::Affine3d& world_pose);
  bool vehicleToWorld(const ros::Time& stamp,
                      const Eigen::Affine3d& vehicle_pose,
                      Eigen::Affine3d& world_pose);

  bool pathToWorld(const ros::Time& stamp,
                   const Eigen::Affine3d& path_pose,
                   Eigen::Affine3d& world_pose);

  template <class T>
  void handleQRCodes(const std::shared_ptr<T>& state_machine,
                     const perception_msgs::QrCodesConstPtr& msg) {
    if (msg->qr_codes.empty()) {
      state_machine->process_event(EventNoQRCodeVisible());
      return;
    }
    state_machine->process_event(EventQRDetection());
  }

 private:
  tf2_ros::TransformListener tf_listener;
  tf2_ros::Buffer tf_buffer;
  Eigen::Vector3d previous_vehicle_pos;
  ros::Subscriber reset_subscriber;
  ros::Subscriber stopping_completed_subscriber;
  ros::Subscriber qr_code_subscriber;

  ros::Timer periodic_timer_;
};

#endif  // PAVLOV_NODE_HELPER_H
