#ifndef CARCONTROLLER_H
#define CARCONTROLLER_H

#include <common/macros.h>

THIRD_PARTY_HEADERS_BEGIN
#include <limits>
#include <ros/ros.h>
#include <tf2_eigen/tf2_eigen.h>
#include <eigen_conversions/eigen_msg.h>
#include <perception_msgs/LookAt.h>
#include <longitudinal_controller_msgs/StopAt.h>
#include <navigation_msgs/DrivePastNextRoadClosure.h>
#include <navigation_msgs/AutomaticCruiseControlUpdate.h>
#include <longitudinal_controller_msgs/DrivingSpeed.h>
#include <std_srvs/SetBool.h>
#include <navigation_msgs/ParkPerpendicular.h>
#include <navigation_msgs/ReverseOutOfParkingSpot.h>
#include <std_srvs/SetBool.h>
#include <nav_msgs/Path.h>
#include <controller_msgs/LightsCommand.h>
#include <perception_msgs/SearchParkingSpots.h>
#include <perception_msgs/PerpendicularParkingSpot.h>
#include <std_msgs/Float32.h>
#include <tf2_ros/transform_broadcaster.h>
#include <navigation_msgs/TurnAt.h>
#include <navigation_msgs/PavlovBlinkerCommand.h>
#include <controller_msgs/BlinkerCommand.h>
#include <longitudinal_controller_msgs/DrivingSpeed.h>

#include <std_srvs/Empty.h>
#include <navigation_msgs/TurnAt.h>
#include "navigation_msgs/SetRespectNoPassingZones.h"
THIRD_PARTY_HEADERS_END

#include "turn_direction.h"

class ICarController {
 public:
  virtual ~ICarController() = default;
  /*!
   * \brief Commands the Car control layer that the car should be stopped at a
   * certain
   * position.
   * \param distance the on-path distance of the stop-point in meters.
   */
  virtual unsigned long stopAtDistance(const double distance, const unsigned long id) = 0;

  /*!
   * \brief Starts normal driving.
   */
  virtual void startDriving(const unsigned long id) = 0;

  /*!
   * \brief sets the maximum speed the car is allowed to drive.
   * \param speed_limit_in_km_per_h the speed limit
   */
  virtual unsigned long setMaxSpeedInModelSizeKmPerH(const double speed_limit_in_km_per_h,
                                                     const unsigned long id) = 0;
  virtual unsigned long setMaxSpeedInMPerS(const double desired_speed_in_m_per_s,
                                           const unsigned long id) = 0;
  /*!
   * \brief clears the maximum speed limitation. The car returns to the
   * discipline-default speed.
   */
  virtual void clearMaxSpeed(const unsigned long id) = 0;
  virtual void setMaxSpeedAfterQRCode() = 0;
  virtual void publishACCMessage(const double speed, const double distance) = 0;
  virtual void setDrivePastNextRoadClosure(const bool should_drive_past) = 0;
  virtual void startLookAt(const tf2::Stamped<Eigen::Affine3d> pose,
                           const Eigen::Vector3d rect) = 0;
  virtual void stopLookAt() = 0;
  virtual void deActivateACC(const bool activate) = 0;

  virtual void searchParkingSpot(const bool search) = 0;
  virtual void parkPerpendicular(const perception_msgs::PerpendicularParkingSpot &parking_spot) = 0;
  virtual void straightPathOutOfStartBox(const bool enable) = 0;
  virtual void activateQrCodeDetection(const bool activate) = 0;
  virtual void reverseOutOfParkingSpot(const nav_msgs::Path &path_reverse_out_of_parking_spot_world,
                                       const bool enable) = 0;
  virtual void publishReversePathToWorldTransform(const Eigen::Affine3d &path_to_world_transform,
                                                  const ros::Time &stamp) = 0;
  virtual void turnAt(const Eigen::Affine3d &pose, const TurnDirection direction) = 0;
  virtual void resetTurning() = 0;
  virtual void resetLaneDetection() = 0;
  virtual void resetPathPreprocessing() = 0;
  virtual void setPavlovBlinkerCommand(controller_msgs::BlinkerCommand::_command_type command) = 0;
  virtual void blinkInParkingSpot(const bool blink) = 0;
  virtual void blinkLeftBeforeParkingSpot(const bool blink) = 0;
  virtual void setHighBeam(const bool high_beam) = 0;
  virtual void resetPavlovBlinkerCommand() = 0;
  virtual void setRespectNoPassingZone(bool respect_no_passing_zone) = 0;
  virtual void resetCarController() = 0;
  virtual void resetEnvironmentalModel() = 0;
};

/*!
 * \brief The CarController is a facade for the actual service calls and
 * message publications that control the car behaviour and provides convenience
 * methods for start / stop procedures of the car.
 */
class CarController : public ICarController {
 public:
  CarController(ros::NodeHandle node_handle);

  unsigned long stopAtDistance(const double distance, const unsigned long id) override;

  /*!
   * \brief Starts normal driving.
   */
  void startDriving(const unsigned long id) override;


  unsigned long setMaxSpeedInModelSizeKmPerH(const double speed_limit_in_km_per_h,
                                             const unsigned long id) override;
  unsigned long setMaxSpeedInMPerS(const double desired_speed_in_m_per_s,
                                   const unsigned long id) override;
  void clearMaxSpeed(unsigned long id) override;
  void setMaxSpeedAfterQRCode() override;
  void publishACCMessage(const double speed, const double distance) override;
  void setDrivePastNextRoadClosure(const bool should_drive_past) override;
  void startLookAt(const tf2::Stamped<Eigen::Affine3d> pose, const Eigen::Vector3d rect) override;
  void stopLookAt() override;
  void deActivateACC(const bool activate) override;
  void turnAt(const Eigen::Affine3d &pose, const TurnDirection direction) override;
  void resetTurning() override;
  void resetLaneDetection() override;
  void resetPathPreprocessing() override;
  void setPavlovBlinkerCommand(controller_msgs::BlinkerCommand::_command_type command) override;
  void blinkInParkingSpot(const bool blink) override;
  void blinkLeftBeforeParkingSpot(const bool blink) override;
  void setHighBeam(const bool high_beam) override;
  void resetPavlovBlinkerCommand() override;
  void setRespectNoPassingZone(bool respect_no_passing_zone) override;
  void searchParkingSpot(const bool search) override;
  void parkPerpendicular(const perception_msgs::PerpendicularParkingSpot &parking_spot) override;
  void straightPathOutOfStartBox(const bool enable) override;
  void activateQrCodeDetection(const bool activate) override;
  void reverseOutOfParkingSpot(const nav_msgs::Path &path_reverse_out_of_parking_spot,
                               const bool enable) override;
  void publishReversePathToWorldTransform(const Eigen::Affine3d &path_to_world_transform,
                                          const ros::Time &stamp) override;
  void resetCarController() override;
  void resetEnvironmentalModel() override;


 private:
  ros::ServiceClient stopping_client_;
  ros::ServiceClient drive_past_next_road_closure_client_;
  ros::ServiceClient look_at_client_;
  ros::ServiceClient acc_activation_;
  ros::ServiceClient set_respect_no_passing_zone_service_;
  ros::ServiceClient turn_at_client_;
  ros::ServiceClient search_parking_lot_client_;
  ros::ServiceClient plan_path_parking_lot_client_;
  ros::ServiceClient out_of_start_box_client_;
  ros::ServiceClient reverse_out_of_parking_spot_client_;
  ros::ServiceClient reset_path_preprocessing_client_;
  ros::ServiceClient reset_lane_detection_client_;
  ros::ServiceClient reset_environmental_model_client_;
  ros::ServiceClient activate_qr_code_detection_client_;
  ros::Publisher velocity_publisher;
  ros::ServiceClient set_pavlov_blinker_command_client_;
  ros::Publisher acc_publisher;
  ros::Publisher light_publisher;
  tf2_ros::TransformBroadcaster tf2_broadcaster_;
  // Remove the following when parallel speed limitations are implemented on
  // control side.
  unsigned long max_speed_id_counter;
  std::map<unsigned long, double> max_speed_limitations;
  double getMinMaxSpeedOrInfinity();
};

#endif  // CARCONTROLLER_H
