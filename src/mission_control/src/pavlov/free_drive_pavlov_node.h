#ifndef FREE_DRIVE_PAVLOV_NODE_H
#define FREE_DRIVE_PAVLOV_NODE_H

#include "common/node_base.h"
#include "node_helper/pavlov_node_helper.h"
#include "parking_fsm/data_parking.h"
#include "parking_fsm/free_drive_fsm.h"
#include "node_helper/state_machine_logger.h"

const ParameterString<bool> PARAM_ENABLE_QR_CODE_DETECTION(
    "enable_qr_code_detection");

class FreeDrivePavlovNode : public NodeBase {
 public:
  using ParkingStateMachine =
      sml::sm<SM::FreeDriveStateMachine, sml::logger<StateMachineLogger>>;

  /*!
   * \brief FreeDrivePavlovNode the constructor.
   * \param node_handle the NodeHandle to be used.
   */
  FreeDrivePavlovNode(ros::NodeHandle& node_handle);
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

  void handleStartLine(const perception_msgs::StartLines::ConstPtr& lines_msg);
  void handleParkingSpots(const perception_msgs::PerpendicularParkingSpots::ConstPtr& parking_spots_msg);
  void handlePath(const nav_msgs::Path::ConstPtr& path_msg);
  void handleReset(const std_msgs::Empty::ConstPtr&);

  bool pathToWorld(const ros::Time& stamp,
                   const Eigen::Affine3d& path_pose,
                   Eigen::Affine3d& world_pose);

  ros::Subscriber start_line_subscriber;
  ros::Subscriber parking_spot_subscriber;
  ros::Subscriber path_subscriber;
  ros::Subscriber parking_lot_found_subscriber;
  std::shared_ptr<int> number_of_park_attempts_ptr;
  ros::Subscriber reset_subscriber;
  CarController car_controller;
  DiagnosticsInterface diagnostics_iface;
  std::shared_ptr<DataParking> data_parking_;
  StateMachineLogger logger_;
  std::shared_ptr<ParkingStateMachine> free_drive_state_machine;
  PavlovNodeHelper pavlov_node_helper_;
};

#endif  // FREE_DRIVE_PAVLOV_NODE_H
