#ifndef PARKINGACTION_H
#define PARKINGACTION_H
#include <common/macros.h>

THIRD_PARTY_HEADERS_BEGIN
#include <ros/ros.h>
#include <ros/callback_queue.h>
#include "navigation_msgs/DriveIntoParkingSlot.h"
THIRD_PARTY_HEADERS_END

#include "parkingcarinterface.h"
#include "parkingorchestrator.h"

/**
 * @brief encapsulates ros service functionality to expose the parking service
 */
class ParkingService {
 public:
  ParkingService(ros::NodeHandle *, const std::string&, ParkingOrchestrator *, ros::CallbackQueue *);

 protected:
  bool startParking(navigation_msgs::DriveIntoParkingSlotRequest &req,
                    navigation_msgs::DriveIntoParkingSlotResponse &res);

  ParkingOrchestrator *parking_sequence;
  ros::ServiceServer server;
  ros::CallbackQueue *service_callback_queue;

};

#endif // PARKINGSERVICE_H
