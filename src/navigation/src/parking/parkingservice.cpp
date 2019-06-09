#include "parkingservice.h"
#include <common/macros.h>

THIRD_PARTY_HEADERS_BEGIN
#include <navigation_msgs/FindParkingSlot.h>
THIRD_PARTY_HEADERS_END

bool ParkingService::startParking(navigation_msgs::DriveIntoParkingSlotRequest &req,
                                  navigation_msgs::DriveIntoParkingSlotResponse &res) {
  ROS_INFO("Parking Service was called");
  parking_sequence->executeParkingSequence(req, res);
  return (res.margin_back < 0 || res.margin_back > 0.02)
         && (res.margin_front < 0 || res.margin_front > 0.02)
         && (res.angle_offset < 6);
}


ParkingService::ParkingService(ros::NodeHandle *nh, const std::string& name,
                               ParkingOrchestrator *parking_seq, ros::CallbackQueue *service_callback_queue) :
        parking_sequence(parking_seq), service_callback_queue(service_callback_queue) {
  ros::AdvertiseServiceOptions opts =
          ros::AdvertiseServiceOptions::create<navigation_msgs::DriveIntoParkingSlot>(
                  name,
                  boost::bind(&ParkingService::startParking, this, _1, _2),
                  ros::VoidPtr(),
                  service_callback_queue
          );
  server = nh->advertiseService(opts);
}

