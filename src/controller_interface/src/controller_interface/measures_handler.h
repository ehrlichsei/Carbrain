#ifndef MEASURES_HANDLER_H
#define MEASURES_HANDLER_H
#include <common/macros.h>

THIRD_PARTY_HEADERS_BEGIN
#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>
#include <sensor_msgs/Range.h>
#include "controller_msgs/StateMeasure.h"
#include "measure.pb.h"
#include <common_msgs/MissionMode.h>
THIRD_PARTY_HEADERS_END

#include <controller_interface/sensormeasurements.h>
#include "controller_interface/logging.h"
#include <common/realtimeipc.h>
#include <common/concurrent_queue.h>
#include <common/realtime_channel_ids.h>

class MeasuresHandler {
 public:
  MeasuresHandler();

  /*!
   * realtime part of measure handling
   * handles ipc to non realtime part of this class and to other realtime
   * components
   * \param protobuf_measure received measure
   */
  void handleReceivedMeasures(const kitcar::Measure& protobuf_measure,
                              const ros::Time& stamp);

  /*!
   * ROS part of measure handling
   * waits for new measure and writes measures in the corresponging message
   */
  void getSensorMessage(controller_msgs::StateMeasure& state_measure);

  bool getManualMode(std_msgs::Bool& manual_mode);

  bool getSnapshotRequest();

  bool getMissionMode(common_msgs::MissionMode& mission_mode);

  void publishDistanceSensores(ros::Publisher& front_ir_sensor_publisher_,
                               ros::Publisher& back_ir_sensor_publisher_,
                               ros::Publisher& front_us_sensor_publisher_,
                               ros::Publisher& back_us_sensor_publisher_);

  void setWriteToSharedMemory(bool write_to_shared_memory);

  void setLog(LoggingQueue* log);

 private:
  common::ConcurrentQueue<SensorMeasurements, 10> sensor_measurements_queue_;
  common::RealtimeIPC<SensorMeasurements> sensor_measurements_ipc_;
  common::RealtimeIPC<bool> manual_mode_for_control_;
  common::ConcurrentQueue<kitcar::Measure_Mode, 10> mode_queue_;
  common::ConcurrentQueue<google::protobuf::RepeatedPtrField<kitcar::Measure_DistanceMeasure>, 10> distance_sensors_queue_;
  common::ConcurrentQueue<bool, 10> manual_mode_queue_;
  common::ConcurrentQueue<bool, 10> snapshot_request_queue_;
  LoggingQueue* log_ = std::nullptr_t();
  bool write_to_shared_memory_ = true;
};

#endif  // MEASURES_HANDLER_H
