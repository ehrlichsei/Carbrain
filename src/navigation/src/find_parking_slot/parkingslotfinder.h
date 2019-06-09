#ifndef PARKING_SLOT_FINDER
#define PARKING_SLOT_FINDER
#include <common/macros.h>

THIRD_PARTY_HEADERS_BEGIN
#include <tf/tf.h>
#include <tf2_ros/buffer.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/OccupancyGrid.h>
#include <std_msgs/Int32.h>
#include <stdint.h>
#include <algorithm>
#include <common_msgs/ActivationService.h>
#include <sensor_msgs/Range.h>
THIRD_PARTY_HEADERS_END

#include "speedcontroller.h"
#include "onedparkinglotmapper.h"


enum class SearchStage { MEASURE_SLOTS, ALIGN_TO_SLOT };

class ParkingSlotFinder {
 public:
  ParkingSlotFinder();
  bool findParkingSlot();

 private:
  ParamReader param_reader;
  SpeedController speed_controller;
  OneDParkinglotMapper one_d_mapper;
  tf2_ros::Buffer transform_buffer;
  tf2_ros::TransformListener transform_listener;
  SearchStage currentStage = SearchStage::MEASURE_SLOTS;
  bool slot_search_canceled = false;
  bool slot_search_finished = false;
  bool slot_found = false;
  ros::Publisher map_publisher_;
  ros::Publisher pslot_pub;
  float ignore_distance;
  float min_slot_size;
  float max_slot_size;

  void startDriving();
  void stopCar();
  void parkinglotMapCallback(const nav_msgs::OccupancyGridConstPtr& parkinglot_map);
  int getCurrentPosMapIndex(nav_msgs::OccupancyGrid& one_d_map);
  float measureFreeSpaceBehindVehicle(nav_msgs::OccupancyGrid& one_d_map);
  float measureFreeSpaceAheadVehicle(nav_msgs::OccupancyGrid& one_d_map);
  void locationCallback(geometry_msgs::PoseStampedConstPtr pose);
  void checkPassedIgnoreDistance();

  void disciplineChangedCallback(std_msgs::Int32 val);
  void autonomousModeCallback(std_msgs::Empty val);
  void infraredBackCallback(sensor_msgs::Range val);
};

#endif  //PARKING_SLOT_FINDER
