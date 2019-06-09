#include "parkingslotfinder.h"
#include <common/macros.h>

THIRD_PARTY_HEADERS_BEGIN
#include <Eigen/Core>
#include <tf2_eigen/tf2_eigen.h>
THIRD_PARTY_HEADERS_END

ParkingSlotFinder::ParkingSlotFinder()
    : one_d_mapper(param_reader.getMinObstacleLen(), param_reader.getMaxRampYDiff()),
      transform_listener(transform_buffer),
      ignore_distance(param_reader.getStartIgnoreDistance()),
      min_slot_size(param_reader.getMinSlotSize()),
      max_slot_size(param_reader.getMaxSlotSize()) {}

bool ParkingSlotFinder::findParkingSlot() {
  ROS_INFO("Setting up module.");
  ros::NodeHandle node;
  pslot_pub = node.advertise<geometry_msgs::PointStamped>("parkingrange", 10);
  ros::Subscriber parkinglot_map_subscriber =
      node.subscribe("map", 10, &ParkingSlotFinder::parkinglotMapCallback, this);
  ros::Subscriber discipline_subscriber = node.subscribe(
      "mission_mode", 1, &ParkingSlotFinder::disciplineChangedCallback, this);
  ros::Subscriber automode_subscriber = node.subscribe(
      "auto_reset", 1, &ParkingSlotFinder::autonomousModeCallback, this);
  ros::Subscriber ir_back_subscriber = node.subscribe(
      "infrared_sensor_back", 1, &ParkingSlotFinder::infraredBackCallback, this);


  slot_search_finished = false;
  slot_search_canceled = false;
  slot_found = false;
  ignore_distance = param_reader.getStartIgnoreDistance();
  min_slot_size = param_reader.getMinSlotSize();
  max_slot_size = param_reader.getMaxSlotSize();
  map_publisher_ = node.advertise<nav_msgs::OccupancyGrid>("one_d_map", 1);

  ROS_INFO_STREAM("Module set up. Go.");

  while (ros::ok() && !slot_search_finished && !slot_search_canceled) {
    ros::spinOnce();
  }

  return slot_found;
}



void ParkingSlotFinder::disciplineChangedCallback(std_msgs::Int32) {
  ROS_INFO_STREAM("Mission mode changed. Stopping search");
  slot_search_canceled = true;
}

void ParkingSlotFinder::autonomousModeCallback(std_msgs::Empty) {
  ROS_INFO_STREAM("AutonomousMode changed.");
  // slot_search_finished = true;
}

void ParkingSlotFinder::parkinglotMapCallback(const nav_msgs::OccupancyGridConstPtr& parkinglot_map) {
  if (currentStage != SearchStage::MEASURE_SLOTS) {
    return;
  }

  nav_msgs::OccupancyGrid one_d_map = one_d_mapper.mapToOneD(*parkinglot_map);
  map_publisher_.publish(one_d_map);  // For debugging

  // Set speed using the free space behind the vehicle.
  // That means the parkingslot is measured before having passed it
  // completely.
  float free_space_behind = measureFreeSpaceBehindVehicle(one_d_map);
  float free_space_ahead = measureFreeSpaceAheadVehicle(one_d_map);

  float current_parkingslot_length = free_space_ahead + free_space_behind;

  bool stop_at_this_slot = current_parkingslot_length > min_slot_size &&
                           current_parkingslot_length < max_slot_size;

  speed_controller.setSpeedForSlotSize(free_space_behind, free_space_ahead, stop_at_this_slot);

  ROS_INFO_STREAM("Measured parking slot: " << current_parkingslot_length
                                            << " m");
  if (stop_at_this_slot && free_space_ahead <= 0.05) {
    ROS_INFO_STREAM(
        "Found parkingslot inside given interval. Aligning to parking lot.");
    // publishParkingSpotTf(slot_start, slot_end);
    ros::param::set(PARAM_HLC_SPEED, 0.1);
    currentStage = SearchStage::ALIGN_TO_SLOT;
  }
}

int ParkingSlotFinder::getCurrentPosMapIndex(nav_msgs::OccupancyGrid& one_d_map) {
  tf2::Stamped<Eigen::Vector3d> vehicle_in_parkinglot;
  try {
    vehicle_in_parkinglot = transform_buffer.transform(
        tf2::Stamped<Eigen::Vector3d>(
            Eigen::Vector3d::Zero(), ros::Time(0), "vehicle"),
        "parkinglot");
  } catch (const tf2::TransformException& e) {
    ROS_WARN_STREAM("Could not transform from vehicle to parkinglot: " << e.what());
    return -1;
  }

  float passed_distance = vehicle_in_parkinglot.y();
  return std::min(one_d_map.info.height,
                  static_cast<unsigned int>(passed_distance / one_d_map.info.resolution));
}

void ParkingSlotFinder::infraredBackCallback(sensor_msgs::Range val) {
  if (currentStage != SearchStage::ALIGN_TO_SLOT) {
    return;
  }
  if (val.range < 0.2) {
    stopCar();
    ROS_INFO("Aligned.");
    slot_found = true;
    slot_search_finished = true;
  }
}


static inline bool isMapEntryOccupied(const int8_t value) {
  return value == OneDParkinglotMapper::VALUE_OCCUPIED;
}

static inline bool isMapEntryAtLeastFiltered(const int8_t value) {
  return value >= OneDParkinglotMapper::VALUE_FILTERED_OUT;
}

float ParkingSlotFinder::measureFreeSpaceBehindVehicle(nav_msgs::OccupancyGrid& one_d_map) {
  int map_index_current_position = getCurrentPosMapIndex(one_d_map);
  if (map_index_current_position == -1) {
    ROS_WARN_THROTTLE(1,
                      "Could not determine current map index, unable to "
                      "calculate free space behind.");
  }
  int map_index_ignore_distance =
      static_cast<int>(ignore_distance / one_d_map.info.resolution);
  // Measure the free space behind the vehicle.
  int i = map_index_current_position;
  while ((i > map_index_ignore_distance) && (!isMapEntryOccupied(one_d_map.data[i]))) {
    --i;
  }
  // Start measuring from the first obstacle behind the ignore distance.
  float free_space_length;
  if (isMapEntryOccupied(one_d_map.data[i])) {
    free_space_length = (map_index_current_position - i) * one_d_map.info.resolution;
  } else {
    free_space_length = 0;
  }

  return free_space_length;
}

float ParkingSlotFinder::measureFreeSpaceAheadVehicle(nav_msgs::OccupancyGrid& one_d_map) {
  int map_index_current_position = getCurrentPosMapIndex(one_d_map);
  if (map_index_current_position == -1) {
    ROS_WARN_THROTTLE(1,
                      "Could not determine current map index, unable to "
                      "calculate free space ahead.");
  }
  int map_index_ignore_distance =
      static_cast<int>(ignore_distance / one_d_map.info.resolution);
  // Measure the free space ahead the vehicle.
  int i = map_index_current_position;
  while ((i < static_cast<int>(one_d_map.info.height)) &&
         ((i < map_index_ignore_distance) ||
          (!isMapEntryAtLeastFiltered(one_d_map.data[i])))) {
    ++i;
  }
  // Start measuring from the first obstacle behind the ignore distance.
  float free_space_length;
  free_space_length = (i - map_index_current_position) * one_d_map.info.resolution;

  return free_space_length;
}

void ParkingSlotFinder::startDriving() {
  ParamReader params;

  ros::param::set(PARAM_HLC_SPEED, params.getVMax());
  ROS_INFO("hl controller activated.");
}

void ParkingSlotFinder::stopCar() {
  ROS_INFO("Invoking navigation services to stop the car.");
  ros::NodeHandle node;
  ros::ServiceClient activate_hl_controller_client =
      node.serviceClient<common_msgs::ActivationService>(
          "/control/longitudinal_controller/activate_module");


  ros::param::set(PARAM_HLC_SPEED, 0.);

  common_msgs::ActivationService activate_hl_controller_srv;
  activate_hl_controller_srv.request.moduleActive = false;
  activate_hl_controller_client.waitForExistence();
  activate_hl_controller_client.call(activate_hl_controller_srv);

  ROS_INFO("[x] Car Stopped");
  ROS_INFO("[x] HL Controller deactivated.");
  ROS_INFO("Ready to go in!");
}
