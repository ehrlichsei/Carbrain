#include "parkingorchestrator.h"
#include <common/macros.h>

THIRD_PARTY_HEADERS_BEGIN
#include <boost/shared_ptr.hpp>
#include <ros/ros.h>
#include <navigation_msgs/DriveIntoParkingSlotRequest.h>
THIRD_PARTY_HEADERS_END

#include "parkingcarinterface.h"
#include "vehiclestate.h"
#include "circle.h"
#include "parkingpathsolver.h"

/**
 * evaluation callback for the turning point
 * @return true, if the turning point of the current parking sequence is reached.
 */
bool ParkingOrchestrator::turningPointReached() {
  //TODO check estimated angle, driven distance, camera based orientation, etc..
  double angle = -tf::getYaw(current_orientation) + tf::getYaw(start_orientation);
  //TODO angleShortestPath might be more suited
  ROS_DEBUG("turning point reached in %f deg rad", (turn_angle - angle));
  return angle <= turn_angle;
}

/**
 * evaluation callback for the "post-start" point
 * @return true if the "post-start" point of the current parking sequence is reached.
 */
bool ParkingOrchestrator::postStartPointReached() {
  //TODO check estimated angle, driven distance, camera based orientation, etc..
  double angle = -tf::getYaw(current_orientation) + tf::getYaw(start_orientation);
  //TODO angleShortestPath might be more suited
  ROS_DEBUG("post_start_point reached in %f deg rad", (post_start_angle - angle));
  return angle <= post_start_angle;
}

/**
 * evaluation callback for the "pre-end" point
 * @return true, if the "pre-end" point of the current parking sequence is reached.
 */
bool ParkingOrchestrator::preEndReached() {
  //TODO check estimated angle, driven distance, camera based orientation AND US-sensor
  return -tf::getYaw(current_orientation) + tf::getYaw(start_orientation) > -(M_PI / 16);
}

/**
 * evaluation callback for the end point
 * @return true, if the end point of the current parking sequence is reached.
 */
bool ParkingOrchestrator::endReached() {
  //TODO check estimated angle, driven distance, camera based orientation AND US-sensor
  return -tf::getYaw(current_orientation) + tf::getYaw(start_orientation) >= parameters.end_angle_offset;
}

/**
 * blocks until the initial orientation is received
 */
void ParkingOrchestrator::waitForInitialisation() {
  for (int i = 0; i < 5; i++) { //wait for orientation to stabilize, might be a problem in the localization node
    while (!orientation_set) {
      ROS_INFO_THROTTLE(5, "no pose estimation has been set. waiting for one.");
      ros::spinOnce();
    }
    orientation_set = false;
  }
}

/**
* returns the starting state of the vehicle in parking spot coordinates
*/
VehicleState ParkingOrchestrator::getStartingState(navigation_msgs::DriveIntoParkingSlotRequest &request) {
  return VehicleState(
          -request.parking_slot_width - request.distance_x,
          request.parking_slot_length + request.distance_y,
          request.angle
  );
}

/**
* returns the chosen pre end state in parking spot coordinates
*/
VehicleState ParkingOrchestrator::getOptimalPreEndState(navigation_msgs::DriveIntoParkingSlotRequest &) {
  //TODO calculate
  return VehicleState(-0.10f, 0.1f, -M_PI / 16); //assuming this is a 70cm spot
}

/**
 * @brief calculates the optimal parking path from a parking request. It internally sets the turning angles needed to
 * execute the parking sequence and must be called before executeParkingSequence()
 */
void ParkingOrchestrator::solveOptimalPath(navigation_msgs::DriveIntoParkingSlotRequest &request) {
  //TODO handle transforms world <-> parking coordinate system
  ParkingPathSolver solver(getStartingState(request), getOptimalPreEndState(request));
  solver.setMaxTurnRadius(parameters.max_turn_radius_l, parameters.max_turn_radius_r);
  solver.solve();
  turn_angle = solver.getTurningPointAngle();
  ROS_INFO("turn_angle was determined to be %.4f", turn_angle);
  post_start_angle = solver.getPreBeginAngle();
  ROS_INFO("post_start_angle was determined to be %.4f", post_start_angle);
  ros_parking_debug.publishSupportCircles(solver.getDebugCircles());
}

/**
 * @brief Constructor
 */
ParkingOrchestrator::ParkingOrchestrator(ParkingParameters parameters, ParkingCarInterface &interface,
                                         RosParkingDebugUtil &ros_parking_debug, ros::Publisher &publisher)
        : blinker_publisher(publisher),
          parameters(parameters),
          car(interface),
          ros_parking_debug(ros_parking_debug) {
  turn_angle = M_PI / 8;
}

/**
 * @brief executes the parking sequence.
 * This methods blocks until the parking is finished and can therefore be used to implement a blocking service.
 */
bool ParkingOrchestrator::executeParkingSequence(navigation_msgs::DriveIntoParkingSlotRequest &request,
                                                 navigation_msgs::DriveIntoParkingSlotResponse &result) {
  lights_msg.blinker.command = controller_msgs::BlinkerCommand::RIGHT;
  blinker_publisher.publish(lights_msg);
  ROS_DEBUG("Waiting for initialisation of distance to obstacle and orientation");
  waitForInitialisation();
  solveOptimalPath(request);
  ROS_DEBUG("executing parking sequence");
  start_orientation = current_orientation;
  ROS_INFO("start pose set");
  car.steerMaxLeft();
  usleep(parameters.wait_between_turns * 1000);
  ROS_INFO("steering to left (max) set");
  ROS_INFO("forwards.");
  car.driveForwardUntil(boost::bind(&ParkingOrchestrator::postStartPointReached, this));
  car.steerMaxRight();
  usleep(parameters.wait_between_turns * 1000);
  ROS_INFO("backwards.");
  car.driveBackwardsUntil(boost::bind(&ParkingOrchestrator::turningPointReached, this));
  car.steerMaxLeft();
  usleep(parameters.wait_between_turns * 1000);
  car.driveBackwardsUntil(boost::bind(&ParkingOrchestrator::preEndReached, this));
  ROS_INFO("pre end reached");
  lights_msg.blinker.command = controller_msgs::BlinkerCommand::NONE;
  blinker_publisher.publish(lights_msg);
  car.steerMaxRight();
  usleep(parameters.wait_between_turns * 1000);
  car.driveForwardUntil(boost::bind(&ParkingOrchestrator::endReached, this));
  car.setSteeringAngle(0);

  result.angle_offset = tf::angleShortestPath(start_orientation, current_orientation);
  result.margin_back = -1; //TODO measure per distance sensors
  result.margin_front = -1;

  lights_msg.blinker.command = controller_msgs::BlinkerCommand::BOTH;
  blinker_publisher.publish(lights_msg);
  ros::Duration(3).sleep(); //! wait until we have blinked 3 times; use BOTH_THREE_TIMES if controller supports this
  lights_msg.blinker.command = controller_msgs::BlinkerCommand::NONE;
  blinker_publisher.publish(lights_msg);

  return true;
}

void ParkingOrchestrator::setOrientationEstimation(tf::Quaternion q) {
  this->orientation_set = true;
  this->current_orientation = q;
}

void ParkingOrchestrator::setRangeSensors(const sensor_msgs::RangeConstPtr& range) {
  this->range = range;
}
