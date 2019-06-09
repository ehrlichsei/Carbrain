#ifndef PARKINGORCHESTRATOR_H
#define PARKINGORCHESTRATOR_H
#include <common/macros.h>

THIRD_PARTY_HEADERS_BEGIN
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/Range.h>
#include <tf/tf.h>
#include <navigation_msgs/DriveIntoParkingSlotRequest.h>
#include <navigation_msgs/DriveIntoParkingSlotResponse.h>
#include "controller_msgs/LightsCommand.h"
THIRD_PARTY_HEADERS_END

#include "parkingparameters.h"
#include "parkingcarinterface.h"
#include "vehiclestate.h"
#include "rosparkingdebugutil.h"

class ParkingOrchestrator {

 public:
  ParkingOrchestrator();

  ParkingOrchestrator(ParkingParameters, ParkingCarInterface &, RosParkingDebugUtil &, ros::Publisher &);

  bool executeParkingSequence(navigation_msgs::DriveIntoParkingSlotRequest &,
                              navigation_msgs::DriveIntoParkingSlotResponse &);

  void setOrientationEstimation(tf::Quaternion q);

  void setRangeSensors(const sensor_msgs::RangeConstPtr& range);

 private:
  VehicleState getStartingState(navigation_msgs::DriveIntoParkingSlotRequest &);

  VehicleState getOptimalPreEndState(navigation_msgs::DriveIntoParkingSlotRequest &);

  ros::Publisher &blinker_publisher;
  controller_msgs::LightsCommand lights_msg;

  bool turningPointReached();

  bool postStartPointReached();

  bool preEndReached();

  bool endReached();

  void waitForInitialisation();


  /*!
   * \brief solveOptimalPath calculates the parameters the optimal parking path.
   * For this, a geometrical construction is used:
   *  ____             ____
   * |    | (g)       |    |
   * |    |           |    |
   * |____|      (p)  |____|
   *
   *                   (s)
   *
   * (g) is the determined best position before reaching the end state. This is usually
   * on the outer end of the parking spot at an angle of ~70°.
   * (s) is the starting point where the car is before starting the parking action.
   * What needs to be determined is (p), where the steering is reversed to reach g.
   * this is done by laying out circles with the car's steering radius going through (s)
   * and (g). (p) is the position where a third circle with radius r that touches the circles of
   * (s) and (g) intersects the circle of (g).
   */
  void solveOptimalPath(navigation_msgs::DriveIntoParkingSlotRequest &);

  tf::Quaternion start_orientation;
  ParkingParameters parameters;
  ParkingCarInterface car;
  double turn_angle = 0.0;
  double post_start_angle = 0.0;
  tf::Quaternion current_orientation;
  bool orientation_set = false;
  sensor_msgs::RangeConstPtr range;
  RosParkingDebugUtil &ros_parking_debug;
};

#endif // PARKINGORCHESTRATOR_H
