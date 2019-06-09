#include "vehiclestate.h"
#include "circle.h"
#include "parkingparameters.h"
#include <vector>

#ifndef PARKING_PARKING_PATH_SOLVER
#define PARKING_PARKING_PATH_SOLVER

class ParkingPathSolver {

 public:
  ParkingPathSolver(VehicleState const &startingState, VehicleState const &goalPreEndState)
          : starting_state(startingState), goal_pre_end_state(goalPreEndState) {
    max_turn_radius_l = max_turn_radius_r = 0.7f;
  }

  void solve();


  double getTurningPointAngle() const {
    return turning_point_angle;
  }

  double getPreBeginAngle() const {
    return pre_begin_angle;
  }


  void setStartingState(VehicleState const &starting_state) {
    ParkingPathSolver::starting_state = starting_state;
  }

  void setGoalPreEndState(VehicleState const &goal_pre_end_state) {
    ParkingPathSolver::goal_pre_end_state = goal_pre_end_state;
  }


  std::vector<Circle> const &getDebugCircles() const {
    return debug_circles;
  }

  void setMaxTurnRadius(double max_turn_radius_l, double max_turn_radius_r) {
    ParkingPathSolver::max_turn_radius_l = max_turn_radius_l;
    ParkingPathSolver::max_turn_radius_r = max_turn_radius_r;
  }

 private:

  double max_turn_radius_l;
  double max_turn_radius_r;

  std::vector<Circle> debug_circles;

  /**
  * The state of the vehicle at the beginning
  */
  VehicleState starting_state;

  /**
  * The desired Pre-End State of the vehicle.
  * The Pre-End State is the state reached before correction takes place
  */
  VehicleState goal_pre_end_state;


  /******************************
  * Result Variables
  ******************************/

  /**
  * The angle that defines when to invert the steering
  */
  double turning_point_angle;

  /**
  * The angle that must be reached by driving forwards before entering the parking spot
  */
  double pre_begin_angle;

};

#endif //PARKING_PARKING_PATH_SOLVER
