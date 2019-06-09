#ifndef VEHICLESTATE_H
#define VEHICLESTATE_H

class VehicleState {
 public:
  VehicleState() {

  }

  VehicleState(double x, double y, double angle) :
          x(x), y(y), angle(angle) { }

  double x;
  double y;
  double angle;
};

#endif // VEHICLESTATE_H
