#include "parkinglot_aligner.h"
#include <common/macros.h>

THIRD_PARTY_HEADERS_BEGIN
#include "navigation/gate.h"
#include <tf2_eigen/tf2_eigen.h>
THIRD_PARTY_HEADERS_END

const ParameterString<double> PARAM_CAR_TO_ORIGIN_DISTANCE(
    "car_to_origin_distance");

ParkinglotAligner::ParkinglotAligner(ParameterInterface* parameters)
    : parameters_ptr_(parameters), calculatedTransformation(nullptr) {
  parameters->registerParam(PARAM_CAR_TO_ORIGIN_DISTANCE);
}


bool ParkinglotAligner::calculateTransformation(const DrivingCorridor& corridor,
                                                const Eigen::Vector3d& carPosition,
                                                Eigen::Affine3d& transformation) {

  const double carToOriginDistance = parameters_ptr_->getParam(PARAM_CAR_TO_ORIGIN_DISTANCE);
  if (corridor.empty()) {
    return false;
  }

  // negative => car behind startPoint
  double distanceToCar = -1;
  if (!newStartPoint && calculatedTransformation != nullptr) {
    distanceToCar = ((*calculatedTransformation).inverse() * carPosition).y();
  }
  Eigen::Quaterniond rotation;
  if (distanceToCar < carToOriginDistance) {
    Gate gate = getGateNextToPoint(corridor, startPoint);

    if (distanceToCar < 0) {
      pointOfOrigin = gate.getRightPole();
    }
    // use for orientation the gate near to the pointOfOrigin
    rotation = gate.getRotation();
  } else {
    Gate gate = getGateNextToPoint(corridor, carPosition);
    if ((gate.getRightPole() - pointOfOrigin).norm() < 1e-10) {
      return false;
    }
    // use for orientation the vector between the pointOfOrigin and the right
    // pole of the gate near to the car
    rotation.setFromTwoVectors(Eigen::Vector3d::UnitY(), gate.getRightPole() - pointOfOrigin);
  }

  transformation = Eigen::Translation3d(pointOfOrigin) * rotation;

  calculatedTransformation = std::make_unique<Eigen::Affine3d>(transformation);

  // the start point was already used to calculate the pointOfOrigin
  newStartPoint = false;
  return true;
}

Gate ParkinglotAligner::getGateNextToPoint(const DrivingCorridor& corridor,
                                           const Eigen::Vector3d& point) {
  const Gate* gate = &corridor.at(0);
  // search nearest gate to the point
  double cur_min_distance = (corridor.at(0).getRightPole() - point).norm();
  for (size_t i = 1; i < corridor.size(); ++i) {
    double current_distance = (corridor.at(i).getRightPole() - point).norm();
    if (current_distance < cur_min_distance) {
      gate = &corridor.at(i);
      cur_min_distance = current_distance;
    }
  }
  return *gate;
}

Eigen::Vector3d ParkinglotAligner::getStartPoint() { return startPoint; }
void ParkinglotAligner::setStartPoint(const Eigen::Vector3d& point) {
  startPoint = point;
  // the start point should be used to calculate a new pointOfOrigin
  newStartPoint = true;
}
