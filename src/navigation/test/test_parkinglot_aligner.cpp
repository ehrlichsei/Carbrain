#include <common/macros.h>

THIRD_PARTY_HEADERS_BEGIN
#include <gtest/gtest.h>
#include <math.h>
THIRD_PARTY_HEADERS_END

#include "common/test/dummy_parameter_handler.h"

#include "../src/parkinglot_aligner/parkinglot_aligner.h"
#include "navigation/driving_corridor.h"

DISABLE_SUGGEST_OVERRIDE_WARNING

static ParameterString<double> DISTANCE("car_to_origin_distance");

class ParkinglotAlignerTest : public ::testing::Test {
 public:
  void compare(Eigen::Vector3d expected, Eigen::Vector3d result) {
    const double epsilon = 1e-10;
    ASSERT_NEAR(expected.x(), result.x(), epsilon);
    ASSERT_NEAR(expected.y(), result.y(), epsilon);
    ASSERT_NEAR(expected.z(), result.z(), epsilon);
  }
};

TEST_F(ParkinglotAlignerTest, isTransformationCorrect) {
  Gate::GateList gates;
  Gate gate(0, Eigen::Vector3d(0, 0, 0), Eigen::Vector3d(1, 0, 0));
  gates.push_back(gate);
  DrivingCorridor corridor(gates);
  DummyParameterHandler paramProvider;
  paramProvider.addParam(DISTANCE, 1.0);
  ParkinglotAligner parkinglotAligner(&paramProvider);

  Eigen::Affine3d transformation;
  ASSERT_TRUE(parkinglotAligner.calculateTransformation(
      corridor, Eigen::Vector3d::Zero(), transformation));
  compare(Eigen::Vector3d(1, 1, 0), transformation * Eigen::Vector3d(0, 1, 0));
  compare(Eigen::Vector3d(1, 2, 0), transformation * Eigen::Vector3d(0, 2, 0));
  compare(Eigen::Vector3d(0, 2, 0), transformation * Eigen::Vector3d(-1, 2, 0));
  compare(Eigen::Vector3d(2, 0, 0), transformation * Eigen::Vector3d(1, 0, 0));
}

TEST_F(ParkinglotAlignerTest, isTransformationCorrectRotatedGate) {

  Gate::GateList gates;

  Gate gate1(0, Eigen::Vector3d(1, 1, 0), Eigen::Vector3d(2, 2, 0));

  gates.push_back(gate1);
  DrivingCorridor corridor(gates);
  DummyParameterHandler paramProvider;
  paramProvider.addParam(DISTANCE, 1.0);
  ParkinglotAligner parkinglotAligner(&paramProvider);

  Eigen::Affine3d transformation;
  ASSERT_TRUE(parkinglotAligner.calculateTransformation(
      corridor, Eigen::Vector3d::Zero(), transformation));
  compare(Eigen::Vector3d(3, 3, 0), transformation * Eigen::Vector3d(sqrt(2), 0, 0));
  compare(Eigen::Vector3d(2, 3, 0),
          transformation * Eigen::Vector3d(sqrt(1.0 / 2), sqrt(1.0 / 2), 0));
  compare(Eigen::Vector3d(1, 1, 0), transformation * Eigen::Vector3d(-sqrt(2), 0, 0));

  Gate gate2(0, Eigen::Vector3d(-1, -1, 0), Eigen::Vector3d(-2, -2, 0));
  gates.clear();
  gates.push_back(gate2);
  corridor = DrivingCorridor(gates);
  parkinglotAligner.setStartPoint(Eigen::Vector3d::Zero());  // set new point
                                                             // for new
                                                             // transformation
                                                             // calculation
  ASSERT_TRUE(parkinglotAligner.calculateTransformation(
      corridor, Eigen::Vector3d::Zero(), transformation));
  compare(Eigen::Vector3d(-3, -3, 0), transformation * Eigen::Vector3d(sqrt(2), 0, 0));
  compare(Eigen::Vector3d(-2, -3, 0),
          transformation * Eigen::Vector3d(sqrt(1.0 / 2), sqrt(1.0 / 2), 0));
  compare(Eigen::Vector3d(-1, -1, 0), transformation * Eigen::Vector3d(-sqrt(2), 0, 0));

  Gate gate3(0, Eigen::Vector3d(1, 1, 0), Eigen::Vector3d(1, 3, 0));
  gates.clear();
  gates.push_back(gate3);
  corridor = DrivingCorridor(gates);
  parkinglotAligner.setStartPoint(Eigen::Vector3d::Zero());  // set new point
                                                             // for new
                                                             // transformation
                                                             // calculation
  ASSERT_TRUE(parkinglotAligner.calculateTransformation(
      corridor, Eigen::Vector3d::Zero(), transformation));
  compare(Eigen::Vector3d(1, 4, 0), transformation * Eigen::Vector3d(1, 0, 0));
  compare(Eigen::Vector3d(0, 3, 0), transformation * Eigen::Vector3d(0, 1, 0));
  compare(Eigen::Vector3d(0, 4, 0), transformation * Eigen::Vector3d(1, 1, 0));
}

TEST_F(ParkinglotAlignerTest, findCorrectGate) {
  const size_t numberOfGates = 10;
  Gate::GateList gates;
  for (size_t i = 0; i < numberOfGates; ++i) {
    Gate gate(0, Eigen::Vector3d(0, i, 0), Eigen::Vector3d(1, i, 0));
    gates.push_back(gate);
  }
  DrivingCorridor corridor(gates);
  DummyParameterHandler paramProvider;
  paramProvider.addParam(DISTANCE, 1.0);
  ParkinglotAligner parkinglotAligner(&paramProvider);

  Eigen::Affine3d transformation;

  for (size_t i = 0; i < numberOfGates; ++i) {
    parkinglotAligner.setStartPoint(Eigen::Vector3d(1, i, 0));
    ASSERT_TRUE(parkinglotAligner.calculateTransformation(
        corridor, Eigen::Vector3d::Zero(), transformation));
    compare(Eigen::Vector3d(1, 1 + i, 0), transformation * Eigen::Vector3d(0, 1, 0));
  }
  for (float f = 0; f < 3; f += 0.5f) {
    parkinglotAligner.setStartPoint(Eigen::Vector3d(f, 1, 0));
    ASSERT_TRUE(parkinglotAligner.calculateTransformation(
        corridor, Eigen::Vector3d::Zero(), transformation));
    compare(Eigen::Vector3d(1, 2, 0), transformation * Eigen::Vector3d(0, 1, 0));
  }
  for (float f = 0.6; f < 1.5; f += 0.1f) {
    parkinglotAligner.setStartPoint(Eigen::Vector3d(1, f, 0));
    ASSERT_TRUE(parkinglotAligner.calculateTransformation(
        corridor, Eigen::Vector3d::Zero(), transformation));
    compare(Eigen::Vector3d(1, 2, 0), transformation * Eigen::Vector3d(0, 1, 0));
  }
}

TEST_F(ParkinglotAlignerTest, orientateOnGateNearStartPosition) {
  const size_t numberOfGates = 10;
  Gate::GateList gates;
  // gates:
  //  | | | |
  //_ ->x direction
  Gate gate1(0, Eigen::Vector3d(0, 0, 0), Eigen::Vector3d(1, 0, 0));
  gates.push_back(gate1);
  Gate gate2(0, Eigen::Vector3d(0, 2, 0), Eigen::Vector3d(1, 1, 0));
  gates.push_back(gate2);
  for (size_t i = 2; i < numberOfGates; ++i) {
    Gate gate(0, Eigen::Vector3d(i, 2, 0), Eigen::Vector3d(i, 3, 0));
    gates.push_back(gate);
  }
  DrivingCorridor corridor(gates);
  DummyParameterHandler paramProvider;
  paramProvider.addParam(DISTANCE, 1.0);
  ParkinglotAligner parkinglotAligner(&paramProvider);

  Eigen::Affine3d transformation;
  parkinglotAligner.setStartPoint(Eigen::Vector3d(0, 0, 0));

  // calculate first Transformation which set the startposition to 1,0,0
  ASSERT_TRUE(parkinglotAligner.calculateTransformation(
      corridor, Eigen::Vector3d(0, 0, 0), transformation));
  compare(Eigen::Vector3d(1, 1, 0), transformation * Eigen::Vector3d(0, 1, 0));

  // change oreintation of the first gate
  corridor.at(0) = Gate(0, Eigen::Vector3d(0, 1, 0), Eigen::Vector3d(1, 0, 0));

  paramProvider.addParam(DISTANCE, 0.0);
  // calculate second Transformation which should not change the startposition
  // but use the right points of the gates near the startposition and near the
  // car for the orientation
  ASSERT_TRUE(parkinglotAligner.calculateTransformation(
      corridor, Eigen::Vector3d(1, 0.9, 0), transformation));
  compare(Eigen::Vector3d(1, 1, 0), transformation * Eigen::Vector3d(0, 1, 0));

  paramProvider.addParam(DISTANCE, 7.0);
  // calculate second Transformation which should not change the startposition
  // but change the orientation to the new orientation of the first gate:
  ASSERT_TRUE(parkinglotAligner.calculateTransformation(
      corridor, Eigen::Vector3d(1, 0.9, 0), transformation));
  compare(Eigen::Vector3d(1 + sqrt(1.0 / 2), sqrt(1.0 / 2), 0),
          transformation * Eigen::Vector3d(0, 1, 0));
}


TEST_F(ParkinglotAlignerTest, orientateOnGateNearCar) {
  const size_t numberOfGates = 10;
  Gate::GateList gates;
  // gates:
  //     _
  //    _
  //   _
  //  _
  // _
  //_ ->x direction
  for (size_t i = 0; i < numberOfGates; ++i) {
    Gate gate(0, Eigen::Vector3d(i, i, 0), Eigen::Vector3d(1 + i, i, 0));
    gates.push_back(gate);
  }
  DrivingCorridor corridor(gates);
  DummyParameterHandler paramProvider;
  paramProvider.addParam(DISTANCE, 1.0);
  ParkinglotAligner parkinglotAligner(&paramProvider);

  Eigen::Affine3d transformation;
  parkinglotAligner.setStartPoint(Eigen::Vector3d(0, 0, 0));

  // calculate first Transformation which use the gate near the startpoint for
  // the orientation
  ASSERT_TRUE(parkinglotAligner.calculateTransformation(
      corridor, Eigen::Vector3d(9.9, 9, 0), transformation));
  compare(Eigen::Vector3d(1, 1, 0), transformation * Eigen::Vector3d(0, 1, 0));

  // calculate second Transformation which use the right point of the gate near
  // the car for the orientation
  ASSERT_TRUE(parkinglotAligner.calculateTransformation(
      corridor, Eigen::Vector3d(9.9, 9, 0), transformation));
  compare(Eigen::Vector3d(1 + sqrt(1.0 / 2), sqrt(1.0 / 2), 0),
          transformation * Eigen::Vector3d(0, 1, 0));
}

// nothing to do here
int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
