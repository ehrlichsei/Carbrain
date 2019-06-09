#include <common/macros.h>

THIRD_PARTY_HEADERS_BEGIN
#include <gtest/gtest.h>
#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>
#include <boost/range/adaptor/indexed.hpp>
#include <navigation_msgs/RoadClosures.h>
#include <tf2_eigen/tf2_eigen.h>
THIRD_PARTY_HEADERS_END

#include "common/test/dummy_parameter_handler.h"

#include "navigation/driving_corridor.h"
#include "../src/collision_detection/collision_detection.h"

DISABLE_SUGGEST_OVERRIDE_WARNING

const ParameterString<double> PARAM_LEFT_LANE_BLOCK_DISTANCE_IN_FRONT_OF_OBSTACLE(
    "left_lane_block_distance_in_front_of_obstacle");
const ParameterString<double> PARAM_LEFT_LANE_BLOCK_DISTANCE_BEHIND_OBSTACLE(
    "left_lane_block_distance_behind_obstacle");
const ParameterString<double> PARAM_LEFT_LANE_BLOCK_DISTANCE_IN_FRONT_OF_ROAD_CLOSURE(
    "left_lane_block_distance_in_front_of_road_closure");
const ParameterString<double> PARAM_LEFT_LANE_BLOCK_DISTANCE_BEHIND_ROAD_CLOSURE(
    "left_lane_block_distance_behind_road_closure");
const ParameterString<double> PARAM_ROAD_CLOSURE_SAFETY_MARGIN(
    "road_closure_safety_margin");
const ParameterString<double> PARAM_OBSTACLE_SAFETY_MARGIN(
    "obstacle_safety_margin");
const ParameterString<double> PARAM_BLOCK_DISTANCE_CAR_IN_ROAD_CLOSURE_AREA(
    "block_distance_car_in_road_closure_area");
const ParameterString<bool> PARAM_BLOCK_LEFT_LANE("block_left_lane");
const ParameterString<bool> PARAM_AVOID_OBSTACLES("avoid_obstacles");

DummyParameterHandler makeParameterHandler(double roadClosureSafetyMargin = 0,
                                           double dInFrontOf = 0,
                                           double dBehind = 0,
                                           double blockDistCarInRoadClosureArea = 0) {
  DummyParameterHandler parameter_handler;
  parameter_handler.addParam(PARAM_ROAD_CLOSURE_SAFETY_MARGIN, roadClosureSafetyMargin);
  parameter_handler.addParam(PARAM_OBSTACLE_SAFETY_MARGIN, 0.0);
  parameter_handler.addParam(PARAM_LEFT_LANE_BLOCK_DISTANCE_IN_FRONT_OF_OBSTACLE, dInFrontOf);
  parameter_handler.addParam(PARAM_LEFT_LANE_BLOCK_DISTANCE_BEHIND_OBSTACLE, dBehind);
  parameter_handler.addParam(
      PARAM_LEFT_LANE_BLOCK_DISTANCE_IN_FRONT_OF_ROAD_CLOSURE, dInFrontOf);
  parameter_handler.addParam(PARAM_LEFT_LANE_BLOCK_DISTANCE_BEHIND_ROAD_CLOSURE, dBehind);
  parameter_handler.addParam(PARAM_BLOCK_DISTANCE_CAR_IN_ROAD_CLOSURE_AREA,
                             blockDistCarInRoadClosureArea);
  parameter_handler.addParam(PARAM_BLOCK_LEFT_LANE, true);
  parameter_handler.addParam(PARAM_AVOID_OBSTACLES, true);
  return parameter_handler;
}

class CollisionDetectionTest : public ::testing::Test {
 public:
  navigation_msgs::RoadClosuresConstPtr createRoadClosures(
      const std::vector<std::vector<Eigen::Vector3d>>& road_closure_points) {
    boost::shared_ptr<navigation_msgs::RoadClosures> message =
        boost::make_shared<navigation_msgs::RoadClosures>();
    message->sub_messages.reserve(road_closure_points.size());
    for (const auto points : road_closure_points | boost::adaptors::indexed(0)) {
      navigation_msgs::RoadClosure road_closure;
      road_closure.hull_polygon.reserve(points.value().size());
      for (const auto& p : points.value()) {
        road_closure.hull_polygon.push_back(tf2::toMsg(p));
      }
      road_closure.id = points.index();
      message->sub_messages.push_back(road_closure);
    }

    return std::move(message);
  }

  template <class C1, class C2>
  void compare(const C1& c1, const C2 c2) {
    ASSERT_EQ(c1.size(), c2.size());
    for (size_t i = 0; i < c1.size(); ++i) {
      compare(c1.at(i), c2.at(i));
    }
  }
  void compare(const Gate& expected, const Gate& result) {
    compare(expected.getRightPole(), result.getRightPole());
    compare(expected.getLeftPole(), result.getLeftPole());
  }
  void compare(const Eigen::Vector3d& expected, const Eigen::Vector3d& result) {
    const double epsilon = 1e-10;
    ASSERT_NEAR(expected.x(), result.x(), epsilon);
    ASSERT_NEAR(expected.y(), result.y(), epsilon);
    ASSERT_NEAR(expected.z(), result.z(), epsilon);
  }
};

TEST_F(CollisionDetectionTest, cropsCorrectAroundRoadClosures) {
  Gate::GateList gates;
  gates.reserve(20);
  for (size_t i = 0; i < 10; ++i) {
    gates.emplace_back(0, Eigen::Vector3d(i, 2, 0), Eigen::Vector3d(i, 0, 0));
  }
  for (size_t i = 10; i < 20; ++i) {
    gates.emplace_back(0, Eigen::Vector3d(10, i - 10, 0), Eigen::Vector3d(12, i - 10, 0));
  }

  DrivingCorridor corridor(gates);
  auto paramProvider = makeParameterHandler();
  CollisionDetection collisionDetection(&paramProvider);
  const navigation_msgs::RoadClosuresConstPtr roadClosures =
      createRoadClosures({{{0, 0, 0}, {2, 1, 0}, {5, 1, 0}, {7, 0, 0}},
                          {{12, 0, 0}, {11, 2, 0}, {11, 5, 0}, {12, 7, 0}}});
  collisionDetection.cropAroundRoadClosures(corridor, roadClosures);

  gates.at(1).setRightPole(gates.at(1).getRightPole() + Eigen::Vector3d(0, 0.5, 0));
  gates.at(2).setRightPole(gates.at(2).getRightPole() + Eigen::Vector3d(0, 1, 0));
  gates.at(3).setRightPole(gates.at(3).getRightPole() + Eigen::Vector3d(0, 1, 0));
  gates.at(4).setRightPole(gates.at(4).getRightPole() + Eigen::Vector3d(0, 1, 0));
  gates.at(5).setRightPole(gates.at(5).getRightPole() + Eigen::Vector3d(0, 1, 0));
  gates.at(6).setRightPole(gates.at(6).getRightPole() + Eigen::Vector3d(0, 0.5, 0));

  gates.at(11).setRightPole(gates.at(11).getRightPole() + Eigen::Vector3d(-0.5, 0, 0));
  gates.at(12).setRightPole(gates.at(12).getRightPole() + Eigen::Vector3d(-1, 0, 0));
  gates.at(13).setRightPole(gates.at(13).getRightPole() + Eigen::Vector3d(-1, 0, 0));
  gates.at(14).setRightPole(gates.at(14).getRightPole() + Eigen::Vector3d(-1, 0, 0));
  gates.at(15).setRightPole(gates.at(15).getRightPole() + Eigen::Vector3d(-1, 0, 0));
  gates.at(16).setRightPole(gates.at(16).getRightPole() + Eigen::Vector3d(-0.5, 0, 0));

  compare(gates, corridor);
}

TEST_F(CollisionDetectionTest, respectsSafetyMargin) {
  const size_t numberOfGates = 10;
  Gate::GateList gates;
  for (size_t i = 0; i < numberOfGates; ++i) {
    Gate gate(0, Eigen::Vector3d(i, 2, 0), Eigen::Vector3d(i, 0, 0));
    gates.push_back(gate);
  }
  DrivingCorridor corridor(gates);
  auto paramProvider = makeParameterHandler(0.5, 0, 0, 0);
  CollisionDetection collisionDetection(&paramProvider);
  const navigation_msgs::RoadClosuresConstPtr roadClosure =
      createRoadClosures({{{0.5, 0, 0}, {2.5, 0.5, 0}, {4.5, 0.5, 0}, {6.5, 0, 0}}});

  collisionDetection.cropAroundRoadClosures(corridor, roadClosure);

  gates.at(1).setRightPole(gates.at(1).getRightPole() + Eigen::Vector3d(0, 0.5, 0));
  gates.at(2).setRightPole(gates.at(2).getRightPole() + Eigen::Vector3d(0, 1, 0));
  gates.at(3).setRightPole(gates.at(3).getRightPole() + Eigen::Vector3d(0, 1, 0));
  gates.at(4).setRightPole(gates.at(4).getRightPole() + Eigen::Vector3d(0, 1, 0));
  gates.at(5).setRightPole(gates.at(5).getRightPole() + Eigen::Vector3d(0, 1, 0));
  gates.at(6).setRightPole(gates.at(6).getRightPole() + Eigen::Vector3d(0, 0.5, 0));

  compare(gates, corridor);
}

TEST_F(CollisionDetectionTest, respectsBlockDistances) {
  const size_t numberOfGates = 15;
  Gate::GateList gates;
  for (size_t i = 0; i < numberOfGates; ++i) {
    Gate gate(0, Eigen::Vector3d(i, 2, 0), Eigen::Vector3d(i, 0, 0));
    gate.setLeftPole(Eigen::Vector3d(i, 1, 0));
    gates.push_back(gate);
  }
  DrivingCorridor corridor(gates);
  auto paramProvider = makeParameterHandler(0, 1.01, 1.01, 0);
  CollisionDetection collisionDetection(&paramProvider);
  const navigation_msgs::RoadClosuresConstPtr roadClosure =
      createRoadClosures({{{2, 0, 0}, {4, 1, 0}, {7, 1, 0}, {9, 0, 0}}});

  collisionDetection.cropAroundRoadClosures(corridor, roadClosure);

  gates.at(3).setRightPole(gates.at(3).getRightPole() + Eigen::Vector3d(0, 0.5, 0));
  gates.at(4).setRightPole(gates.at(4).getRightPole() + Eigen::Vector3d(0, 1, 0));
  gates.at(5).setRightPole(gates.at(5).getRightPole() + Eigen::Vector3d(0, 1, 0));
  gates.at(6).setRightPole(gates.at(6).getRightPole() + Eigen::Vector3d(0, 1, 0));
  gates.at(7).setRightPole(gates.at(7).getRightPole() + Eigen::Vector3d(0, 1, 0));
  gates.at(8).setRightPole(gates.at(8).getRightPole() + Eigen::Vector3d(0, 0.5, 0));

  for (size_t i = 1; i < 11; ++i) {
    gates.at(i).setLeftPole(Eigen::Vector3d(i, 2, 0));
  }

  compare(gates, corridor);
}

TEST_F(CollisionDetectionTest, blocksLeftLaneIfCarOnLeftSideOfRoadClosure) {
  // distance between two gates
  const double a = 0.11 * CollisionDetection::LENGTH_OF_LEFT_LANE_BLOCKED;
  Gate::GateList gates;
  gates.reserve(20);
  for (size_t i = 0; i < 20; ++i) {
    gates.emplace_back(0, Eigen::Vector3d(i * a, 2 * a, 0), Eigen::Vector3d(i * a, 0, 0));
  }
  DrivingCorridor corridor(gates);
  auto paramProvider = makeParameterHandler(0, 0, 0, a * 1.99);
  CollisionDetection collisionDetection(&paramProvider);
  navigation_msgs::DrivePastNextRoadClosureRequest request;
  request.drive_past_next_road_closure = false;
  navigation_msgs::DrivePastNextRoadClosureResponse response;
  collisionDetection.setDrivePastNextRoadClosure(request, response);

  // second RoadClosure to test if the function detects that the other
  // RoadClosure commes next in front of the car
  const navigation_msgs::RoadClosuresConstPtr roadClosures = createRoadClosures(
      {{{19.1 * a, 1.1 * a, 0}, {19.1 * a, 1.1 * a, 0}, {19.1 * a, 1.1 * a, 0}, {19.1 * a, 1.1 * a, 0}},
       {{12 * a, 0, 0}, {14 * a, a, 0}, {17 * a, a, 0}, {19 * a, 0, 0}}});

  collisionDetection.cropAroundRoadClosures(corridor, roadClosures);

  gates.at(13).setRightPole(gates.at(13).getRightPole() + Eigen::Vector3d(0, 0.5 * a, 0));
  gates.at(14).setRightPole(gates.at(14).getRightPole() + Eigen::Vector3d(0, a, 0));
  gates.at(15).setRightPole(gates.at(15).getRightPole() + Eigen::Vector3d(0, a, 0));
  gates.at(16).setRightPole(gates.at(16).getRightPole() + Eigen::Vector3d(0, a, 0));
  gates.at(17).setRightPole(gates.at(17).getRightPole() + Eigen::Vector3d(0, a, 0));
  gates.at(18).setRightPole(gates.at(18).getRightPole() + Eigen::Vector3d(0, 0.5 * a, 0));

  for (size_t i = 1; i < 11; ++i) {
    gates.at(i).setLeftPole(gates.at(i).getLaneCenter());
  }

  compare(gates, corridor);
}

TEST_F(CollisionDetectionTest, cropsCorrectAroundRoadClosuresOnLeftLane) {
  Gate::GateList gates;
  gates.reserve(20);
  for (size_t i = 0; i < 10; ++i) {
    gates.emplace_back(0, Eigen::Vector3d(i, 2, 0), Eigen::Vector3d(i, 0, 0));
  }
  for (size_t i = 10; i < 20; ++i) {
    gates.emplace_back(0, Eigen::Vector3d(10, i - 10, 0), Eigen::Vector3d(12, i - 10, 0));
  }
  DrivingCorridor corridor(gates);
  auto paramProvider = makeParameterHandler();
  CollisionDetection collisionDetection(&paramProvider);
  const navigation_msgs::RoadClosuresConstPtr roadClosures =
      createRoadClosures({{{0, 2, 0}, {2, 1, 0}, {5, 1, 0}, {7, 2, 0}},
                          {{10, 0, 0}, {11, 2, 0}, {11, 5, 0}, {10, 7, 0}}});

  collisionDetection.cropAroundRoadClosures(corridor, roadClosures);

  gates.at(1).setLeftPole(gates.at(1).getLeftPole() - Eigen::Vector3d(0, 0.5, 0));
  gates.at(2).setLeftPole(gates.at(2).getLeftPole() - Eigen::Vector3d(0, 1, 0));
  gates.at(3).setLeftPole(gates.at(3).getLeftPole() - Eigen::Vector3d(0, 1, 0));
  gates.at(4).setLeftPole(gates.at(4).getLeftPole() - Eigen::Vector3d(0, 1, 0));
  gates.at(5).setLeftPole(gates.at(5).getLeftPole() - Eigen::Vector3d(0, 1, 0));
  gates.at(6).setLeftPole(gates.at(6).getLeftPole() - Eigen::Vector3d(0, 0.5, 0));

  gates.at(11).setLeftPole(gates.at(11).getLeftPole() - Eigen::Vector3d(-0.5, 0, 0));
  gates.at(12).setLeftPole(gates.at(12).getLeftPole() - Eigen::Vector3d(-1, 0, 0));
  gates.at(13).setLeftPole(gates.at(13).getLeftPole() - Eigen::Vector3d(-1, 0, 0));
  gates.at(14).setLeftPole(gates.at(14).getLeftPole() - Eigen::Vector3d(-1, 0, 0));
  gates.at(15).setLeftPole(gates.at(15).getLeftPole() - Eigen::Vector3d(-1, 0, 0));
  gates.at(16).setLeftPole(gates.at(16).getLeftPole() - Eigen::Vector3d(-0.5, 0, 0));

  compare(gates, corridor);
}

TEST_F(CollisionDetectionTest, corridorIsRightLaneInNoPassingZone) {
  Gate::GateList gates;
  gates.reserve(30);
  for (size_t i = 0; i < 30; ++i) {
    gates.emplace_back(0, Eigen::Vector3d(i, 2, 0), Eigen::Vector3d(i, 0, 0));
  }
  DrivingCorridor corridor(gates);
  auto paramProvider = makeParameterHandler(0, 0.1, 0.1, 0);
  CollisionDetection collisionDetection(&paramProvider);

  const navigation::Obstacle::ObstacleList obstacles = {
      {Eigen::Affine3d(Eigen::Translation3d(1.55, 0.45, 0)), Eigen::Vector3d(1.1, 0.9, 0), 0, 0},
      {Eigen::Affine3d(Eigen::Translation3d(7.55, 0.45, 0)), Eigen::Vector3d(1.1, 0.9, 0), 0, 0},
      {Eigen::Affine3d(Eigen::Translation3d(23.55, 0.45, 0)), Eigen::Vector3d(1.1, 0.9, 0), 0, 0}};

  navigation_msgs::SetRespectNoPassingZones respectNoPassingZone;
  respectNoPassingZone.request.respect_no_passing_zones = true;
  NoPassingZone no_passing_zone;
  no_passing_zone.setStart(Eigen::Vector3d(5, 0, 0));
  no_passing_zone.setEnd(Eigen::Vector3d(15, 0, 0));
  std::vector<NoPassingZone> zones = {no_passing_zone};
  collisionDetection.setRespectNoPassingZones(respectNoPassingZone.request,
                                              respectNoPassingZone.response);
  collisionDetection.setNoPassingZones(zones);
  collisionDetection.cropAroundObstacles(corridor, obstacles);


  for (size_t i = 0; i < gates.size(); ++i) {
    if ((i < 23 || i > 24) && (i < 1 || i > 2)) {
      gates.at(i).setLeftPole(gates.at(i).getCenter());
    } else {
      gates.at(i).setRightPole(gates.at(i).getRightPole() + Eigen::Vector3d(0, 0.9, 0));
    }
  }

  compare(gates, corridor);
}

// nothing to do here
int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
