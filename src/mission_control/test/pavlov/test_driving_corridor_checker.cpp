#include <common/macros.h>

THIRD_PARTY_HEADERS_BEGIN
#include <gtest/gtest.h>
#include <tf2/convert.h>
THIRD_PARTY_HEADERS_END

#include "navigation/driving_corridor.h"
#include "navigation/gate.h"

#include "../../src/pavlov/node_helper/driving_corridor_checker.h"

DISABLE_SUGGEST_OVERRIDE_WARNING

// Test fixture
class DrivingCorridorCheckerTest : public ::testing::Test {
 protected:
  DrivingCorridorChecker testee;
  navigation_msgs::DrivingCorridor corridor_msg;
  navigation_msgs::Obstacles all_obstacles_msg;
  navigation_msgs::Obstacles no_obstacles_msg;

 public:
  DrivingCorridorCheckerTest()
      : testee(),
        corridor_msg(createCorridorMsg()),
        all_obstacles_msg(createAllObstaclesMsg()),
        no_obstacles_msg(navigation_msgs::Obstacles()) {
    // Simulate path.
    testee.handleFullCorridor(std::make_unique<navigation_msgs::DrivingCorridor>(corridor_msg));

    // Simulate obstacles.
    testee.handleObstacles(all_obstacles_msg);
  }

  static navigation_msgs::DrivingCorridor createCorridorMsg() {
    Gate::GateList gates;
    for (size_t i = 0; i < 9; i++) {
      Eigen::Vector3d right(i, 0, 0);
      Eigen::Vector3d left(i, 3, 0);
      gates.push_back(Gate(i, left, right));
    }

    return DrivingCorridor(gates).toMessage();
  }

  static navigation_msgs::Obstacles createAllObstaclesMsg() {
    auto obs_a_msg = createObstacleMsg(Eigen::Vector3d(3, 2, 0));
    auto obs_b_msg = createObstacleMsg(Eigen::Vector3d(5, 1, 0));

    navigation_msgs::Obstacles obstacles_msg;
    obstacles_msg.sub_messages.push_back(obs_a_msg);
    obstacles_msg.sub_messages.push_back(obs_b_msg);

    return obstacles_msg;
  }

  static navigation_msgs::Obstacle createObstacleMsg(Eigen::Vector3d pos) {
    navigation_msgs::Obstacle obs;
    obs.pose.position.x = pos[0];
    obs.pose.position.y = pos[1];
    obs.pose.position.z = pos[2];

    return obs;
  }
};


// XXXXXXXXX-
// --A------
// ----B----
// XXXXXXXXX-
// 123456789

TEST_F(DrivingCorridorCheckerTest, hasObstacleOnRightLane_NoObstaclesTracked_ReturnsFalse) {
  testee.handleObstacles(no_obstacles_msg);
  double a = 0, b = 0;
  bool hasObstacles = testee.hasObstacleOnRightLane(Eigen::Vector3d(4, 0, 0), 2, a, b);

  ASSERT_FALSE(hasObstacles);
}

TEST_F(DrivingCorridorCheckerTest, hasObstacleOnRightLane_NoObstaclesInArea_ReturnsFalse) {
  double a = 0, b = 0;
  bool hasObstacles = testee.hasObstacleOnRightLane(Eigen::Vector3d(0, 0, 0), 3, a, b);

  ASSERT_FALSE(hasObstacles);
}

TEST_F(DrivingCorridorCheckerTest, hasObstacleOnRightLane_ObstacleOnRightLane_ReturnsTrue) {
  double a = 0, b = 0;
  bool hasObstacles = testee.hasObstacleOnRightLane(Eigen::Vector3d(0, 0, 0), 6, a, b);

  ASSERT_TRUE(hasObstacles);
}

TEST_F(DrivingCorridorCheckerTest, hasObstacleOnRightLane_ObstacleOnLeftLane_ReturnsFalse) {
  double a = 0, b = 0;
  bool hasObstacles = testee.hasObstacleOnRightLane(Eigen::Vector3d(0, 0, 0), 4, a, b);

  ASSERT_FALSE(hasObstacles);
}

TEST_F(DrivingCorridorCheckerTest, hasObstacleOnRightLaneBehindStart_ReturnsFalse) {
  double a = 0, b = 0;
  bool hasObstacles =
      testee.hasObstacleOnRightLane(Eigen::Vector3d(5.1, 0, 0), 10, a, b);

  ASSERT_FALSE(hasObstacles);
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
