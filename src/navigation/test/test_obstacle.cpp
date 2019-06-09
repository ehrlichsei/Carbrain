#include <common/macros.h>

THIRD_PARTY_HEADERS_BEGIN
#include <gtest/gtest.h>
#include <boost/range/algorithm/for_each.hpp>
#include "navigation_msgs/Obstacles.h"
THIRD_PARTY_HEADERS_END

#include "navigation/obstacle.h"
#include "common/tf2_eigen_addon.h"

DISABLE_SUGGEST_OVERRIDE_WARNING

// execute tests with one of the following command:
// catkin_make run_tests_navigation
// catkin_make run_tests_navigation_gtest_obstacle_test

namespace Eigen {

void PrintTo(const Eigen::Vector3d& vector, ::std::ostream* os) {
  *os << "\n" << vector;
}

}  // namespace Eigen

namespace {
using namespace navigation;

bool isApproxVector(const Eigen::Vector3d& expected, const Eigen::Vector3d& actual) {
  return expected.isApprox(actual);
}

class ObstacleTest : public ::testing::Test {
 public:
  static const Eigen::Vector3d CENTER;
  static const Eigen::Vector3d FRONT_LEFT_CORNER;
  static const Eigen::Vector3d BACK_LEFT_CORNER;
  static const Eigen::Vector3d BACK_RIGHT_CORNER;
  static const Eigen::Vector3d FRONT_RIGHT_CORNER;
  static const std::vector<Eigen::Vector3d> CORNERS;
  static const Eigen::Vector3d SCALE;
  static const Eigen::AngleAxisd ROTATION;

 protected:
  static Eigen::Affine3d createObstaclePose() {
    Eigen::Affine3d pose;
    pose.translation() << CENTER;
    pose.linear().setIdentity();
    return pose;
  }

  static Eigen::Affine3d createRotatedObstaclePose() {
    return createObstaclePose() * ROTATION;
  }

  static Obstacle createObstacle() {
    return Obstacle(createObstaclePose(), SCALE, 0, 0);
  }

  static Obstacle createRotatedObstacle() {
    return Obstacle(createRotatedObstaclePose(), SCALE, 0, 0);
  }
};

//     y
//     ^
//  6 -|  +---------------+
//     |  | Test Obstacle |
//  4 -|  |       +       |
//     |  |               |
//  2 -|  +---------------+
//     |
//     |--+-------+-------+--> x
//        |       |       |
//        1       5       9
//
const Eigen::Vector3d ObstacleTest::CENTER(5, 4, 0);
const Eigen::Vector3d ObstacleTest::FRONT_LEFT_CORNER(9, 6, 0);
const Eigen::Vector3d ObstacleTest::BACK_LEFT_CORNER(1, 6, 0);
const Eigen::Vector3d ObstacleTest::BACK_RIGHT_CORNER(1, 2, 0);
const Eigen::Vector3d ObstacleTest::FRONT_RIGHT_CORNER(9, 2, 0);
const std::vector<Eigen::Vector3d> ObstacleTest::CORNERS{
    FRONT_LEFT_CORNER, BACK_LEFT_CORNER, BACK_RIGHT_CORNER, FRONT_RIGHT_CORNER};
const Eigen::Vector3d ObstacleTest::SCALE(ObstacleTest::FRONT_LEFT_CORNER -
                                          ObstacleTest::BACK_RIGHT_CORNER);
const Eigen::AngleAxisd ObstacleTest::ROTATION(0.5 * M_PI, Eigen::Vector3d::UnitZ());


TEST_F(ObstacleTest, getCenter) {
  const Obstacle obstacle = createObstacle();
  ASSERT_EQ(CENTER, obstacle.getCenter());
}

TEST_F(ObstacleTest, getCorners) {
  const Obstacle obstacle = createObstacle();
  auto corners = obstacle.getCorners();
  ASSERT_EQ(corners.size(), 4);
  // check if the corners are a rectangle and ordered correctly e.g. 0-1-2-3 or 2-1-0-3
  ASSERT_EQ(corners[2] - corners[0], corners[1] - corners[0] + corners[3] - corners[0]);
}

TEST_F(ObstacleTest, approximateDistanceFromCorner) {
  const Obstacle obstacle = createObstacle();
  boost::range::for_each(obstacle.getCorners(), [&obstacle](Eigen::Vector3d p) {
    EXPECT_EQ(0, obstacle.approximateProjectedDistance(p));
  });
}

TEST_F(ObstacleTest, approximateDistanceFromPoint) {
  const Obstacle obstacle = createObstacle();
  const Eigen::Vector3d point(6, 8, 0);
  ASSERT_EQ((point - CENTER).norm() - SCALE.norm() / 2, obstacle.approximateProjectedDistance(point));
}

TEST_F(ObstacleTest, extendByZero) {
  Obstacle obstacle = createObstacle();
  auto before = obstacle.getCorners();
  obstacle.extendBy(0);
  const auto after = obstacle.getCorners();
  for (size_t i = 0; i < before.size(); i++) {
    ASSERT_PRED2(isApproxVector, before[i], after[i]);
  }
}

TEST_F(ObstacleTest, extendByOne) {
  Obstacle obstacle = createObstacle();
  const auto before = obstacle.getCorners();
  obstacle.extendBy(1);
  const auto after = obstacle.getCorners();
  for (size_t i = 0; i < before.size(); i++) {
    // Check displacement
    ASSERT_EQ(std::sqrt(2.), (after[i] - before[i]).norm());
    // Check if direction is actually away from center
    ASSERT_GT((before[i] - CENTER).dot(after[i] - before[i]), 0);
  }

}

TEST_F(ObstacleTest, rotatedObstacle) {
  const Obstacle obstacle_a = createObstacle();
  const Obstacle obstacle_b = createRotatedObstacle();
  auto corners_a = obstacle_a.getCorners();
  auto corners_b = obstacle_b.getCorners();
  ASSERT_PRED2(isApproxVector, obstacle_a.getCenter(), obstacle_b.getCenter());
  for (size_t i = 0; i < corners_a.size(); i++) {
    ASSERT_PRED2(isApproxVector,
                 ROTATION * (corners_a[i] - obstacle_a.getCenter()) + obstacle_a.getCenter(),
                 corners_b[i]);
  }
}

TEST_F(ObstacleTest, fromMessage) {
  navigation_msgs::Obstacle obstacle_msg;
  obstacle_msg.pose = tf2::toMsg(createObstaclePose());
  obstacle_msg.scale = tf2::toVector3Msg(SCALE);
  const Obstacle obstacle = Obstacle::fromMessage(obstacle_msg);
  const Obstacle expected = createObstacle();
  ASSERT_EQ(expected.getCenter(), obstacle.getCenter());
  ASSERT_EQ(expected.getScale(), obstacle.getScale());
}

}  // namespace

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
