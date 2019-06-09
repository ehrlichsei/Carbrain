#include <common/macros.h>

THIRD_PARTY_HEADERS_BEGIN
#include <string>
#include <ros/ros.h>
#include <gtest/gtest.h>
#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Path.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_eigen/tf2_eigen.h>
THIRD_PARTY_HEADERS_END

#include <navigation_msgs/ReverseOutOfParkingSpot.h>
#include "common/tf2_eigen_addon.h"
#include "common/path_conversion.h"
#include "navigation/driving_corridor.h"
#include "navigation/gate.h"

DISABLE_SUGGEST_OVERRIDE_WARNING

const double GATE_WIDTH(2.0);
const double GATE_DIST(0.5);  // Distance between Gates
const int GATE_COUNT(20);  // Gates per basic corridor element (straight, curve)
const ros::Duration TIME_OUT(1);

class PathPlanningTest : public testing::Test {
 protected:
  ros::NodeHandle nh;
  ros::Publisher car_corridor_publisher;
  ros::Subscriber safe_target_path_subscriber;
  ros::ServiceClient client = nh.serviceClient<navigation_msgs::ReverseOutOfParkingSpot>(
      "/navigation/path_planning/reverse_out_of_parking_spot_service");
  tf2_ros::TransformBroadcaster tf2_broadcaster;
  nav_msgs::Path::ConstPtr latest_path;
  bool callback_called = false;

  void expectPublisher(ros::Subscriber& subscriber) {
    ros::Rate spin_rate(10);
    ros::Time t_start = ros::Time::now();
    while (ros::Time::now() - t_start < TIME_OUT) {
      ros::spinOnce();
      if (subscriber.getNumPublishers() != 0) {
        return;
      }
      spin_rate.sleep();
    }
    return;
  }

  void expectSubscription(ros::Publisher& publisher) {
    ros::Rate spin_rate(10);
    ros::Time t_start = ros::Time::now();
    while (ros::Time::now() - t_start < TIME_OUT) {
      ros::spinOnce();
      if (publisher.getNumSubscribers() != 0) {
        return;
      }
      spin_rate.sleep();
    }
    return;
  }

  void publishWorldPathTransform() {
    tf2::Stamped<Eigen::Affine3d> path_to_world_transform_stamped(
        Eigen::Affine3d::Identity(), ros::Time::now(), "world");
    geometry_msgs::TransformStamped path_to_world_transform_msg =
        tf2::toMsg(path_to_world_transform_stamped, "path");
    tf2_broadcaster.sendTransform(path_to_world_transform_msg);
  }

  void publishCorridorWaitAndSpinOnce(const DrivingCorridor& corridor) {
    publishWorldPathTransform();

    navigation_msgs::DrivingCorridor corridor_msg = corridor.toMessage();
    corridor_msg.header.stamp = ros::Time::now();
    corridor_msg.header.frame_id = "world";
    callback_called = false;
    car_corridor_publisher.publish(corridor_msg);
    // Wait for the message to be handled from test nodes.
    // Spin once for own callback.
    // The following tests depend on it.
    // TODO duration reasonable or use different approch?
    ros::Rate spin_rate(30);
    for (size_t i = 0; i < 100 && !callback_called; i++) {
      ros::spinOnce();
      spin_rate.sleep();
    }
    ASSERT_TRUE(callback_called);
  }

  static Gate createFirstGate(int id) {
    // Arbitrary Points with (z=0), uses GATE_WIDTH
    Eigen::Vector3d leftBoundary(1.1, 2.2, 0.);
    Eigen::Vector3d rightBoundary(
        leftBoundary + Eigen::Vector3d(3.3, 4.4, 0.0).normalized() * GATE_WIDTH);
    // different id for each test case avoiding interference
    return Gate(id, leftBoundary, rightBoundary);
  }

  static Gate createNextGate(const Gate& prev_gate, double distance, double rad) {
    Gate next_gate(prev_gate);
    // Moves half gate distance forward
    // Turns around gate mid (of gate boundaries, not poles)
    // Moves half gate distance forward
    next_gate.transform(prev_gate.getTransformationFromGateFrame() *
                        Eigen::Translation3d(GATE_WIDTH * 0.5, distance / 2., 0) *
                        Eigen::AngleAxisd(rad, Eigen::Vector3d::UnitZ()) *
                        Eigen::Translation3d(-GATE_WIDTH * 0.5, distance / 2., 0) *
                        prev_gate.getTransformationToGateFrame());
    return next_gate;
  }

  static void appendStraight(DrivingCorridor& corridor) {
    for (size_t i = 0; i < GATE_COUNT; i++) {
      corridor.push_back(createNextGate(corridor.back(), GATE_DIST, 0.));
    }
  }

  // 90 degrees curve right
  static void appendCurveRight(DrivingCorridor& corridor) {
    for (size_t i = 0; i < GATE_COUNT; i++) {
      corridor.push_back(createNextGate(corridor.back(), GATE_DIST, 0.5 * M_PI / GATE_COUNT));
    }
  }

  // 90 degrees curve left
  static void appendCurveLeft(DrivingCorridor& corridor) {
    for (size_t i = 0; i < GATE_COUNT; i++) {
      corridor.push_back(createNextGate(corridor.back(), GATE_DIST, -0.5 * M_PI / GATE_COUNT));
    }
  }

 public:
  PathPlanningTest() : nh("/navigation/path_planning_test") {
    car_corridor_publisher = nh.advertise<navigation_msgs::DrivingCorridor>(
        "/navigation/collision_detection/car_corridor", 100);
    ros::Duration(0.1).sleep();
    safe_target_path_subscriber = nh.subscribe(
        "/navigation/path_planning/safe_target_path", 100, &PathPlanningTest::handleSafeTargetPath, this);

    // Wait for the testee node to publish and subscribe.
    // copied from test_pavlov.cpp
    expectPublisher(safe_target_path_subscriber);
    expectSubscription(car_corridor_publisher);
  }

  ~PathPlanningTest() {}

  void handleSafeTargetPath(const nav_msgs::Path::ConstPtr& msg) {
    latest_path = msg;
    callback_called = true;
  }
};

bool pointOnGate(const Gate& gate, const Eigen::Vector3d& point) {
  const double length = gate.getLaneLineSegment().getLength();
  const double eps = 1e-5;

  return std::abs((gate.getTransformationToGateFrame() * point).y()) < eps &&
         (gate.contains(gate.getParam<Gate::LEFT>() +
                        (gate.getTransformationToGateFrame() * point).x() / length) ||
          gate.contains(gate.getParam<Gate::LEFT>() +
                        (gate.getTransformationToGateFrame() * point).x() / length - eps) ||
          gate.contains(gate.getParam<Gate::LEFT>() +
                        (gate.getTransformationToGateFrame() * point).x() / length + eps));
}

TEST_F(PathPlanningTest, SubscribesAndPublishesCorrectly) {
  EXPECT_NE(safe_target_path_subscriber.getNumPublishers(), 0);
  EXPECT_NE(car_corridor_publisher.getNumSubscribers(), 0);
}

TEST_F(PathPlanningTest, StraightCorridorLeadsToStraightPath) {
  // Straight Corridor (with straight poles)
  DrivingCorridor corridor;
  corridor.push_back(createFirstGate(1));
  appendStraight(corridor);
  publishCorridorWaitAndSpinOnce(corridor);

  ASSERT_TRUE(latest_path != NULL) << "No path recieved.";
  std::vector<Eigen::Vector3d> path;
  common::msg_helper::fromMsg(*latest_path, path);

  // Transform points to connecting vectors
  std::transform(
      ++path.begin(), path.end(), path.begin(), path.begin(), std::minus<Eigen::Vector3d>());
  path.pop_back();

  const double EPSILON = 1e-6;
  // Check angles
  // length(A) * length(B) * cos(angle) = abs(dot(A,B))
  // rearranged with angle = 0 or 180deg:
  // dot(A,A) * dot(B,B) - dot(A,B) * dot(A,B) = 0
  Eigen::Vector3d a, b;
  auto it1 = path.begin();
  auto it2 = ++path.begin();
  for (; it2 != path.end(); it1++, it2++) {
    a = *it1;
    b = *it2;
    ASSERT_NEAR(a.dot(a) * b.dot(b) - a.dot(b) * a.dot(b), 0., EPSILON)
        << "Path not straight. Angle greater than " << EPSILON;
  }
}

TEST_F(PathPlanningTest, AllPointsOfPathOnGates) {
  // Arbitrary Corridor
  DrivingCorridor corridor;
  corridor.push_back(createFirstGate(2));
  appendCurveRight(corridor);
  appendCurveRight(corridor);
  appendCurveLeft(corridor);
  appendCurveLeft(corridor);

  // Unchanged (Poles at the gate boundaries)

  publishCorridorWaitAndSpinOnce(corridor);
  ASSERT_TRUE(latest_path != NULL) << "No path recieved.";
  std::vector<Eigen::Vector3d> path;
  common::msg_helper::fromMsg(*latest_path, path);

  // Test
  EXPECT_EQ(corridor.size(), path.size());
  for (size_t i = 0; i < corridor.size(); ++i) {
    EXPECT_TRUE(pointOnGate(corridor.at(i), path[i]));
  }
}

TEST_F(PathPlanningTest, AllPointsOfPathOnGatesUsingPoles) {
  // Arbitrary Corridor
  DrivingCorridor corridor;
  corridor.push_back(createFirstGate(3));
  appendStraight(corridor);

  // Shrink gates to 10% at varying positions (triangular)
  const double INTERVALS = 3.f;
  const double RATIO = 0.2f;
  /*
   * Gates not necessarily straight
   * + + + + + + + + + +
   * | | |   | | | | |
   * | |       | | |
   * |     |     |     |
   *     | | |       | |
   *   | | | | |   | | |
   * + + + + + + + + + +
   */
  for (size_t i = 0; i != corridor.size(); i++) {
    double x = double(i) / corridor.size();
    x = std::abs(std::fmod(x * INTERVALS, 1.) * 2. - 1.);
    double left = x * (1. - RATIO);
    double right = (1. - x) * (1. - RATIO);
    corridor.at(i).shrinkFromLeftBy(left * GATE_WIDTH);
    corridor.at(i).shrinkFromRightBy(right * GATE_WIDTH);
  }

  publishCorridorWaitAndSpinOnce(corridor);
  ASSERT_TRUE(latest_path != NULL) << "No path recieved.";
  std::vector<Eigen::Vector3d> path;
  common::msg_helper::fromMsg(*latest_path, path);

  // Test
  EXPECT_EQ(corridor.size(), path.size());
  for (size_t i = 0; i < corridor.size(); ++i) {
    EXPECT_TRUE(pointOnGate(corridor.at(i), path[i]));
  }
}

TEST_F(PathPlanningTest, ReverseOutParkingSpotService) {
  // Publish corridor -> save path for checks
  // Use service (enable)
  // Publish corridor -> path overwritten(?)
  // Use service (disable)
  // Publish corridor -> path not overwritten anymore(?)

  // Publish first corridor
  DrivingCorridor first_corridor;
  first_corridor.push_back(createFirstGate(4));
  publishCorridorWaitAndSpinOnce(first_corridor);
  std::vector<Eigen::Vector3d> first_path;
  common::msg_helper::fromMsg(*latest_path, first_path);

  // Use service (enable)
  std::vector<Eigen::Vector3d> reverse_path;
  reverse_path.push_back(Eigen::Vector3d(7.0, 8.0, 9.0));
  nav_msgs::Path path = common::msg_helper::createPathMsg(
      reverse_path, ros::Time::now(), "world");

  navigation_msgs::ReverseOutOfParkingSpot srv_enable;
  srv_enable.request.path_reverse_out_of_parking_spot = path;
  srv_enable.request.enable = true;
  ASSERT_TRUE(client.call(srv_enable));

  // Publish second corridor
  DrivingCorridor second_corridor;
  second_corridor.push_back(createFirstGate(5));
  publishCorridorWaitAndSpinOnce(second_corridor);
  std::vector<Eigen::Vector3d> second_path;
  common::msg_helper::fromMsg(*latest_path, second_path);

  // Check if path is overwritten as expected
  ASSERT_NE(first_path, second_path);

  // Use service (disable)
  navigation_msgs::ReverseOutOfParkingSpot srv_disable;
  srv_disable.request.enable = false;
  ASSERT_TRUE(client.call(srv_disable));

  // Publish third corridor
  DrivingCorridor third_corridor;
  third_corridor.push_back(createFirstGate(6));
  publishCorridorWaitAndSpinOnce(third_corridor);
  std::vector<Eigen::Vector3d> third_path;
  common::msg_helper::fromMsg(*latest_path, third_path);

  // Check if path is not overwritten anymore
  ASSERT_EQ(first_path, third_path);
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "test_node_path_planning");
  return RUN_ALL_TESTS();
}
