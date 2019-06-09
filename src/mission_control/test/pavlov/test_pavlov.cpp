#include <common/macros.h>

THIRD_PARTY_HEADERS_BEGIN
#include <atomic>
#include <string>
#include <ros/ros.h>
#include <tf2_eigen/tf2_eigen.h>
#include <gtest/gtest.h>
#include <std_msgs/UInt64.h>
#include <longitudinal_controller_msgs/StopAt.h>
#include <navigation_msgs/DrivingCorridor.h>
#include <navigation_msgs/Obstacles.h>
#include <navigation_msgs/Crosswalks.h>
#include <perception_msgs/Junctions.h>
#include <navigation_msgs/RoadClosures.h>
#include <navigation_msgs/Junctions.h>
#include <common_msgs/ActivationService.h>
#include <std_msgs/Empty.h>
THIRD_PARTY_HEADERS_END

#include "navigation/driving_corridor.h"
#include "navigation/gate.h"
#include "common/tf2_eigen_addon.h"

DISABLE_SUGGEST_OVERRIDE_WARNING

const ros::Duration TIME_OUT(1);

template <typename REQ, typename RES>
class MockServiceServer {
  ros::NodeHandle nh;
  ros::ServiceServer service_server;
  ros::AsyncSpinner spinner;
  std::atomic_bool called;

 public:
  MockServiceServer(std::string ns, std::string service_name)
      : nh(ns), spinner(1), called(false) {
    service_server = nh.advertiseService(service_name, &MockServiceServer::handle, this);
    spinner.start();
  }

  bool handle(REQ&, RES&) {
    called.store(true);
    // Yes, whatever.
    return true;
  }

  bool wasCalled() { return called.load(); }

  void reset() { called.store(false); }

  bool waitForCall() {
    reset();
    ros::Rate spin_rate(10);
    ros::Time t_start = ros::Time::now();
    while (ros::Time::now() - t_start < TIME_OUT) {
      ros::spinOnce();
      if (wasCalled()) {
        return true;
      }
      spin_rate.sleep();
    }

    return false;
  }
};

class PavlovTest : public testing::Test {
 protected:
  ros::NodeHandle nh;
  MockServiceServer<longitudinal_controller_msgs::StopAt::Request,
                    longitudinal_controller_msgs::StopAt::Response> mockStopServiceServer;
  ros::Publisher crosswalk_publisher;
  ros::Publisher junction_publisher;
  ros::Publisher road_closure_publisher;
  ros::Publisher stopping_completed_publisher;
  ros::Publisher full_corridor_publisher;
  ros::Publisher obstacles_publisher;
  ros::Publisher reset_publisher;

  bool stopping;
  float gate_distance;

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

  void activatePavlov() {
    ros::ServiceClient client = nh.serviceClient<common_msgs::ActivationService>(
        "/mission_control/obstacle_pavlov/activate_module");
    bool exists = client.waitForExistence(TIME_OUT);
    if (!exists) {
      FAIL() << "Pavlov node activation service not available within timeout.";
    }

    common_msgs::ActivationService activation;
    activation.request.moduleActive = true;

    if (!client.call(activation)) {
      FAIL() << "Failed to activate Pavlov node.";
    }
  }

  void deactivatePavlov() {
    ros::ServiceClient client = nh.serviceClient<common_msgs::ActivationService>(
        "/mission_control/obstacle_pavlov/activate_module");
    common_msgs::ActivationService activation;
    activation.request.moduleActive = false;

    if (!client.call(activation)) {
      FAIL() << "Failed to deactivate Pavlov node.";
    }
  }

  void publishCorridorAndWait() {
    Gate::GateList gates;
    float target_corridor_len = 3;

    int id = 0;
    for (float cor_len = 0; cor_len < target_corridor_len; cor_len += gate_distance) {
      id++;
      Eigen::Vector3d right(cor_len, -0.5, 0);
      Eigen::Vector3d left(cor_len, 0.5, 0);
      gates.push_back(Gate(id, left, right));
    }

    auto msg = DrivingCorridor(gates).toMessage();

    full_corridor_publisher.publish(msg);

    // Wait for the message to be handled.
    // The following tests depend on it.
    ros::Duration(1).sleep();
  }

  void publishObstacle(Eigen::Vector3d pos) {
    navigation_msgs::Obstacle ob;
    ob.pose.position.x = pos[0];
    ob.pose.position.y = pos[1];
    ob.pose.position.z = pos[2];

    navigation_msgs::Obstacles obs;
    obs.sub_messages.push_back(ob);

    obstacles_publisher.publish(obs);
  }

  void publishCrosswalk(int id, double distance, const bool pedestrian_waiting) {
    navigation_msgs::Crosswalks crosswalks_msg;
    crosswalks_msg.header.stamp = ros::Time::now();
    navigation_msgs::Crosswalk crosswalk_msg;
    crosswalk_msg.pose.position.x = distance;
    crosswalk_msg.pedestrian_waiting = pedestrian_waiting;
    crosswalk_msg.id = id;
    crosswalks_msg.sub_messages.push_back(crosswalk_msg);
    crosswalk_publisher.publish(crosswalks_msg);
  }

  void publishJunction(int id, double distance, unsigned int junction_type, bool right_lane = true) {
    navigation_msgs::Junctions junctions_msg;
    junctions_msg.header.stamp = ros::Time::now();
    navigation_msgs::Junction junction_msg;
    junction_msg.pose.position.x = distance;
    junction_msg.pose.position.y = right_lane ? -0.25 : 0.25;
    junction_msg.junction_type = junction_type;
    junction_msg.id = id;
    junctions_msg.sub_messages.push_back(junction_msg);
    junction_publisher.publish(junctions_msg);
  }

  void publishRoadClosure(int id, double distance, double len) {
    navigation_msgs::RoadClosures rcs_msg;
    rcs_msg.header.stamp = ros::Time::now();
    navigation_msgs::RoadClosure rc_msg;
    rc_msg.hull_polygon.push_back(tf2::toMsg(Eigen::Vector3d(distance, 0, 0)));
    rc_msg.hull_polygon.push_back(tf2::toMsg(Eigen::Vector3d(distance, 0.2, 0)));
    rc_msg.hull_polygon.push_back(tf2::toMsg(Eigen::Vector3d(distance + len, 0.2, 0)));
    rc_msg.hull_polygon.push_back(tf2::toMsg(Eigen::Vector3d(distance + len, 0, 0)));
    rc_msg.id = id;
    rcs_msg.sub_messages.push_back(rc_msg);
    road_closure_publisher.publish(rcs_msg);
  }

 public:
  PavlovTest()
      : nh("/mission_control/obstacle_pavlov"),
        mockStopServiceServer("/mission_control/obstacle_pavlov", "stop_at"),
        gate_distance(0.03) {
    crosswalk_publisher = nh.advertise<navigation_msgs::Crosswalks>(
        "/mission_control/obstacle_pavlov/crosswalks", 100);
    junction_publisher = nh.advertise<navigation_msgs::Junctions>(
        "/mission_control/obstacle_pavlov/junctions", 100);
    road_closure_publisher = nh.advertise<navigation_msgs::RoadClosures>(
        "/mission_control/obstacle_pavlov/road_closures", 100);
    stopping_completed_publisher = nh.advertise<std_msgs::UInt64>(
        "/mission_control/obstacle_pavlov/stopping_completed", 100);
    full_corridor_publisher = nh.advertise<navigation_msgs::DrivingCorridor>(
        "/mission_control/obstacle_pavlov/full_corridor", 100);
    obstacles_publisher = nh.advertise<navigation_msgs::Obstacles>(
        "/mission_control/obstacle_pavlov/obstacles", 100);
    reset_publisher = nh.advertise<std_msgs::Empty>(
        "/mission_control/obstacle_pavlov/auto_reset", 100);
    ros::Duration(0.1).sleep();
    activatePavlov();
    // Wait for the testee node to publish and subscribe.
    expectSubscription(crosswalk_publisher);
    expectSubscription(junction_publisher);
    expectSubscription(road_closure_publisher);
    expectSubscription(stopping_completed_publisher);
    expectSubscription(full_corridor_publisher);
    expectSubscription(obstacles_publisher);

    publishCorridorAndWait();
  }

  ~PavlovTest() { deactivatePavlov(); }
};

TEST_F(PavlovTest, Activate_OnCall_SubscribesAndPublishesCorrectly) {
  // Wait for the testee node to subscribe.
  ASSERT_NE(crosswalk_publisher.getNumSubscribers(), 0);
  ASSERT_NE(junction_publisher.getNumSubscribers(), 0);
  ASSERT_NE(road_closure_publisher.getNumSubscribers(), 0);
  ASSERT_NE(stopping_completed_publisher.getNumSubscribers(), 0);
  ASSERT_NE(full_corridor_publisher.getNumSubscribers(), 0);
  ASSERT_NE(obstacles_publisher.getNumSubscribers(), 0);
}

TEST_F(PavlovTest, CrosswalkWithPedestrians_WhileNormalDriving_WaitsUntilPedestriansPassed) {
  reset_publisher.publish(std_msgs::Empty());
  ros::Duration(0.5).sleep();
  publishCrosswalk(11, 3, true);
  mockStopServiceServer.reset();

  publishCrosswalk(11, 2, true);

  ASSERT_TRUE(mockStopServiceServer.waitForCall());

  std_msgs::UInt64 stopped_msg;
  stopped_msg.data = 0;
  stopping_completed_publisher.publish(stopped_msg);
}

TEST_F(PavlovTest, CrosswalkWithPedestrians_AfterCrosswalkWithoutPedestrians_GetsHandled) {
  reset_publisher.publish(std_msgs::Empty());
  ros::Duration(0.5).sleep();
  mockStopServiceServer.reset();
  publishCrosswalk(11, 1, false);

  ASSERT_FALSE(mockStopServiceServer.waitForCall());
  mockStopServiceServer.reset();
  publishCrosswalk(11, 1, true);

  ASSERT_TRUE(mockStopServiceServer.waitForCall());
}

TEST_F(PavlovTest, CrosswalkWithPedestrians_StoppingGetUpdate) {
  reset_publisher.publish(std_msgs::Empty());
  ros::Duration(0.5).sleep();
  publishCrosswalk(11, 1, true);

  ASSERT_TRUE(mockStopServiceServer.waitForCall());

  mockStopServiceServer.reset();
  publishCrosswalk(22, 3, true);

  ASSERT_TRUE(mockStopServiceServer.waitForCall());
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "test_pavlov");

  return RUN_ALL_TESTS();
}
