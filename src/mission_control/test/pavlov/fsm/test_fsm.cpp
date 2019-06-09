#include <common/macros.h>

THIRD_PARTY_HEADERS_BEGIN
#include <gtest/gtest.h>
#include <tf2_eigen/tf2_eigen.h>
#include <boost_sml_catkin/sml.hpp>
THIRD_PARTY_HEADERS_END

#include "common/test/dummy_parameter_handler.h"

#include "../../../src/pavlov/fsm_utils/carcontroller.h"
#include "../../../src/pavlov/fsm_utils/diagnostics_interface.h"
#include "../../../src/pavlov/obstacle_fsm/hl_fsm.h"
#include "../../../src/pavlov/obstacle_pavlov_node.h"
#include "../../../src/pavlov/node_helper/state_machine_logger.h"
#include "../../../src/pavlov/obstacle_fsm/shared_data.h"
#include "../../../src/pavlov/obstacle_fsm/turn_fsm.h"

DISABLE_SUGGEST_OVERRIDE_WARNING


// Mock car controller
// Gmock is not supported so this has to be a manual written mock with a lot of
// overhead code.
// TODO: Use some other mocking framework.
class FakeCarController : public ICarController {
 public:
  unsigned long stopAtDistance(const double /*distance*/, const unsigned long /*id*/) override {
    stop_at_distance_called = true;
    return 1;
  }
  void startDriving(const unsigned long /*id*/) override {
    start_driving_called = true;
  }
  unsigned long setMaxSpeedInModelSizeKmPerH(const double max_speed,
                                             const unsigned long id) override {
    set_max_speed_called = true;
    set_max_speed_param = max_speed;
    return id;
  }
  void clearMaxSpeed(const unsigned long) override {
    clear_max_speed_called = true;
  }

  void publishACCMessage(const double /*speed*/, const double /*distance*/) override {}
  void setRespectNoPassingZone(bool /*respect_no_passing_zone*/) override {}

  void setDrivePastNextRoadClosure(const bool should_drive_past) override {
    drive_past_next_road_closure_called = true;
    drive_past_next_road_closure_param = should_drive_past;
  }
  void startLookAt(const tf2::Stamped<Eigen::Affine3d> /*pose*/,
                   const Eigen::Vector3d /*rect*/) override {
    start_look_at = true;
  }
  void stopLookAt() override { stop_look_at = true; }
  void turnAt(const Eigen::Affine3d & /*pose*/, const TurnDirection /*direction*/) override {
    // TODO
  }
  void resetTurning() override {
    // TODO
  }
  void resetLaneDetection() override {
    // TODO
  }
  void resetPathPreprocessing() override {
    // TODO
  }
  void resetCarController() override {
    // TODO
  }

  void reset() {
    stop_at_distance_called = false;
    start_driving_called = false;
    set_max_speed_called = false;
    clear_max_speed_called = false;
    drive_past_next_road_closure_called = false;
    start_look_at = false;
    stop_look_at = false;
  }

  double set_max_speed_param;
  bool stop_at_distance_called;
  bool start_driving_called;
  bool set_max_speed_called;
  bool clear_max_speed_called;
  bool drive_past_next_road_closure_called;
  bool drive_past_next_road_closure_param;
  bool start_look_at;
  bool stop_look_at;

  // ICarController interface
 public:
  void deActivateACC(const bool /*activate*/) override {}
  void searchParkingSpot(const bool /*search*/) override {}
  void parkPerpendicular(const perception_msgs::PerpendicularParkingSpot & /*parking_spot*/) override {
  }
  void reverseOutOfParkingSpot(const nav_msgs::Path & /*path_reverse_out_of_parking_spot_world*/,
                               const bool /*enable*/) override {}
  void publishReversePathToWorldTransform(const Eigen::Affine3d & /*path_to_world_transform*/,
                                          const ros::Time & /*stamp*/) override {}

  void setPavlovBlinkerCommand(controller_msgs::BlinkerCommand::_command_type /*command*/) override {
  }


  unsigned long setMaxSpeedInMPerS(const double /*desired_speed_in_m_per_s*/,
                                   const unsigned long id) override {
    return id;
  }

  void blinkInParkingSpot(const bool /*blink*/) override {}
  void blinkLeftBeforeParkingSpot(const bool /*blink*/) override {}

  void resetPavlovBlinkerCommand() override {}
  void setHighBeam(const bool /*high_beam*/) override {}
  void resetEnvironmentalModel() override {}
  void straightPathOutOfStartBox(const bool /*enable*/) override {}
  void activateQrCodeDetection(const bool /*activate*/) override {}
  void setMaxSpeedAfterQRCode() override {}
};

class FakeDiagnosticsInterface : public IDiagnosticsInterface {
 public:
  void speak(const unsigned char /*lvl*/, const std::string /*msg*/) override {}
};


// Test fixture
class HlFsmTest : public ::testing::Test {
  static DummyParameterHandler setupParameterHandler() {
    DummyParameterHandler parameter_handler;
    parameter_handler.addParam(
        ParameterString<double>("safety_margin_crosswalk"), 1.0);
    parameter_handler.addParam(
        ParameterString<double>("safety_margin_road_closure"), 1.0);
    parameter_handler.addParam(
        ParameterString<double>("safety_margin_junction"), 0.4);
    parameter_handler.addParam(
        ParameterString<double>("no_passing_empty_msg_timeout_in_s"), 1.0);
    parameter_handler.addParam(
        ParameterString<double>("max_stop_at_update_distance"), 0.4);
    parameter_handler.addParam(
        ParameterString<double>("approaching_unsure_observation_timeout_in_s"), 3.0);
    parameter_handler.addParam(
        ParameterString<double>(
            "approaching_unsure_observation_speed_in_m_per_s"),
        0.8);
    parameter_handler.addParam(
        ParameterString<double>("unidentified_object_velocity_factor"), 2.5);
    parameter_handler.addParam(
        ParameterString<double>("unidentified_object_distance_lowest_velocity"), 1.5);
    parameter_handler.addParam(
        ParameterString<double>("unidentified_object_lowest_velocity"), 0.5);
    parameter_handler.addParam(
        ParameterString<double>("waiting_for_redetection_timeout_in_s"), 1.0);
    parameter_handler.addParam(
        ParameterString<double>("waiting_at_crosswalk_timeout_in_s"), 4.0);
    parameter_handler.addParam(
        ParameterString<double>(
            "waiting_at_junction_with_obstacle_timeout_in_s"),
        3.0);
    parameter_handler.addParam(
        ParameterString<double>("waiting_at_road_closure_timeout_in_s"), 3.0);
    parameter_handler.addParam(
        ParameterString<double>("unidentified_object_timeout"), 0.5);
    parameter_handler.addParam(
        ParameterString<double>(
            "blocking_entity_preemption_distance_threshold_in_m"),
        0.1);
    parameter_handler.addParam(
        ParameterString<double>("speed_limit_reset_distance/with_10_kmh"), 20.0);
    parameter_handler.addParam(
        ParameterString<double>("speed_limit_reset_distance/with_20_kmh"), 20.0);
    parameter_handler.addParam(
        ParameterString<double>("speed_limit_reset_distance/with_30_kmh"), 20.0);
    parameter_handler.addParam(
        ParameterString<double>("speed_limit_reset_distance/with_40_kmh"), 20.0);
    parameter_handler.addParam(
        ParameterString<double>("speed_limit_reset_distance/with_50_kmh"), 20.0);
    parameter_handler.addParam(
        ParameterString<double>("speed_limit_reset_distance/with_60_kmh"), 20.0);
    parameter_handler.addParam(
        ParameterString<double>("speed_limit_reset_distance/with_70_kmh"), 20.0);
    parameter_handler.addParam(
        ParameterString<double>("speed_limit_reset_distance/with_80_kmh"), 20.0);
    parameter_handler.addParam(
        ParameterString<double>("speed_limit_reset_distance/with_90_kmh"), 20.0);
    return parameter_handler;
  }
  ObstaclePavlovNode::StateMachine createTestObstacleStateMachine(
      const std::shared_ptr<SharedData> &shared_data, StateMachineLogger &logger) {
    auto data_junction = std::make_shared<SM::DataJunctionSM>();
    auto data_road_closure = std::make_shared<SM::DataRoadClosure>();
    auto data_passing = std::make_shared<SM::DataPassing>();
    auto data_turn = std::make_shared<SM::DataTurn>();
    auto data_crosswalk = std::make_shared<SM::DataCrosswalk>();
    auto data_speed_limit = std::make_shared<SM::DataSpeedLimit>();
    auto data_qr_code = std::make_shared<SM::DataQRCode>();
    return ObstaclePavlovNode::StateMachine(
        shared_data,
        data_junction,
        data_road_closure,
        data_passing,
        data_turn,
        data_crosswalk,
        data_speed_limit,
        data_qr_code,
        SM::PassingSM(shared_data, data_passing),
        SM::InNoPassingZone(shared_data),
        SM::CrosswalkSM(shared_data, data_crosswalk),
        SM::JunctionSM(shared_data, data_junction),
        SM::RoadClosureSM(shared_data, data_road_closure),
        SM::SpeedLimitSM(shared_data, data_speed_limit),
        SM::TurningSM(shared_data, data_turn),
        SM::Driving(),
        SM::QRCodeFSM<SharedData, SM::Driving>(),
        logger);
  }

 public:
  HlFsmTest()
      : dummy_parameter_handler(setupParameterHandler()),
        shared_data_(std::make_shared<SharedData>(
            &dummy_parameter_handler, fake_car_controller, fake_diagnostics_iface)),
        state_machine_(createTestObstacleStateMachine(shared_data_, logger_)) {
    ros::Time::init();
  }

 protected:
  FakeCarController fake_car_controller;
  FakeDiagnosticsInterface fake_diagnostics_iface;
  DummyParameterHandler dummy_parameter_handler;
  std::shared_ptr<SharedData> shared_data_;
  StateMachineLogger logger_;
  ObstaclePavlovNode::StateMachine state_machine_;
};


template <class STATE>
void assertIsInQRCodeSubSM(ObstaclePavlovNode::StateMachine &fsm) {
  using namespace boost::sml;
  using namespace SM;
  ASSERT_TRUE(fsm.is<decltype(state<SM::QRCodeFSM<SharedData, Driving>>)>(state<STATE>));
}

template <class ROAD_CLOSURE, class JUNCTION, class CROSSWALK, class TURNING, class PASSING, class SPEED_LIMIT>
void assertIsInDrvingSubSM(ObstaclePavlovNode::StateMachine &fsm) {
  using namespace boost::sml;
  using namespace SM;
  ASSERT_TRUE(fsm.is<decltype(state<Driving>)>(state<ROAD_CLOSURE>,
                                               state<JUNCTION>,
                                               state<CROSSWALK>,
                                               state<TURNING>,
                                               state<PASSING>,
                                               state<SPEED_LIMIT>));
}

TEST_F(HlFsmTest, EventStart_OnProcess_StartsDriving) {
  using namespace boost::sml;
  using namespace SM;
  ASSERT_TRUE(state_machine_.is(state<SM::QRCodeFSM<SharedData, Driving>>));

  state_machine_.process_event(EventStart());
  assertIsInQRCodeSubSM<Driving>(state_machine_);

  ASSERT_TRUE(state_machine_.is<decltype(state<SM::QRCodeFSM<SharedData, Driving>>)>(
      state<SM::Driving>));
  assertIsInDrvingSubSM<NoRoadClosureHL, NoJunctionHL, NoCrosswalkHL, TurningSM, PassingSM, SpeedLimitSM>(
      state_machine_);
}
TEST_F(HlFsmTest, TurningStateMachine) {
  using namespace boost::sml;
  using namespace SM;

  state_machine_.process_event(EventStart());

  assertIsInQRCodeSubSM<Driving>(state_machine_);
  ASSERT_TRUE(state_machine_.is<decltype(state<SM::TurningSM>)>(state<NoTurn>));
  state_machine_.process_event(EventTurnDetected(
      0, Eigen::Affine3d::Identity(), 1.0, TurnDirection::LEFT, ros::Time::now()));
  ASSERT_TRUE(state_machine_.is<decltype(state<SM::TurningSM>)>(state<Turning>));
  state_machine_.process_event(EventNoTurn());
  ASSERT_TRUE(state_machine_.is<decltype(state<SM::TurningSM>)>(state<NoTurn>));
  state_machine_.process_event(EventTurnDetected(
      1, Eigen::Affine3d::Identity(), 1.0, TurnDirection::LEFT, ros::Time::now()));
  ASSERT_TRUE(state_machine_.is<decltype(state<SM::TurningSM>)>(state<Turning>));
  state_machine_.process_event(EventTurnPassed(1));
  ASSERT_TRUE(state_machine_.is<decltype(state<SM::TurningSM>)>(state<NoTurn>));
  state_machine_.process_event(EventTurnDetected(1,
                                                 Eigen::Affine3d::Identity(),
                                                 1.0,
                                                 TurnDirection::LEFT,
                                                 ros::Time::now()));  // same id
  ASSERT_TRUE(state_machine_.is<decltype(state<SM::TurningSM>)>(state<NoTurn>));
}

TEST_F(HlFsmTest, CrosswalkStateMachine) {
  using namespace boost::sml;
  using namespace SM;

  state_machine_.process_event(EventStart());

  assertIsInQRCodeSubSM<Driving>(state_machine_);
  assertIsInDrvingSubSM<NoRoadClosureHL, NoJunctionHL, NoCrosswalkHL, TurningSM, PassingSM, SpeedLimitSM>(
      state_machine_);
  state_machine_.process_event(EventCrosswalkWithPedestriansDetected(1.5, 0));
  assertIsInDrvingSubSM<NoRoadClosureHL, NoJunctionHL, CrosswalkSM, TurningSM, PassingSM, SpeedLimitSM>(
      state_machine_);
  ASSERT_TRUE(state_machine_.is<decltype(state<CrosswalkSM>)>(
      state<CrosswalkActive>, state<CrosswalkPedestrians>));
  ASSERT_TRUE(fake_car_controller.stop_at_distance_called);
  fake_car_controller.reset();
  state_machine_.process_event(EventCrosswalkFreeDetected(1.5, 0));
  ASSERT_TRUE(state_machine_.is<decltype(state<CrosswalkSM>)>(
      state<CrosswalkActive>, state<EmptyCrosswalk>));
  ASSERT_TRUE(fake_car_controller.start_driving_called);
  fake_car_controller.reset();

  state_machine_.process_event(EventCrosswalkWithPedestriansDetected(1.5, 0));
  ASSERT_TRUE(fake_car_controller.stop_at_distance_called);
  fake_car_controller.reset();
  state_machine_.process_event(EventNoCrosswalk());
  ASSERT_TRUE(fake_car_controller.start_driving_called);
  fake_car_controller.reset();

  assertIsInDrvingSubSM<NoRoadClosureHL, NoJunctionHL, NoCrosswalkHL, TurningSM, PassingSM, SpeedLimitSM>(
      state_machine_);
}

TEST_F(HlFsmTest, QRCodeDetection) {
  using namespace boost::sml;
  using namespace SM;
  assertIsInQRCodeSubSM<Start>(state_machine_);
  state_machine_.process_event(EventQRDetection());
  assertIsInQRCodeSubSM<QRDetected>(state_machine_);
  state_machine_.process_event(EventNoQRCodeVisible());
  assertIsInQRCodeSubSM<WaitInBox>(state_machine_);
  state_machine_.process_event(EventTimeUpdate(ros::Time::now() + ros::Duration(5.0)));
  state_machine_.process_event(EventDroveDistance(100.0));

  // handleReset in obstaclePavlovNode and freeDrivePavlovNode require that this
  // check is possible
  assertIsInQRCodeSubSM<Driving>(state_machine_);
  assertIsInDrvingSubSM<NoRoadClosureHL, NoJunctionHL, NoCrosswalkHL, TurningSM, PassingSM, SpeedLimitSM>(
      state_machine_);
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
