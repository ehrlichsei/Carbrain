#ifndef TURN_FSM_H
#define TURN_FSM_H

#include "hl_fsm.h"

THIRD_PARTY_HEADERS_BEGIN
#include "controller_msgs/BlinkerCommand.h"
THIRD_PARTY_HEADERS_END
namespace SM {
struct DataTurn {
  unsigned long current_id_;
  std::set<unsigned long> completed_;
};

struct TurnStateVisualization : public DrivingSubStateVisualization {
public:
  std::string getName() override { return "turn"; }
};
struct TurnSubStateVisualization : public VisualizationState {
public:
  std::string getBase() override { return "turn"; }
};
struct NoTurn : public TurnSubStateVisualization {
  std::string getName() override { return "no turn"; }
};
struct Turning : public TurnSubStateVisualization {
  std::string getName() override { return "turning"; }
};
struct TurningSM : public TurnStateVisualization {
  TurningSM() = default;
  TurningSM(const std::shared_ptr<SharedData>& shared_data,
            const std::shared_ptr<DataTurn>& data_turn)
      : shared_data_(shared_data), data_turn_(data_turn) {}
  auto operator()() const {
    const auto resetAfterTurning = [](const std::shared_ptr<DataTurn> data_turn,
                                      const std::shared_ptr<SharedData> shared_data,
                                      const EventTurnPassed& ev) {
      data_turn->current_id_ = ev.id;
      shared_data->getCarController().resetLaneDetection();
      shared_data->getCarController().resetPavlovBlinkerCommand();
      shared_data->getCarController().resetPathPreprocessing();
      shared_data->getCarController().resetEnvironmentalModel();
    };

    const auto resetPathPreprocessingAndBlinking =
        [](const std::shared_ptr<SharedData> shared_data) {
          shared_data->getCarController().resetPathPreprocessing();
          shared_data->getCarController().resetPavlovBlinkerCommand();
        };

    const auto turnAt = [](const std::shared_ptr<DataTurn> data_turn,
                           const EventTurnDetected& ev,
                           const std::shared_ptr<SharedData> shared_data) {
      shared_data->getCarController().turnAt(ev.turn_junction_pose_in_world, ev.direction);
      data_turn->current_id_ = ev.id;
    };
    const auto startBlinking =
        [](const EventTurnDetected& ev, const std::shared_ptr<SharedData> shared_data) {
          if (ev.direction == TurnDirection::RIGHT) {
            shared_data->getCarController().setPavlovBlinkerCommand(
                controller_msgs::BlinkerCommand::RIGHT);
          } else if (ev.direction == TurnDirection::LEFT) {
            shared_data->getCarController().setPavlovBlinkerCommand(
                controller_msgs::BlinkerCommand::LEFT);
          } else {
            ROS_WARN("unknown TurnDirection");
          }
        };

    const auto reactToTurn =
        [](const std::shared_ptr<DataTurn> data_turn, const EventTurnDetected& event) {
          return data_turn->completed_.count(event.id) == 0;
        };
    const auto idMatches =
        [](const std::shared_ptr<DataTurn> data_turn, const EventTurnPassed& event) {
          return event.id == data_turn->current_id_;
        };

    AddToCompleted<DataTurn> addToCompleted;
    using namespace sml;
    // clang-format off
    DISABLE_USED_BUT_MARKED_UNUSED_WARNING
    return make_transition_table(
        * state<NoTurn>  + event<EventTurnDetected>[reactToTurn] / (startBlinking, defer)            = state<Turning>,

          state<Turning> + event<EventTurnDetected> / turnAt,
          state<Turning> + event<EventTurnPassed>[idMatches] / (resetAfterTurning, addToCompleted)   = state<NoTurn>,
          state<Turning> + event<EventNoTurn> / resetPathPreprocessingAndBlinking                    = state<NoTurn>);
    // clang-format on
    ENABLE_USED_BUT_MARKED_UNUSED_WARNING
  }

 private:
  std::shared_ptr<SharedData> shared_data_;
  std::shared_ptr<DataTurn> data_turn_;
};
}

#endif
