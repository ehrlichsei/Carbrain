#ifndef NO_PASSING_FSM_H
#define NO_PASSING_FSM_H

#include "hl_fsm.h"
#include "../node_helper/passing_point_checker.h"
#include <common/macros.h>
THIRD_PARTY_HEADERS_BEGIN
#include <functional>
#include <tf2_eigen/tf2_eigen.h>
THIRD_PARTY_HEADERS_END

namespace SM {
struct NoPassingStateVisualization : public DrivingSubStateVisualization {
public:
  std::string getName() override { return "no passing"; }
};
struct NoPassingSubStateVisualization : public VisualizationState {
public:
  std::string getBase() override { return "no passing"; }
};
struct NoObstacle : public NoPassingSubStateVisualization {
  std::string getName() override { return "no obstacle"; }
};
struct FollowingObstacle : public NoPassingSubStateVisualization {
  std::string getName() override { return "following obstacle"; }
};
struct on_left_lane : public NoPassingSubStateVisualization {
  std::string getName() override { return "on left lane"; }
};
struct on_right_lane : public NoPassingSubStateVisualization {
  std::string getName() override { return "on right lane"; }
};
struct passing_allowed : public NoPassingSubStateVisualization {
  std::string getName() override { return "passing allowed"; }
};
struct DataPassing {
  bool on_left_lane = true;
  void setOnLeftLane(bool value) { on_left_lane = value; }
};

struct InNoPassingZone : public NoPassingStateVisualization {  // sub state machine
  InNoPassingZone() = default;
  InNoPassingZone(const std::shared_ptr<SharedData>& shared_data)
      : shared_data_(shared_data) {}
  auto operator()() const noexcept {
    const auto sendACCMessage = [](const EventObstacleAheadDetectedRight& ev,
                                   const std::shared_ptr<SharedData> shared_data) {
      shared_data->getCarController().publishACCMessage(ev.obstacle_speed, ev.driving_distance);
    };
    const auto activateACC = std::bind(
        &ICarController::deActivateACC, &shared_data_->getCarController(), true);
    const auto deactivateACC = std::bind(
        &ICarController::deActivateACC, &shared_data_->getCarController(), false);
    using namespace sml;
    // clang-format off
    DISABLE_USED_BUT_MARKED_UNUSED_WARNING
    return make_transition_table(
        * state<NoObstacle>        + event<EventObstacleAheadDetectedRight> / defer  = state<FollowingObstacle>,
          state<FollowingObstacle> + event<EventNoObstacleAheadDetectedRight>        = state<NoObstacle>,
          state<FollowingObstacle> + sml::on_entry<_> / activateACC,
          state<FollowingObstacle> + sml::on_exit<_> / deactivateACC,
          state<FollowingObstacle> + event<EventObstacleAheadDetectedRight> / sendACCMessage);
    // clang-format on
    ENABLE_USED_BUT_MARKED_UNUSED_WARNING
  }

 private:
  std::shared_ptr<SharedData> shared_data_;
};


struct PassingSM : public NoPassingStateVisualization {
  PassingSM() = default;
  PassingSM(const std::shared_ptr<SharedData>& shared_data,
            const std::shared_ptr<DataPassing>& data_passing)
      : shared_data_(shared_data), data_passing_(data_passing) {}
  auto operator()() const {
    const auto ifAcceptNoPassingZone =
        [](const std::shared_ptr<DataPassing> data_passing, const EventNoPassingZone& ev) {
          return !data_passing->on_left_lane &&
                 (ev.vehicle_pose.inverse() * ev.start).x() < 0.8;
        };
    const auto ifRejectNoPassingZone = [](const EventNoPassingZone& ev) {
      return (ev.vehicle_pose.inverse() * ev.start).x() > 1.0;
    };

    const auto setOnLeftLane = std::bind(&DataPassing::setOnLeftLane, data_passing_, true);
    const auto setOnRightLane =
        std::bind(&DataPassing::setOnLeftLane, data_passing_, false);
    const auto respectNoPassingZone = std::bind(
        &ICarController::setRespectNoPassingZone, &shared_data_->getCarController(), true);
    const auto ignoreNoPassingZone = std::bind(
        &ICarController::setRespectNoPassingZone, &shared_data_->getCarController(), false);

    using namespace sml;
    // clang-format off
    DISABLE_USED_BUT_MARKED_UNUSED_WARNING
    return make_transition_table(
        * state<on_left_lane>       + event<EventOnRightLane>    / setOnRightLane      = state<on_right_lane>,
          state<on_right_lane>      + event<EventNotOnRightLane> / setOnLeftLane       = state<on_left_lane>,
                                 
        * state<passing_allowed>    + event<EventNoPassingZone>[ifAcceptNoPassingZone] = state<InNoPassingZone>,
          state<InNoPassingZone> + event<EventNoPassingZone>[ifRejectNoPassingZone] = state<passing_allowed>,
          state<InNoPassingZone> + event<EventNoPassingMessageEmpty>                = state<passing_allowed>,
          state<InNoPassingZone> + sml::on_entry<_> / respectNoPassingZone,
          state<InNoPassingZone> + sml::on_exit<_>  / ignoreNoPassingZone);
    // clang-format on
    ENABLE_USED_BUT_MARKED_UNUSED_WARNING
  }

 private:
  std::shared_ptr<SharedData> shared_data_;
  std::shared_ptr<DataPassing> data_passing_;
};
}

#endif
