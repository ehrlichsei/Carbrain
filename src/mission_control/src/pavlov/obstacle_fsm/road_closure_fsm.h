#ifndef ROAD_CLOSURE_FSM_H
#define ROAD_CLOSURE_FSM_H

#include "hl_fsm.h"
namespace SM {
struct RoadClosureStateVisualization : public DrivingSubStateVisualization {
public:
  std::string getName() override { return "road closure"; }
};
struct RoadClosureSubStateVisualization : public VisualizationState {
public:
  std::string getBase() override { return "road closure"; }
};
struct StoppingAtRoadClosure : public RoadClosureSubStateVisualization {
  std::string getName() override { return "stopping at road closure"; }
};
struct WaitingAtRoadClosure : public RoadClosureSubStateVisualization {
  std::string getName() override { return "waiting at road closure"; }
};
struct RoadClosureActive : public RoadClosureSubStateVisualization {
  std::string getName() override { return "road closure active"; }
};
struct no_road_closure : public RoadClosureSubStateVisualization {
  std::string getName() override { return "no road closure"; }
};
struct DataRoadClosure {
  unsigned long stopping_id_ = 0;
  Timeout timeout_;
  unsigned long current_id_;
  std::set<unsigned long> completed_;
};


struct RoadClosureSM : public RoadClosureStateVisualization {
  RoadClosureSM() = default;
  RoadClosureSM(const std::shared_ptr<SharedData>& shared_data,
                const std::shared_ptr<DataRoadClosure>& data_road_closure)
      : shared_data_(shared_data), data_road_closure_(data_road_closure) {}
  auto operator()() const {
    const auto stopBeforeRoadClosure = [](
        const std::shared_ptr<DataRoadClosure> data_road_closure,
        const EventRoadClosureBlockedDetected& event,
        const std::shared_ptr<SharedData> shared_data) {
      double safety_margin = shared_data->getSafetyMargins().road_closure_margin;
      data_road_closure->stopping_id_ = shared_data->getCarController().stopAtDistance(
          std::max(0.0, event.distance - safety_margin), data_road_closure->stopping_id_);
      data_road_closure->current_id_ = event.id;
    };
    IfTimeoutPassed<DataRoadClosure> ifTimeoutPassed;
    StartDriving<DataRoadClosure> startDriving;
    AddToCompleted<DataRoadClosure> addToCompleted;
    StartDrivingIfStoppingIdNotZero<DataRoadClosure> startDrivingIfStoppingIdNotZero;
    StoppingIdMatches<DataRoadClosure> stoppingIdMatches;
    const auto restrictCorridor = std::bind(&ICarController::setDrivePastNextRoadClosure,
                                            &shared_data_->getCarController(),
                                            false);
    const auto finishPassingNextRoadClosure = std::bind(
        &ICarController::setDrivePastNextRoadClosure, &shared_data_->getCarController(), true);
    const auto setRoadClosureTimeOut =
        std::bind(setTimeout<DataRoadClosure>, data_road_closure_, 5.5);
    const auto reactToRoadClosureBlocked =
        [](const std::shared_ptr<DataRoadClosure> data_road_closure,
           const EventRoadClosureBlockedDetected& event) {
          return data_road_closure->completed_.count(event.id) == 0;
        };
    SetStoppingIdToZero<DataRoadClosure> setStoppingIdToZero;

    using namespace sml;
    // clang-format off
    DISABLE_USED_BUT_MARKED_UNUSED_WARNING
    return make_transition_table(
        * state<RoadClosureActive>     + sml::on_exit<_> / (startDrivingIfStoppingIdNotZero, finishPassingNextRoadClosure),
                                       
        * state<no_road_closure>       + event<EventRoadClosureBlockedDetected>[reactToRoadClosureBlocked] / (restrictCorridor,
                                                                                                              setStoppingIdToZero,
                                                                                                              defer)        = state<StoppingAtRoadClosure>,
          state<StoppingAtRoadClosure> + event<EventRoadClosureBlockedDetected> / stopBeforeRoadClosure,
          state<StoppingAtRoadClosure> + event<EventCarHasStopped>[stoppingIdMatches] / setRoadClosureTimeOut               = state<WaitingAtRoadClosure>,

          state<WaitingAtRoadClosure>  + event<EventTimeUpdate>[ifTimeoutPassed] / (startDriving, finishPassingNextRoadClosure, addToCompleted) = state<no_road_closure>);
    // clang-format on
    ENABLE_USED_BUT_MARKED_UNUSED_WARNING
  }
  std::shared_ptr<SharedData> shared_data_;
  std::shared_ptr<DataRoadClosure> data_road_closure_;
};
}

#endif
