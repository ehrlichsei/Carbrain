#ifndef JUNCTION_FSM_H
#define JUNCTION_FSM_H


#include "shared_data.h"
#include <common/macros.h>
THIRD_PARTY_HEADERS_BEGIN
#include <functional>
#include <set>
THIRD_PARTY_HEADERS_END
#include "hl_fsm.h"

namespace SM {
struct JunctionStateVisualization : public DrivingSubStateVisualization {
 public:
  std::string getName() override { return "junction"; }
};

struct JunctionSubStateVisualization : public VisualizationState {
 public:
  std::string getBase() override { return "junction"; }
};

struct StoppingAtStopline : public JunctionSubStateVisualization {
  std::string getName() override { return "stopping at stopline"; }
};
struct StoppingAtHoldline : public JunctionSubStateVisualization {
  std::string getName() override { return "stopping at holdline"; }
};
struct WaitingAtJunction : public JunctionSubStateVisualization {
  std::string getName() override { return "waiting at junction"; }
};
struct JunctionActive : public JunctionSubStateVisualization {
  std::string getName() override { return "no visualization"; }
};
struct activate_stopping_and_look_at : public JunctionSubStateVisualization {
  std::string getName() override { return "activate stopping and look at"; }
};
struct junction_is_empty : public JunctionSubStateVisualization {
  std::string getName() override { return "junction is empty"; }
};
struct junction_has_obstacle : public JunctionSubStateVisualization {
  std::string getName() override { return "junction has obstacle"; }
};
struct no_junction : public JunctionSubStateVisualization {
  std::string getName() override { return "no junction"; }
};

struct DataJunctionSM {
  unsigned long stopping_id_ = 0;
  Timeout timeout_;
  unsigned long current_id_;
  std::set<unsigned long> completed_;

};

struct JunctionSM : public JunctionStateVisualization {
  JunctionSM() = default;
  JunctionSM(const std::shared_ptr<SharedData>& shared_data,
             const std::shared_ptr<DataJunctionSM>& data_junction)
      : shared_data_(shared_data), data_junction_(data_junction) {
    data_junction_->timeout_.setTimeoutWhenBlocked(ros::Duration(8.0));
  }
  static void stopAtDistance(const std::shared_ptr<DataJunctionSM> data_junction,
                             const double distance,
                             const std::shared_ptr<SharedData> shared_data) {
    double safety_margin = shared_data->getSafetyMargins().junction_margin;
    data_junction->stopping_id_ = shared_data->getCarController().stopAtDistance(
        std::max(0.0, distance - safety_margin), data_junction->stopping_id_);
  }
  auto operator()() const {
    const auto reactToStopJunction = [](const std::shared_ptr<DataJunctionSM> data_junction,
                                        const EventStopJunction& event) {
      return data_junction->completed_.count(event.id) == 0;
    };
    const auto reactToHoldJunction = [](const std::shared_ptr<DataJunctionSM> data_junction,
                                        const EventHoldJunction& event) {
      return data_junction->completed_.count(event.id) == 0;
    };

    const auto stopAtStopJunction = [](const std::shared_ptr<DataJunctionSM> data_junction,
                                       const EventStopJunction& event,
                                       const std::shared_ptr<SharedData> shared_data) {
      stopAtDistance(data_junction, event.distance, shared_data);
      data_junction->current_id_ = event.id;
    };
    const auto stopAtHoldJunction = [](const std::shared_ptr<DataJunctionSM> data_junction,
                                       const EventHoldJunction& event,
                                       const std::shared_ptr<SharedData> shared_data) {
      stopAtDistance(data_junction, event.distance, shared_data);
      data_junction->current_id_ = event.id;
    };

    const auto startDrivingAndStopLookAt =
        [](const std::shared_ptr<DataJunctionSM> data_junction,
           const std::shared_ptr<SharedData> shared_data) {
          shared_data->getCarController().stopLookAt();
          if (data_junction->stopping_id_ != 0) {
            StartDriving<DataJunctionSM> startDriving;
            startDriving(data_junction, shared_data);
          }
        };
    IfTimeoutPassed<DataJunctionSM> ifTimeoutPassed;
    AddToCompleted<DataJunctionSM> addToCompleted;
    StoppingIdMatches<DataJunctionSM> stoppingIdMatches;
    const auto setStopplineWaitingTime =
        std::bind(setNewDuration<DataJunctionSM>, data_junction_, 3);
    const auto setHoldlineWaitingTime =
        std::bind(setNewDuration<DataJunctionSM>, data_junction_, 1);

    using namespace sml;
    // clang-format off
    DISABLE_USED_BUT_MARKED_UNUSED_WARNING
    return make_transition_table(
        * state<JunctionActive>                        + sml::on_exit<_> / startDrivingAndStopLookAt,
        * state<activate_stopping_and_look_at> + event<EventStopJunction>[reactToStopJunction] / stopAtStopJunction,
          state<activate_stopping_and_look_at> + event<EventHoldJunction>[reactToHoldJunction] / stopAtHoldJunction,
                                                                                                                                    
        * state<no_junction>                   + event<EventStopJunction>[reactToStopJunction]                                         = state<StoppingAtStopline>,
          state<no_junction>                   + event<EventHoldJunction>[reactToHoldJunction]                                         = state<StoppingAtHoldline>,

          state<StoppingAtStopline>         + event<EventHoldJunction>                                                              = state<StoppingAtHoldline>,
          state<StoppingAtStopline>         + event<EventCarHasStopped>[stoppingIdMatches] / setStopplineWaitingTime                = state<WaitingAtJunction>,

          state<StoppingAtHoldline>         + event<EventStopJunction>                                                              = state<StoppingAtStopline>,
          state<StoppingAtHoldline>         + event<EventCarHasStopped>[stoppingIdMatches] / setHoldlineWaitingTime                 = state<WaitingAtJunction>,

          state<WaitingAtJunction>          + event<EventTimeUpdate>[ifTimeoutPassed] / (startDrivingAndStopLookAt, addToCompleted) = state<no_junction>,

        * state<junction_is_empty>             + event<EventJunctionBlockedByObstacle>                                                 = state<junction_has_obstacle>,
          state<junction_has_obstacle>         + event<EventEmptyJunction>                                                             = state<junction_is_empty>,

          state<junction_is_empty>             + sml::on_entry<_> / std::bind(&Timeout::setBlocked, &data_junction_->timeout_, false),
          state<junction_has_obstacle>         + sml::on_entry<_> / std::bind(&Timeout::setBlocked, &data_junction_->timeout_, true)
            );
    // clang-format on
    ENABLE_USED_BUT_MARKED_UNUSED_WARNING
  }

 private:
  std::shared_ptr<SharedData> shared_data_;
  std::shared_ptr<DataJunctionSM> data_junction_;
};
}

#endif
