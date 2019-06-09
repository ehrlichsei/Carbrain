#ifndef CROSSWALK_FSM_H
#define CROSSWALK_FSM_H

#include <common/macros.h>
THIRD_PARTY_HEADERS_BEGIN
#include <functional>
#include <boost_sml_catkin/sml.hpp>
THIRD_PARTY_HEADERS_END
#include "hl_fsm.h"
#include "shared_data.h"

namespace SM {
struct CrosswalkStateVisualization : public DrivingSubStateVisualization {
 public:
  std::string getName() override { return "crosswalk"; }
};
struct CrosswalkSubStateVisualization : public VisualizationState {
 public:
  std::string getBase() override { return "crosswalk"; }
};

struct EmptyCrosswalk : public CrosswalkSubStateVisualization {
  std::string getName() override { return "empty crosswalk"; }
};
struct CrosswalkPedestrians : public CrosswalkSubStateVisualization {
  std::string getName() override { return "crosswalk pedestrians"; }
};
struct WaitingForPedestrians : public CrosswalkSubStateVisualization {
  std::string getName() override { return "waiting for pedestrians"; }
};
struct CrosswalkActive : public CrosswalkSubStateVisualization {
  std::string getName() override { return "ignore"; }
};
struct NoCrosswalk : public CrosswalkSubStateVisualization {
  std::string getName() override { return "no crosswalk"; }
};

struct DataCrosswalk {
  unsigned long stopping_id_ = 0;
  unsigned long speed_limitation_id = 0;
  Timeout timeout_;
  unsigned long current_id_;
  std::set<unsigned long> completed_;
};

struct CrosswalkSM : public CrosswalkStateVisualization {
  CrosswalkSM() = default;
  CrosswalkSM(const std::shared_ptr<SharedData>& shared_data,
              const std::shared_ptr<DataCrosswalk>& data_crosswalk)
      : shared_data_(shared_data), data_crosswalk_(data_crosswalk) {}
  auto operator()() const {
    const auto crosswalkWithPedestriansDetected = [](
        const std::shared_ptr<DataCrosswalk> data_crosswalk,
        const EventCrosswalkWithPedestriansDetected& event,
        const std::shared_ptr<SharedData> shared_data) {
      double safety_margin = shared_data->getSafetyMargins().crosswalk_margin;
      data_crosswalk->stopping_id_ = shared_data->getCarController().stopAtDistance(
          std::max(0.0, event.distance - safety_margin), data_crosswalk->stopping_id_);
      data_crosswalk->current_id_ = event.id;
    };
    const auto reactToEmptyCrosswalk = [](const std::shared_ptr<DataCrosswalk> data_crosswalk,
                                          const EventCrosswalkFreeDetected& event) {
      return data_crosswalk->completed_.count(event.id) == 0;
    };
    const auto reactToCrosswalkWithPedestrians =
        [](const std::shared_ptr<DataCrosswalk> data_crosswalk,
           const EventCrosswalkWithPedestriansDetected& event) {
          return data_crosswalk->completed_.count(event.id) == 0;
        };

    const auto enterApproachingCrosswalk =
        [](const std::shared_ptr<DataCrosswalk> data_crosswalk,
           const std::shared_ptr<SharedData> shared_data) {
          StartDrivingIfStoppingIdNotZero<DataCrosswalk> startDrivingIfStoppingIdNotZero;
          startDrivingIfStoppingIdNotZero(data_crosswalk, shared_data);
          data_crosswalk->speed_limitation_id = shared_data->getCarController().setMaxSpeedInMPerS(
              shared_data->getApproachingUnsureObservationSpeedInMPerS(),
              data_crosswalk->speed_limitation_id);
        };
    const auto exitApproachingCrosswalk =
        [](const std::shared_ptr<DataCrosswalk> data_crosswalk,
           const std::shared_ptr<SharedData> shared_data) {
          shared_data->getCarController().clearMaxSpeed(data_crosswalk->speed_limitation_id);
          data_crosswalk->speed_limitation_id = 0;
        };
    StartDrivingIfStoppingIdNotZero<DataCrosswalk> startDrivingIfStoppingIdNotZero;
    SetStoppingIdToZero<DataCrosswalk> setStoppingIdToZero;


    IfTimeoutPassed<DataCrosswalk> ifTimeoutPassed;
    AddToCompleted<DataCrosswalk> addToCompleted;

    StartDriving<DataCrosswalk> startDriving;
    StoppingIdMatches<DataCrosswalk> stoppingIdMatches;

    using namespace sml;
    // clang-format off
    DISABLE_USED_BUT_MARKED_UNUSED_WARNING
    return make_transition_table(
        * state<CrosswalkActive>                   + sml::on_exit<_> / (startDrivingIfStoppingIdNotZero,setStoppingIdToZero),
        * state<NoCrosswalk>             + event<EventCrosswalkFreeDetected>[reactToEmptyCrosswalk]                        = state<EmptyCrosswalk>,
          state<NoCrosswalk>             + event<EventCrosswalkWithPedestriansDetected>[reactToCrosswalkWithPedestrians]
                                                                                                               / defer   = state<CrosswalkPedestrians>,
                                                                                                                         
          state<EmptyCrosswalk>        + event<EventCrosswalkWithPedestriansDetected> / defer                            =  state<CrosswalkPedestrians>,
          state<EmptyCrosswalk>        + event<EventCrosswalkPassed>                                                     = state<NoCrosswalk>,
          state<EmptyCrosswalk>        + sml::on_entry<_> / enterApproachingCrosswalk,                                   
          state<EmptyCrosswalk>        + sml::on_exit<_> / exitApproachingCrosswalk,                                     
                                                                                                                         
          state<CrosswalkPedestrians>  + event<EventCrosswalkFreeDetected> / startDriving                                = state<EmptyCrosswalk>,
          state<CrosswalkPedestrians>  + event<EventCrosswalkWithPedestriansDetected> / crosswalkWithPedestriansDetected,
          state<CrosswalkPedestrians>  + event<EventCarHasStopped>[stoppingIdMatches]                                    = state<WaitingForPedestrians>,

          state<WaitingForPedestrians> + event<EventCrosswalkFreeDetected> / startDriving                                = state<NoCrosswalk>,
          state<WaitingForPedestrians> + sml::on_entry<_> / std::bind(setTimeout<DataCrosswalk>, data_crosswalk_, 5.5),
          state<WaitingForPedestrians> + event<EventTimeUpdate>[ifTimeoutPassed] / (startDriving, addToCompleted)        = state<NoCrosswalk>);
    // clang-format on
    ENABLE_USED_BUT_MARKED_UNUSED_WARNING
  }

 private:
  std::shared_ptr<SharedData> shared_data_;
  std::shared_ptr<DataCrosswalk> data_crosswalk_;
};
}
#endif
