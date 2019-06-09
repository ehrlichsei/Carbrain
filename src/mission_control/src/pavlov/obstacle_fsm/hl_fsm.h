#ifndef FSM_HIGH_LEVEL_FSM
#define FSM_HIGH_LEVEL_FSM

#include "common/macros.h"

THIRD_PARTY_HEADERS_BEGIN
#include <algorithm>
#include <limits>
#include <ros/ros.h>
#include <boost_sml_catkin/sml.hpp>
THIRD_PARTY_HEADERS_END

#include "common/parameter_interface.h"
#include "../fsm_utils/carcontroller.h"
#include "../fsm_utils/diagnostics_interface.h"
#include "../fsm_utils/events.h"
#include "../fsm_utils/safety_margins.h"
#include "../fsm_utils/state_timeouts.h"
#include "../fsm_utils/timeout.h"
#include "../fsm_utils/unidentified_object_params.h"
#include "../obstacle_fsm/shared_data.h"

#include "../fsm_utils/state_machine_helper.h"
#include "../fsm_utils/qr_code_fsm.h"

namespace SM {
struct DrivingVisualizationState : VisualizationState {
  virtual std::string getBase() override { return "root"; }
  virtual std::string getName() override { return "driving"; }
};
struct DrivingSubStateVisualization : VisualizationState {
  virtual std::string getBase() override { return "driving"; }
};
}

#include "crosswalk_fsm.h"
#include "junction_fsm.h"
#include "no_passing_fsm.h"
#include "road_closure_fsm.h"
#include "speed_limit_fsm.h"
#include "turn_fsm.h"
//#include "unidentified_object_fsm.h"


namespace sml = boost::sml;
namespace SM {
struct NoRoadClosureHL : public DrivingSubStateVisualization {
  std::string getName() override { return "no road closure hl"; }
};
struct NoJunctionHL : public DrivingSubStateVisualization {
  std::string getName() override { return "no junction hl"; }
};
struct NoCrosswalkHL : public DrivingSubStateVisualization {
  std::string getName() override { return "no crosswalk hl"; }
};
struct Driving : public DrivingVisualizationState {
  auto operator()() const {
    const auto reactToRoadClosureBlocked =
        [](const std::shared_ptr<DataRoadClosure> data_road_closure,
           const EventRoadClosureBlockedDetected& event) {
          return data_road_closure->completed_.count(event.id) == 0;
        };
    AddToCompleted<DataRoadClosure> addCurrentRoadClosureToCompleted;
    using namespace sml;
    // clang-format off
    DISABLE_USED_BUT_MARKED_UNUSED_WARNING
    return make_transition_table(
        /* * "error_handler"_s +
            unexpected_event<_> /
                [] { ROS_ERROR_STREAM("unexpeceted Event in Driving"); } =
            "error_handler"_s,*/
        * state<NoRoadClosureHL> + event<EventRoadClosureBlockedDetected>[reactToRoadClosureBlocked] / defer = state<RoadClosureSM>,
          state<RoadClosureSM>   + event<EventNoRoadClosure>           /addCurrentRoadClosureToCompleted      = state<NoRoadClosureHL>,
          state<RoadClosureSM>   + event<EventRoadClosureFreeDetected> /addCurrentRoadClosureToCompleted      = state<NoRoadClosureHL>,

        * state<NoJunctionHL>    + event<EventStopJunction> / defer                                          = state<JunctionSM>,
          state<NoJunctionHL>    + event<EventHoldJunction> / defer                                          = state<JunctionSM>,
          state<JunctionSM>      + event<EventNoJunction>                                                    = state<NoJunctionHL>,

        * state<NoCrosswalkHL>   + event<EventCrosswalkWithPedestriansDetected> / defer                      = state<CrosswalkSM>,
          state<NoCrosswalkHL>   + event<EventCrosswalkFreeDetected> / defer                                 = state<CrosswalkSM>,
          state<CrosswalkSM>     + event<EventNoCrosswalk>                                                   = state<NoCrosswalkHL>,

        * state<TurningSM>       + "not_existing_event"_e                                                    = state<TurningSM>,

        * state<PassingSM>       + "not_existing_event"_e                                                    = state<PassingSM>,

        * state<SpeedLimitSM>    + "not_existing_event"_e                                                    = state<SpeedLimitSM>);
    //* state<UnidentifiedSM> + "not_existing_event"_e =
    // state<UnidentifiedSM>);
    // clang-format on
    ENABLE_USED_BUT_MARKED_UNUSED_WARNING
  }
};


struct ObstacleStateMachine {
  auto operator()() const {
    using namespace sml;
    return make_transition_table(*state<QRCodeFSM<SharedData, Driving>> +
                                     "not_exisitng_event"_e =
                                     state<QRCodeFSM<SharedData, Driving>>);
  }
};
}

#endif
