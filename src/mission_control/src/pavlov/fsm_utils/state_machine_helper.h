#ifndef STATE_MACHINE_HELPER_H
#define STATE_MACHINE_HELPER_H

#include "carcontroller.h"
#include "timeout.h"
#include "../obstacle_fsm/shared_data.h"

THIRD_PARTY_HEADERS_BEGIN
#include <mission_control_msgs/StateMachineStates.h>
THIRD_PARTY_HEADERS_END

namespace sml = boost::sml;
namespace SM {
struct VisualizationState {
  virtual std::string getBase() { return "none"; }
  virtual std::string getName() { return "none"; }

  mission_control_msgs::StateMachineState toMsg() {
    mission_control_msgs::StateMachineState state;
    state.name = getName();
    state.base_state_name = getBase();
    return state;
  }
  virtual ~VisualizationState(){};
};

template <class Data>
struct AddToCompleted {
  void operator()(const std::shared_ptr<Data> data) {
    data->completed_.insert(data->current_id_);
  }
};

template <class Data>
struct StartDriving {
  void operator()(const std::shared_ptr<Data> data,
                  const std::shared_ptr<SharedData> shared_data) {
    shared_data->getCarController().startDriving(data->stopping_id_);
    data->stopping_id_ = 0;
  }
};
template <class Data>
struct StartDrivingIfStoppingIdNotZero {
  void operator()(const std::shared_ptr<Data> data,
                  const std::shared_ptr<SharedData> shared_data) {
    if (data->stopping_id_ != 0) {
      StartDriving<Data> startDriving;
      startDriving(data, shared_data);
    }
  }
};
template <class Data>
struct SetStoppingIdToZero {
  void operator()(const std::shared_ptr<Data> data) { data->stopping_id_ = 0; }
};

template <class Data>
struct StoppingIdMatches {
  bool operator()(const std::shared_ptr<Data> data, const EventCarHasStopped& ev) {
    return ev.stopping_id == 0 || data->stopping_id_ == ev.stopping_id;
  }
};
/*const auto startDriving =
    [](unsigned long& stopping_id, const std::shared_ptr<SharedData>&
   shared_data) {
      shared_data->getCarController().startDriving(stopping_id);
      stopping_id = 0;
    };*/

template <class Data>
void setTimeout(const std::shared_ptr<Data> data, const double duration) {
  data->timeout_ = Timeout(ros::Time::now(), ros::Duration(duration));
}
template <class Data>
void setNewDuration(const std::shared_ptr<Data> data, const double duration) {
  data->timeout_.setNewDuration(ros::Time::now(), ros::Duration(duration));
}

template <class Data>
struct IfTimeoutPassed {
  bool operator()(const std::shared_ptr<Data> data, const EventTimeUpdate& ev) {
    data->timeout_.update(ev.current_time);
    return data->timeout_.hasPassed();
  }
};
}

#endif  // STATE_MACHINE_HELPER_H
