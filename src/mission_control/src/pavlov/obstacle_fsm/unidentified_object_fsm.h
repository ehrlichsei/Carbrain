#ifndef UNIDENTIFIED_OBJECT_FSM_H
#define UNIDENTIFIED_OBJECT_FSM_H

#include "hl_fsm.h"

struct UnidentifiedObject;

struct UnidentifiedSM {
  auto operator()() const {
    using namespace sml;
    return make_transition_table(
        * "s"_s + "e"_e = X);
  }
};/*
struct NoUnidentifiedObject
    : sc::state<NoUnidentifiedObject, Driving::orthogonal<ORTHO_ID_UNIDENTIFIED_OBJECT>> {
  typedef sc::state<NoUnidentifiedObject, Driving::orthogonal<ORTHO_ID_UNIDENTIFIED_OBJECT>> my_base;
  typedef sc::custom_reaction<EventUnidentifiedObject> reactions;
  NoUnidentifiedObject(my_context ctx) : my_base(ctx) {
    ROS_INFO_THROTTLE(2, "NoUnidentifiedObject entered.");
  }

  ~NoUnidentifiedObject() {
    ROS_INFO_THROTTLE(2, "NoUnidentifiedObject complete.");
  }
  sc::result react(const EventUnidentifiedObject& ev) {
    return transit<UnidentifiedObject>();
    post_event(ev);
  }
};

struct UnidentifiedObject
    : sc::state<UnidentifiedObject, Driving::orthogonal<ORTHO_ID_UNIDENTIFIED_OBJECT>> {
  typedef sc::state<UnidentifiedObject, Driving::orthogonal<ORTHO_ID_UNIDENTIFIED_OBJECT>> my_base;
  typedef boost::mpl::list<sc::custom_reaction<EventUnidentifiedObject>, sc::custom_reaction<EventNoUnidentifiedObject>> reactions;
  UnidentifiedObject(my_context ctx) : my_base(ctx), max_speed_id(0) {
    ROS_INFO_THROTTLE(2, "UnidentifiedObject entered.");
    const double lowest_velocity =
        context<ObstacleStateMachine>().getUnidentifiedObjectParams().lowest_velocity;
    max_speed_id = context<ObstacleStateMachine>().getCarController().setMaxSpeedInMPerS(
        lowest_velocity, 0);
  }
  ~UnidentifiedObject() {
    context<ObstacleStateMachine>().getCarController().clearMaxSpeed(max_speed_id);
    ROS_INFO_THROTTLE(2, "UnidentifiedObject complete.");
  }
  sc::result react(const EventUnidentifiedObject& /*ev*/) {
    if (timeout.isEnabled()) {
      // Disable timeout.
      timeout = Timeout();
    }

    return discard_event();
  }

  sc::result react(const EventNoUnidentifiedObject& ev) {
    if (!timeout.isEnabled()) {
      // Enable timeout.
      double timeout_in_s = context<ObstacleStateMachine>().getStateTimeouts().unidentified_object_timeout;
      timeout = Timeout(ev.stamp, ros::Duration(timeout_in_s));
      return discard_event();
    }
    timeout.update(ev.stamp);
    if (timeout.hasPassed()) {
      return transit<NoUnidentifiedObject>();
    }
    return discard_event();
  }
  unsigned long max_speed_id;
  Timeout timeout;
};

*/

#endif
