#ifndef STATE_TIMEOUTS_H
#define STATE_TIMEOUTS_H

struct StateTimeouts {
  double no_passing_empty_msg_timeout_in_s;
  double approaching_unsure_observation_timeout_in_s;
  double unidentified_object_timeout;
  double waiting_for_redetection_timeout_in_s;
  double waiting_at_crosswalk_timeout_in_s;
  double waiting_at_junction_with_obstacle_timeout_in_s;
  double waiting_at_road_closure_timeout_in_s;
};

#endif
