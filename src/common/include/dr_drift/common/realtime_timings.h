#ifndef REALTIME_TIMINGS_H
#define REALTIME_TIMINGS_H

/*!
 * \brief The RealtimeTimings struct defines the timings within the low-level
 * realtime loop.
 *
 * **NOTE:** All timings are given in _MICRO_ seconds.
 */
struct RealtimeTimings {
  /*!
   * \brief CONTROLLER_INTERFACE_OFFSET the offset of the controller interface
   * in the realtime loop. This variabe exists currently only for consistency
   * reasons.
   */
  static constexpr int CONTROLLER_INTERFACE_OFFSET = 0;
  static_assert(CONTROLLER_INTERFACE_OFFSET >= 0, "");
  /*!
   * \brief CONTROLLER_INTERFACE_MEASUREMENT_CYCLE_TIME the time for a
   * measurement to be retrieved.
   */
  static constexpr int CONTROLLER_INTERFACE_MEASUREMENT_CYCLE_TIME = 1200;
  static_assert(CONTROLLER_INTERFACE_MEASUREMENT_CYCLE_TIME >= 0, "");
  /*!
   * \brief CONTROLLER_INTERFACE_COMMAND_CYCLE_TIME the time for the command to
   * reach the bus-interface.
   */
  static constexpr int CONTROLLER_INTERFACE_COMMAND_CYCLE_TIME = 2000;
  static_assert(CONTROLLER_INTERFACE_COMMAND_CYCLE_TIME  >= 0, "");

  /*!
   * \brief STATE_ESTIMATION_OFFSET the offset of the state_estimation in the
   * realtime-loop.
   */
  static constexpr int STATE_ESTIMATION_OFFSET =
      CONTROLLER_INTERFACE_OFFSET + CONTROLLER_INTERFACE_MEASUREMENT_CYCLE_TIME;
  static_assert(STATE_ESTIMATION_OFFSET  >= 0, "");
  /*!
   * \brief STATE_ESTIMATION_CYCLE_TIME the time the state estimation may take
   * to compute an estimation.
   */
  static constexpr int STATE_ESTIMATION_CYCLE_TIME = 200;
  static_assert(STATE_ESTIMATION_CYCLE_TIME  >= 0, "");

  /*!
   * \brief LOWLEVEL_CONTROLLER_CYCLE_TIME the time the low-level-controllers
   * may
   * need to generate their outputs.
   */
  static constexpr int LOWLEVEL_CONTROLLER_CYCLE_TIME = 200;
  static_assert(LOWLEVEL_CONTROLLER_CYCLE_TIME  >= 0, "");
  /*!
   * \brief LOWLEVEL_CONTROLLER_WAIT_TIME the time the low-level-controllers are
   * waiting to reviece the new set-point from the high-level-controllers. If no
   * waiting is desired, this value should be 0!
   */
  static constexpr int LOWLEVEL_CONTROLLER_WAIT_TIME = 4000;
  static_assert(LOWLEVEL_CONTROLLER_WAIT_TIME >= 0, "");
  /*!
   * \brief LOWLEVEL_CONTROLLER_OFFSET the offset of the low-level-controllers
   * in the realtime-loop.
   */
  static constexpr int LOWLEVEL_CONTROLLER_OFFSET =
      STATE_ESTIMATION_OFFSET + STATE_ESTIMATION_CYCLE_TIME + LOWLEVEL_CONTROLLER_WAIT_TIME;
  static_assert(LOWLEVEL_CONTROLLER_OFFSET  >= 0, "");

  /*!
   * \brief CONTROLLER_INTERFACE_COMMAND_WAIT_TIME the time in the realtie-loop
   * at which the controller sends the new command to the bus-interface.
   */
  static constexpr int CONTROLLER_INTERFACE_COMMAND_WAIT_TIME =
      STATE_ESTIMATION_CYCLE_TIME + LOWLEVEL_CONTROLLER_CYCLE_TIME + LOWLEVEL_CONTROLLER_WAIT_TIME;
  static_assert(CONTROLLER_INTERFACE_COMMAND_WAIT_TIME  >= 0, "");

  /*!
   * \brief COMPLETE_CYCLE_TIME the time one realtime-cycle needs at least.
   */
  static constexpr int COMPLETE_CYCLE_TIME =
      CONTROLLER_INTERFACE_OFFSET + CONTROLLER_INTERFACE_MEASUREMENT_CYCLE_TIME +
      CONTROLLER_INTERFACE_COMMAND_WAIT_TIME + CONTROLLER_INTERFACE_COMMAND_CYCLE_TIME;
  static_assert(COMPLETE_CYCLE_TIME  >= 0, "");

  /*!
   * \brief LOOP_RATE the realtime-loop-rate.
   */
  static constexpr int LOOP_RATE = 120;
  static_assert(LOOP_RATE  > 0, "");

  /*
   * Consistnecy checks
   */
  static_assert((1000000.0 / LOOP_RATE) > COMPLETE_CYCLE_TIME, "");
  static_assert(STATE_ESTIMATION_OFFSET > CONTROLLER_INTERFACE_OFFSET, "");
  static_assert(LOWLEVEL_CONTROLLER_OFFSET > STATE_ESTIMATION_OFFSET, "");
};

#endif  // REALTIME_TIMINGS_H
