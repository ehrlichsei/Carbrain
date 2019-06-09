#ifndef REALTIMECONTROLLER_H
#define REALTIMECONTROLLER_H

#include "common/parameter_interface.h"
#include <boost/thread.hpp>

#define REALTIME_PRIORITY 49
#define FAULT_SAFE_STACK_SIZE (1024 * 1024)

namespace common {
namespace realtime {
typedef boost::chrono::time_point<boost::chrono::high_resolution_clock> instant;
typedef boost::chrono::high_resolution_clock wall_clock;

/**
 * @brief The RealtimeController class is an abstract class that makes writing
 * fixed-interval real-time
 * code such as controllers easier and manages most of the ugly details.
 * It sets up a control thread that is scheduled with real-time priorities,
 * provides a fault safe stack and
 * monitors the scheduling latencies.
 * Implementors can overwrite update() to execute the time-critical code.
 */
class RealtimeController {
 public:
  RealtimeController(int rate, long long offset_microseconds, const ParameterInterface* parameters);

  /*!
   * \brief ~RealtimeController destructor, interrupts and joins all threads
   * cleanly.
   */
  virtual ~RealtimeController() = 0;

  /**
   * @brief update updates the controller values
   * this is the realtime loop function that gets called in fixed intervals.
   * Time spent in this function MUST be smaller than the loop interval!
   */
  virtual void update(boost::chrono::nanoseconds) = 0;

  /**
   * @brief pollParameters is a non-realtime method that gets called at regular
   * intervals.
   * it can be used to perform parameter updates and to interact with other
   * non-realtime components.
   */
  virtual void pollParameters() = 0;

  /**
   * @brief scheduleFixedIntervalUpdates runs in a separate real-time thread and
   * calls the update() method at the configured intervals.
   */
  void scheduleFixedIntervalUpdates();

  /*!
   * \brief scheduleParameterPolling runs in a separate low-priority thread and
   * calls pollParameters() each second
   */
  void scheduleParameterPolling();

  /**
   * @brief startControlThread starts a realtime thread that calls update() at
   * fixed intervals
   */
  void startControlThread();

  /**
   * @brief stopControlThread stops the controller thread
   */
  void stopControlThread();

  /**
   * @brief getJitter returns the maximum jitter in microseconds
   */
  long getJitter() const { return (latency_max_ - latency_min_).count(); }

  /**
   * @brief getRemainingTimeSliceMicroseconds returns the number of microseconds
   * that remain before the next update() invocation will be scheduled.
   */
  boost::chrono::microseconds getRemainingTimeSliceMicroseconds() const;
  boost::chrono::microseconds getTimeSliceMicroseconds() const;

  /**
   * @brief getUpdateRate returns the update rate used for this realtime
   * controller
   * @return the rate in which update() is scheduled
   */
  int getUpdateRate() const;

 protected:
  const ParameterInterface* parameter_interface_;
  boost::thread control_thread_;
  boost::thread parameter_poll_thread_;
  boost::chrono::nanoseconds update_period_;

  /*!
   * \brief latency_max_usec_ stores the maximum scheduling latency in
   * microseconds
   */
  boost::chrono::microseconds latency_max_;

  /*!
   * \brief latency_min_usec_ stores the minimum scheduling latency in
   * microseconds
   */
  boost::chrono::microseconds latency_min_;


 private:
  instant getNextScheduleInstant() const;
  instant reference_time;
};

}  // namespsace realtime
using namespace realtime;
}  // namespace common

#endif  // REALTIMECONTROLLER_H
