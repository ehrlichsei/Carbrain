#include "common/realtimecontroller.h"
#include "common/realtime_utils.h"
#include <common/macros.h>

THIRD_PARTY_HEADERS_BEGIN
#include <boost/chrono.hpp>
#include <boost/thread.hpp>
#include <ros/ros.h>
#include <sched.h>
#include <sys/mman.h>
THIRD_PARTY_HEADERS_END

namespace common {
namespace realtime {

RealtimeController::RealtimeController(int rate,
                                       long long offset_microseconds,
                                       const ParameterInterface *parameters)
    : parameter_interface_(parameters),
      update_period_(1'000'000'000 / rate),
      latency_max_(0),
      latency_min_(0),
      reference_time(boost::chrono::microseconds(offset_microseconds)) {}

// In general it is a very good practice to have not-throwing destructors.
// In this special case it does not really matter though, because this
// destructor should only be called during shutdown. If boost::thread throws
// an exception the operating system will take care of the remains and clean
// them up.
// NOLINTNEXTLINE(bugprone-exception-escape)
RealtimeController::~RealtimeController() {
  control_thread_.interrupt();
  parameter_poll_thread_.interrupt();
  control_thread_.join();
  parameter_poll_thread_.join();
  stopControlThread();
}

void RealtimeController::scheduleFixedIntervalUpdates() {
  common::realtime::prefaultStack<FAULT_SAFE_STACK_SIZE>();
  common::realtime::elevateThreadRTPriority(REALTIME_PRIORITY);

  while (!boost::this_thread::interruption_requested()) {
    const auto next_update = getNextScheduleInstant();
    boost::this_thread::sleep_until(next_update);

    const auto scheduling_latency =
        boost::chrono::duration_cast<boost::chrono::microseconds>(wall_clock::now() - next_update);
    update(update_period_);
    //! @TODO maybe pass the measured value -
    // although it should not matter on PREEMPT RT
    // kernel.

    latency_max_ = std::max(scheduling_latency, latency_max_);
    latency_min_ = std::min(scheduling_latency, latency_min_);
  }
}

instant RealtimeController::getNextScheduleInstant() const {
  const instant current_time = wall_clock::now();
  const auto time_running = current_time - reference_time;
  const auto time_used = time_running % update_period_;
  return current_time + update_period_ - time_used;
}

void RealtimeController::scheduleParameterPolling() {
  while (!boost::this_thread::interruption_requested()) {
    boost::this_thread::sleep_for(boost::chrono::seconds(1));
    pollParameters();
  }
}

boost::chrono::microseconds RealtimeController::getRemainingTimeSliceMicroseconds() const {
  const instant next_invocation = getNextScheduleInstant();
  return boost::chrono::duration_cast<boost::chrono::microseconds>(
      next_invocation - wall_clock::now());
}

boost::chrono::microseconds RealtimeController::getTimeSliceMicroseconds() const {
  const instant current_time = wall_clock::now();
  const auto time_running = current_time - reference_time;
  const auto time_used = time_running % update_period_;
  return boost::chrono::duration_cast<boost::chrono::microseconds>(time_used);
}

int RealtimeController::getUpdateRate() const {
  return 1'000'000'000 / static_cast<int>(update_period_.count());
}

void RealtimeController::startControlThread() {
  if (!common::realtime::lockPages()) {
    ROS_ERROR(
        "Could not lock memory pages. Check privileges. Real-time guarantees "
        "may suffer.");
  }
  control_thread_ =
      boost::thread(boost::bind(&RealtimeController::scheduleFixedIntervalUpdates, this));
  parameter_poll_thread_ =
      boost::thread(boost::bind(&RealtimeController::scheduleParameterPolling, this));
}


void RealtimeController::stopControlThread() {
  if (!common::realtime::unlockPages()) {
    ROS_ERROR("Could not unlock memory pages.");
  }
  control_thread_.interrupt();
  parameter_poll_thread_.interrupt();
}

}  // namespace realtime
}  // namespace common
