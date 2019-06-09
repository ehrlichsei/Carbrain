#ifndef REALTIME_UTILS
#define REALTIME_UTILS

#include <common/macros.h>

THIRD_PARTY_HEADERS_BEGIN
#include <sched.h>
#include <cstdlib>
#include <cstdio>
#include <sys/mman.h>
#include <cstring>
#include <ros/console.h>
THIRD_PARTY_HEADERS_END

namespace common {
/*!
 * \namespace common::realtime
 * \brief realtime contains some tools to deal with realtime processing and
 * communication.
 */
namespace realtime {

/*!
 * \brief elevateThreadRTPriority sets the RT-priority of the current thread.
 * \param realtime_priority the new priority of this thread.
 */
static void elevateThreadRTPriority(int realtime_priority) {
  struct sched_param param;
  param.sched_priority = realtime_priority;

  int success = sched_setscheduler(0, SCHED_RR, &param);
  if (success == -1) {
    ROS_ERROR(
        "Could not set realtime priority. Check privileges. Can not continue.");
  }
}

/*!
 * \brief lockPages pins all current and future pages in memory so they cannot
 * be swapped out.
 */
static bool lockPages() {
  const int lock_result = mlockall(MCL_CURRENT | MCL_FUTURE);
  return lock_result != -1;
}

/*!
 * \brief unlockPages unlock the pages locked by lockPages().
 */
static bool unlockPages() {
  const int unlock_result = munlockall();
  return unlock_result != -1;
}

template <size_t prefault_size>
static void prefaultStack() {
  char stack[prefault_size];
  memset(stack, 0, prefault_size);
}

}  // namespace realtime
}  // namepsace common

#endif  // REALTIME_UTILS
