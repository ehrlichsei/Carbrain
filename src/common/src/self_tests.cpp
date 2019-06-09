#include <common/macros.h>

THIRD_PARTY_HEADERS_BEGIN
#include <ros/ros.h>
THIRD_PARTY_HEADERS_END

#include "common/self_tests.h"

namespace common
{

/*!
 * \brief Timer callback to call pending self tests regularly
 * \param e TimerEvent
 */
void SelfTests::runTests(UNUSED const ros::TimerEvent& e)
{
  ROS_DEBUG("run self tests");
  force_update();
}

} // namespace common
