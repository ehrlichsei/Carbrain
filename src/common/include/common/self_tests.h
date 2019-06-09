#include <common/macros.h>

THIRD_PARTY_HEADERS_BEGIN
#include <ros/timer.h>
#include <diagnostic_updater/diagnostic_updater.h>
THIRD_PARTY_HEADERS_END

namespace common
{

// remove this pragmas as soon as diagnostic_updater is fixed.
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wnon-virtual-dtor"
class SelfTests : public diagnostic_updater::Updater
{
public:
  SelfTests(ros::Duration rate = ros::Duration(1.0), const std::string &hardware_id = "brain")
  {
    tests_timer = node.createTimer(rate, &SelfTests::runTests, this);
    setHardwareID(hardware_id);
  }

private:
  ros::NodeHandle node;
  ros::Timer tests_timer;

  void runTests(const ros::TimerEvent& e);
};
#pragma GCC diagnostic pop
}
