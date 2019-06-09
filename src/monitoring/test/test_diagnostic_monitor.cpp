#include <common/macros.h>

THIRD_PARTY_HEADERS_BEGIN
#include <gtest/gtest.h>
THIRD_PARTY_HEADERS_END

#include "../src/diagnostic_monitor/diagnostic_monitor.h"

DISABLE_SUGGEST_OVERRIDE_WARNING

class DiagnosticMonitorTest : public ::testing::Test {
 public:
  OptionalDiagnosticStatus status_empty = boost::none;

  static OptionalDiagnosticStatus getStatusOne() {
    OptionalDiagnosticStatus status = diagnostic_msgs::DiagnosticStatus();
    status->level = diagnostic_msgs::DiagnosticStatus::OK;
    status->name = "test one";
    status->message = "this is a test";
    return status;
  }

  static OptionalDiagnosticStatus getStatusTwo() {
    OptionalDiagnosticStatus status = diagnostic_msgs::DiagnosticStatus();
    status->level = diagnostic_msgs::DiagnosticStatus::OK;
    status->name = "test two";
    status->message = "this is an other test";
    return status;
  }

  static OptionalDiagnosticStatus getStatusWarn() {
    OptionalDiagnosticStatus status = diagnostic_msgs::DiagnosticStatus();
    status->level = diagnostic_msgs::DiagnosticStatus::WARN;
    status->name = "test warning";
    status->message = "this is a warning";
    return status;
  }
};

/**!
* This is a placeholder for you REAL implementation unit tests.
*/
TEST_F(DiagnosticMonitorTest, compareStatus) {
  ASSERT_TRUE(DiagnosticMonitor::equal(getStatusOne(), getStatusOne()));
  ASSERT_FALSE(DiagnosticMonitor::equal(getStatusOne(), getStatusTwo()));
  ASSERT_FALSE(DiagnosticMonitor::equal(status_empty, getStatusTwo()));
  ASSERT_FALSE(DiagnosticMonitor::equal(getStatusOne(), status_empty));
}



// nothing to do here
int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
