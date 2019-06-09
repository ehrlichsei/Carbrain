#include <common/macros.h>

THIRD_PARTY_HEADERS_BEGIN
#include <gtest/gtest.h>
THIRD_PARTY_HEADERS_END

#include "../../../src/pavlov/fsm_utils/timeout.h"

DISABLE_SUGGEST_OVERRIDE_WARNING

// Test fixture
class DisabledTimeoutTest : public ::testing::Test {
  protected:
    Timeout testee;
};

TEST_F(DisabledTimeoutTest, isEnabledAndHasPassed_WhileNotEnabled_ReturnFalse) {
  ASSERT_FALSE(testee.isEnabled());
  ASSERT_FALSE(testee.hasPassed());
}

TEST_F(DisabledTimeoutTest, update_WhileNotEnabled_DoesNotChangeObject) {
  testee.update(ros::Time(1000000));

  ASSERT_FALSE(testee.isEnabled());
  ASSERT_FALSE(testee.hasPassed());
}

TEST(EnabledTimeoutTest, enabledCtor_OnCall_InitializesObjectCorrectly) {
  Timeout testee(ros::Time(100), ros::Duration(10));
  
  ASSERT_TRUE(testee.isEnabled());
  ASSERT_FALSE(testee.hasPassed());
}

TEST(EnabledTimeoutTest, hasPassed_AfterTimeoutHasNotPassedUpdate_ReturnsTrue) {
  Timeout testee(ros::Time(100), ros::Duration(10));

  testee.update(ros::Time(101));
  ASSERT_FALSE(testee.hasPassed());
}

TEST(EnabledTimeoutTest, hasPassed_AfterTimeoutHasPassedUpdate_ReturnsTrue) {
  Timeout testee(ros::Time(100), ros::Duration(10));

  testee.update(ros::Time(111));
  ASSERT_TRUE(testee.hasPassed());
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
