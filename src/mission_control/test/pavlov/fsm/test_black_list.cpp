#include <common/macros.h>

THIRD_PARTY_HEADERS_BEGIN
#include <gtest/gtest.h>
THIRD_PARTY_HEADERS_END

#include "../../../src/pavlov/fsm/black_list.h"

DISABLE_SUGGEST_OVERRIDE_WARNING

// Test fixture
class BlackListTest : public ::testing::Test {
 public:
  BlackListTest()
      : testee(ros::Duration(5)), dt_not_expired(3), dt_expired(6) {}

 protected:
  BlackList testee;
  const ros::Duration dt_not_expired;
  const ros::Duration dt_expired;
};

TEST_F(BlackListTest, isBlocked_AfterAdding_ReturnsTrue) {
  testee.add(ros::Time(0), Entity::CROSSWALK, 0);

  ASSERT_TRUE(testee.isBlocked(Entity::CROSSWALK, 0));
}

TEST_F(BlackListTest, isBlocked_BeforeTimeoutExpired_ReturnsTrue) {
  testee.add(ros::Time(0), Entity::CROSSWALK, 0);

  testee.update(ros::Time(0) + dt_not_expired);

  ASSERT_TRUE(testee.isBlocked(Entity::CROSSWALK, 0));
}

TEST_F(BlackListTest, isBlocked_BeforeMultipleTimeoutExpired_ReturnsTrue) {
  testee.add(ros::Time(0), Entity::CROSSWALK, 0);
  testee.add(ros::Time(0), Entity::CROSSWALK, 1);
  testee.add(ros::Time(0), Entity::ROAD_CLOSURE, 0);

  testee.update(ros::Time(0) + dt_not_expired);

  ASSERT_TRUE(testee.isBlocked(Entity::CROSSWALK, 0));
  ASSERT_TRUE(testee.isBlocked(Entity::CROSSWALK, 1));
  ASSERT_TRUE(testee.isBlocked(Entity::ROAD_CLOSURE, 0));
}

TEST_F(BlackListTest, isBlocked_AfterTimeoutExpired_ReturnsFalse) {
  testee.add(ros::Time(0), Entity::CROSSWALK, 0);

  testee.update(ros::Time(0) + dt_expired);

  ASSERT_FALSE(testee.isBlocked(Entity::CROSSWALK, 0));
}

TEST_F(BlackListTest, isBlocked_AfterMultipleTimeoutExpired_ReturnsFalse) {
  testee.add(ros::Time(0), Entity::CROSSWALK, 0);
  testee.add(ros::Time(0), Entity::CROSSWALK, 1);
  testee.add(ros::Time(0), Entity::ROAD_CLOSURE, 0);

  testee.update(ros::Time(0) + dt_expired);

  ASSERT_FALSE(testee.isBlocked(Entity::CROSSWALK, 0));
  ASSERT_FALSE(testee.isBlocked(Entity::CROSSWALK, 1));
  ASSERT_FALSE(testee.isBlocked(Entity::ROAD_CLOSURE, 0));
}

TEST_F(BlackListTest, isBlocked_AfterMixedTimeoutExpired_ReturnsCorrect) {
  testee.add(ros::Time(0), Entity::CROSSWALK, 0);
  testee.add(ros::Time(0), Entity::ROAD_CLOSURE, 0);
  testee.add(ros::Time(0) + dt_expired, Entity::ROAD_CLOSURE, 1);

  testee.update(ros::Time(0) + dt_expired);

  ASSERT_FALSE(testee.isBlocked(Entity::CROSSWALK, 0));
  ASSERT_FALSE(testee.isBlocked(Entity::ROAD_CLOSURE, 0));
  ASSERT_TRUE(testee.isBlocked(Entity::ROAD_CLOSURE, 1));
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
