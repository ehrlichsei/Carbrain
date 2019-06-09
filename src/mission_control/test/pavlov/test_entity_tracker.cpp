#include <common/macros.h>

THIRD_PARTY_HEADERS_BEGIN
#include <gtest/gtest.h>
#include <perception_msgs/Junctions.h>
THIRD_PARTY_HEADERS_END

#include "common/test/dummy_parameter_handler.h"

#include "../../src/pavlov/entity_tracker.h"

DISABLE_SUGGEST_OVERRIDE_WARNING

using Msg = perception_msgs::Junction;

// For assertions.
namespace perception_msgs {
bool operator==(const perception_msgs::Junction& m1, const perception_msgs::Junction& m2) {
  return m1.id == m2.id && m1.header.stamp == m2.header.stamp;
}
}

class EntityTrackerTest : public ::testing::Test {
 public:
  EntityTrackerTest()
      : dummy_parameter_handler(setupDummyParameterHandler()),
        testee(&dummy_parameter_handler) {}

 protected:
  Msg createMsg(const long id, const double stamp_in_s) {
    Msg m;
    m.id = id;
    m.header.stamp = ros::Time(stamp_in_s);

    return m;
  }

  static DummyParameterHandler setupDummyParameterHandler() {
    DummyParameterHandler parameter_handler;
    parameter_handler.addParam(
        ParameterString<double>("entity_tracking_timeout_in_s"), 5.0);
    return parameter_handler;
  }

  DummyParameterHandler dummy_parameter_handler;
  EntityTracker<Msg> testee;
};

TEST_F(EntityTrackerTest, getTrackedEntities_WithinTrackingDuration_ReturnsNewEntities) {
  Msg msg = createMsg(0, 0);
  testee.updateTrackedEntities(ros::Time(0.0), {{msg}});
  auto tracked = testee.getTrackedEntities();

  ASSERT_EQ(1, tracked.size());
  ASSERT_EQ(msg, tracked[0]);

  testee.updateTrackedEntities(ros::Time(4.0), {{}});
  tracked = testee.getTrackedEntities();

  ASSERT_EQ(1, tracked.size());
  ASSERT_EQ(msg, tracked[0]);
}

TEST_F(EntityTrackerTest, getTrackedEntities_AfterTrackingDuration_ReturnsEmpty) {
  Msg msg = createMsg(0, 0);
  testee.updateTrackedEntities(ros::Time(0.0), {{msg}});
  auto tracked = testee.getTrackedEntities();

  testee.updateTrackedEntities(ros::Time(6.0), {{}});
  tracked = testee.getTrackedEntities();

  ASSERT_EQ(0, tracked.size());
}

TEST_F(EntityTrackerTest, getTrackedEntities_MultipleMixedEntities_RemovesOnlyOldOnes) {
  Msg msg1 = createMsg(0, 0);
  Msg msg2 = createMsg(1, 0);
  Msg msg3 = createMsg(2, 0);
  testee.updateTrackedEntities(ros::Time(0.0), {{msg1, msg2, msg3}});

  msg2.header.stamp = ros::Time(5);
  testee.updateTrackedEntities(ros::Time(5.0), {{msg2}});

  testee.updateTrackedEntities(ros::Time(6.0), {{}});
  auto tracked = testee.getTrackedEntities();

  ASSERT_EQ(1, tracked.size());
  ASSERT_EQ(msg2, tracked[0]);
}

TEST_F(EntityTrackerTest, getTrackedEntities_WithLockedEntities_DoesNotRemoveLocked) {
  Msg msg1 = createMsg(0, 0);
  Msg msg2 = createMsg(1, 0);
  testee.updateTrackedEntities(ros::Time(0.0), {{msg1, msg2}});

  testee.lockEntity(0);
  testee.lockEntity(1);
  testee.unlockEntity(1);

  testee.updateTrackedEntities(ros::Time(7.0), {{msg1, msg2}});
  auto tracked = testee.getTrackedEntities();

  ASSERT_EQ(1, tracked.size());
  ASSERT_EQ(msg1, tracked[0]);
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
