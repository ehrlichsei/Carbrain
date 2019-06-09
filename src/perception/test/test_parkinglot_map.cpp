#include <common/macros.h>

THIRD_PARTY_HEADERS_BEGIN
#include <gtest/gtest.h>
THIRD_PARTY_HEADERS_END

#include "../src/parkinglot_mapping/parkinglot_map.h"

DISABLE_SUGGEST_OVERRIDE_WARNING

namespace {

// Test fixture
class ParkinglotMapTest3x2 : public ::testing::Test {
 protected:
  const int WIDTH_IN_PARKINGLOT = 3;
  const int HEIGHT_IN_PARKINGLOT = 2;
  const float RESOLUTION = 1;
  const float DEFAULT_VALUE = 0.5;

  ParkinglotMapTest3x2()
      : ::testing::Test(),
        testee(WIDTH_IN_PARKINGLOT, HEIGHT_IN_PARKINGLOT, RESOLUTION, DEFAULT_VALUE) {}

  ParkinglotMap testee;
};

// Test fixture
class ParkinglotMapTest3x3 : public ::testing::Test {
 protected:
  const int WIDTH_IN_PARKINGLOT = 3;
  const int HEIGHT_IN_PARKINGLOT = 3;
  const float RESOLUTION = 1;
  const float DEFAULT_VALUE = 0.5;

  ParkinglotMapTest3x3()
      : ::testing::Test(),
        testee(WIDTH_IN_PARKINGLOT, HEIGHT_IN_PARKINGLOT, RESOLUTION, DEFAULT_VALUE) {}

  ParkinglotMap testee;
};

TEST_F(ParkinglotMapTest3x2, Ctor_OnCall_SetsDefaultValues) {
  float expected = DEFAULT_VALUE;
  for (int x = 0; x < WIDTH_IN_PARKINGLOT; ++x) {
    for (int y = 0; y < HEIGHT_IN_PARKINGLOT; ++y) {
      float actual = testee.getData(x, y);
      ASSERT_FLOAT_EQ(expected, actual);
    }
  }
}

TEST_F(ParkinglotMapTest3x2, GetData_WithYOutOfRange_ReturnsDefaultValue) {
  float data_value = testee.getData(0, -1);

  ASSERT_FLOAT_EQ(DEFAULT_VALUE, data_value);
}

TEST_F(ParkinglotMapTest3x2, UpdateData_MultipleTimes_SetsCorrectValue) {
  // These values and updates were calculated in excel before.
  const std::vector<float> values = {
      0.5f, 0.5f, 0.1f, 0.012195122f, 0.1f, 0.5f, 0.5f, 0.5f, 0.9f, 0.987804878f};
  const std::vector<float> updates = {0.5f, 0.1f, 0.1f, 0.9f, 0.9f, 0.5f, 0.5f, 0.9f, 0.9f};

  // Precondition
  ASSERT_EQ(values.size() - 1, updates.size());
  int x_test = 0;
  int y_test = 0;
  ASSERT_FLOAT_EQ(values[0], testee.getData(x_test, y_test));
  for (unsigned int i = 0; i < updates.size(); ++i) {
    testee.updateData(x_test, y_test, updates[i]);
    ASSERT_FLOAT_EQ(values[i + 1], testee.getData(x_test, y_test));
  }
}

TEST_F(ParkinglotMapTest3x2, UpdateData_WithinRange_ReturnsTrue) {
  bool updated = testee.updateData(0, 0, 0.1f);
  ASSERT_EQ(true, updated);
}
//! This tests for numerical instabilities
TEST_F(ParkinglotMapTest3x2, UpdateData_RepeatedUpdates_Grow_SetsCorrectValue) {
  const int NUM_ITERATIONS = 10000;
  int x_test = 0;
  int y_test = 0;

  float last_value = testee.getData(x_test, y_test);

  // Let the values grow
  for (unsigned int i = 0; i < NUM_ITERATIONS; ++i) {
    testee.updateData(x_test, y_test, 0.7);
    float updated_value = testee.getData(x_test, y_test);
    ASSERT_GE(updated_value, last_value);
    last_value = updated_value;
  }
  ASSERT_NEAR(last_value, 1.f, 0.00001);
}

TEST_F(ParkinglotMapTest3x2, UpdateData_RepeatedUpdates_Shrink_SetsCorrectValue) {
  const int NUM_ITERATIONS = 10000;
  int x_test = 0;
  int y_test = 0;

  float last_value = testee.getData(x_test, y_test);

  // Let the values shrink
  for (unsigned int i = 0; i < NUM_ITERATIONS; ++i) {
    testee.updateData(x_test, y_test, 0.3);
    float updated_value = testee.getData(x_test, y_test);
    ASSERT_LE(updated_value, last_value);
    last_value = updated_value;
  }
  ASSERT_NEAR(last_value, 0.f, 0.00001);
}


TEST_F(ParkinglotMapTest3x2, UpdateData_WithXOutOfRange_ReturnsFalse) {
  int x_too_high = WIDTH_IN_PARKINGLOT + RESOLUTION * 2;
  int y = HEIGHT_IN_PARKINGLOT - 1;
  bool updated = testee.updateData(x_too_high, y, 0.42f);

  ASSERT_EQ(false, updated);
}

TEST_F(ParkinglotMapTest3x2, CreateMessage_OnCall_ReturnsSameDataAsParkinglotMap) {
  testee.updateData(0, 0, 0.1f);
  testee.updateData(2, 1, 0.1f);

  nav_msgs::OccupancyGrid msg = testee.createMessage("frame", ros::Time(42.0), 1.0);

  ASSERT_EQ("frame", msg.header.frame_id);
  ASSERT_DOUBLE_EQ(42.0, msg.header.stamp.toSec());
  ASSERT_DOUBLE_EQ(1.0, msg.info.origin.orientation.w);
  ASSERT_EQ(RESOLUTION, msg.info.resolution);
  ASSERT_EQ(WIDTH_IN_PARKINGLOT / RESOLUTION, msg.info.width);
  ASSERT_EQ(HEIGHT_IN_PARKINGLOT / RESOLUTION, msg.info.height);
  ASSERT_EQ(10, msg.data[0]);
  ASSERT_EQ(10, msg.data[5]);
}

TEST_F(ParkinglotMapTest3x2, UpdateDataOnLine_WithFlatLine_UpdatesOnlyThisRow) {
  testee.updateDataOnLine(0, 0, 2, 0, 0.1f, 0.2f);

  ASSERT_EQ(0.1f, testee.getData(0, 0));
  ASSERT_EQ(0.1f, testee.getData(1, 0));
  ASSERT_EQ(0.2f, testee.getData(2, 0));
  for (int y = 1; y < HEIGHT_IN_PARKINGLOT; ++y) {
    for (int x = 0; x < WIDTH_IN_PARKINGLOT; ++x) {
      ASSERT_EQ(DEFAULT_VALUE, testee.getData(x, y));
    }
  }
}

TEST_F(ParkinglotMapTest3x2, UpdateDataOnLine_WithinRange_ReturnsTrue) {
  bool result = testee.updateDataOnLine(0, 0, 1, 1, 0.1f, 0.1f);

  ASSERT_EQ(true, result);
}

TEST_F(ParkinglotMapTest3x2, UpdateDataOnLine_WithXOutOfRange_ReturnsFalse) {
  bool result = testee.updateDataOnLine(0, 0, 5, 1, 0.1f, 0.1f);

  ASSERT_EQ(false, result);
}

TEST_F(ParkinglotMapTest3x2, UpdateDataOnRay_WithPositiveXDirection_UpdatesAllUntilEndOfMap) {
  testee.updateDataOnRay(0, 0, 1, 0, 0.1f, 0.2f);
  ASSERT_EQ(0.1f, testee.getData(0, 0));
  ASSERT_EQ(0.2f, testee.getData(1, 0));
  ASSERT_EQ(0.2f, testee.getData(2, 0));
  for (int y = 1; y < HEIGHT_IN_PARKINGLOT; ++y) {
    for (int x = 0; x < WIDTH_IN_PARKINGLOT; ++x) {
      ASSERT_EQ(DEFAULT_VALUE, testee.getData(x, y));
    }
  }
}

TEST_F(ParkinglotMapTest3x2, UpdateDataOnRay_WithNegativeXDirection_UpdatesAllUntilEndOfMap) {
  testee.updateDataOnRay(2, 0, 1, 0, 0.1f, 0.2f);
  ASSERT_EQ(0.2f, testee.getData(0, 0));
  ASSERT_EQ(0.2f, testee.getData(1, 0));
  ASSERT_EQ(0.1f, testee.getData(2, 0));
  for (int y = 1; y < HEIGHT_IN_PARKINGLOT; ++y) {
    for (int x = 0; x < WIDTH_IN_PARKINGLOT; ++x) {
      ASSERT_EQ(DEFAULT_VALUE, testee.getData(x, y));
    }
  }
}

TEST_F(ParkinglotMapTest3x2,
       UpdateDataOnRay_WithPositiveXDirectionAndStartOutOfMap_UpdatesAllUntilEndOfMap) {
  testee.updateDataOnRay(-99, 0, 1, 0, 0.1f, 0.2f);
  ASSERT_EQ(0.1f, testee.getData(0, 0));
  ASSERT_EQ(0.2f, testee.getData(1, 0));
  ASSERT_EQ(0.2f, testee.getData(2, 0));
  for (int y = 1; y < HEIGHT_IN_PARKINGLOT; ++y) {
    for (int x = 0; x < WIDTH_IN_PARKINGLOT; ++x) {
      ASSERT_EQ(DEFAULT_VALUE, testee.getData(x, y));
    }
  }
}

TEST_F(ParkinglotMapTest3x2,
       UpdateDataOnRay_WithPositiveXDirectionAndEndOutOfMap_UpdatesAllUntilEndOfMap) {
  testee.updateDataOnRay(0, 0, 99, 0, 0.1f, 0.2f);
  ASSERT_EQ(0.1f, testee.getData(0, 0));
  ASSERT_EQ(0.1f, testee.getData(1, 0));
  ASSERT_EQ(0.1f, testee.getData(2, 0));
  for (int y = 1; y < HEIGHT_IN_PARKINGLOT; ++y) {
    for (int x = 0; x < WIDTH_IN_PARKINGLOT; ++x) {
      ASSERT_EQ(DEFAULT_VALUE, testee.getData(x, y));
    }
  }
}

TEST_F(ParkinglotMapTest3x3, UpdateDataOnRay_WithPositiveYDirection_UpdatesAllUntilEndOfMap) {
  testee.updateDataOnRay(0, 0, 0, 1, 0.1f, 0.2f);
  ASSERT_EQ(0.1f, testee.getData(0, 0));
  ASSERT_EQ(0.2f, testee.getData(0, 1));
  ASSERT_EQ(0.2f, testee.getData(0, 2));
  for (int y = 0; y < HEIGHT_IN_PARKINGLOT; ++y) {
    for (int x = 1; x < WIDTH_IN_PARKINGLOT; ++x) {
      ASSERT_EQ(DEFAULT_VALUE, testee.getData(x, y));
    }
  }
}

TEST_F(ParkinglotMapTest3x3,
       UpdateDataOnRay_WithPositiveYDirectionAndStartOutOfMap_UpdatesAllUntilEndOfMap) {
  testee.updateDataOnRay(0, -2, 0, 1, 0.1f, 0.2f);
  ASSERT_EQ(0.1f, testee.getData(0, 0));
  ASSERT_EQ(0.2f, testee.getData(0, 1));
  ASSERT_EQ(0.2f, testee.getData(0, 2));
  for (int y = 0; y < HEIGHT_IN_PARKINGLOT; ++y) {
    for (int x = 1; x < WIDTH_IN_PARKINGLOT; ++x) {
      ASSERT_EQ(DEFAULT_VALUE, testee.getData(x, y));
    }
  }
}

TEST_F(ParkinglotMapTest3x3, UpdateDataOnRay_WithNegativeYDirection_UpdatesAllUntilEndOfMap) {
  testee.updateDataOnRay(0, 2, 0, 1, 0.1f, 0.2f);
  ASSERT_EQ(0.2f, testee.getData(0, 0));
  ASSERT_EQ(0.2f, testee.getData(0, 1));
  ASSERT_EQ(0.1f, testee.getData(0, 2));
  for (int y = 0; y < HEIGHT_IN_PARKINGLOT; ++y) {
    for (int x = 1; x < WIDTH_IN_PARKINGLOT; ++x) {
      ASSERT_EQ(DEFAULT_VALUE, testee.getData(x, y));
    }
  }
}

TEST_F(ParkinglotMapTest3x3,
       UpdateDataOnRay_WithNegativeYDirectionAndStartAndEndOutOfMap_UpdatesAllUntilEndOfMap) {
  testee.updateDataOnRay(0, 99, 0, -99, 0.1f, 0.2f);
  ASSERT_EQ(0.1f, testee.getData(0, 0));
  ASSERT_EQ(0.1f, testee.getData(0, 1));
  ASSERT_EQ(0.1f, testee.getData(0, 2));
  for (int y = 0; y < HEIGHT_IN_PARKINGLOT; ++y) {
    for (int x = 1; x < WIDTH_IN_PARKINGLOT; ++x) {
      ASSERT_EQ(DEFAULT_VALUE, testee.getData(x, y));
    }
  }
}

}  // Namespace

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
