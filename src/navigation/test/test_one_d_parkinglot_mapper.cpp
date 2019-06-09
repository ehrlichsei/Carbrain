#include <common/macros.h>

THIRD_PARTY_HEADERS_BEGIN
#include <gtest/gtest.h>
THIRD_PARTY_HEADERS_END

#include "../src/find_parking_slot/onedparkinglotmapper.h"

DISABLE_SUGGEST_OVERRIDE_WARNING

#define ASSERT_MAP_CONTENT_EQUALS(a, b, len)                \
  do {                                                      \
    for (size_t i = 0; i < len; i++) ASSERT_EQ(a[i], b[i]); \
  } while (false)

namespace {
// Test fixture
class OneDParkinglotMapperTest : public ::testing::Test {
 protected:
  const float MAX_RAMP_Y_DIFF = 1;
  const size_t MIN_OBSTACLE_LENGTH = 3;
  const int OCC = OneDParkinglotMapper::VALUE_OCCUPIED;
  const int FIL = OneDParkinglotMapper::VALUE_FILTERED_OUT;
  const int EMP = OneDParkinglotMapper::VALUE_EMPTY;

  OneDParkinglotMapperTest()
      : ::testing::Test(), testee(MIN_OBSTACLE_LENGTH, MAX_RAMP_Y_DIFF) {}

  OneDParkinglotMapper testee;
};

TEST_F(OneDParkinglotMapperTest, MapToOneD_MinObstacleSize_DoesNotFilterAnything) {
  nav_msgs::OccupancyGrid twoDMap;
  twoDMap.info.height = 5;
  twoDMap.info.width = 5;
  twoDMap.info.resolution = 1;
  twoDMap.data = {0,   0, 0, 0,   0,   0,   100, 100, 100, 0, 0, 100, 100,
                  100, 0, 0, 100, 100, 100, 0,   0,   0,   0, 0, 0};
  auto res = testee.mapToOneD(twoDMap);

  std::array<int, 5> expected{{EMP, OCC, OCC, OCC, EMP}};
  ASSERT_MAP_CONTENT_EQUALS(expected, res.data, res.info.height * res.info.width);
}

TEST_F(OneDParkinglotMapperTest, MapToOneD_BeginningRampUnderThreshold_FiltersOutRamp) {
  nav_msgs::OccupancyGrid twoDMap;
  twoDMap.info.height = 6;
  twoDMap.info.width = 5;
  twoDMap.info.resolution = 1;
  twoDMap.data = {0, 0,   0,   0,   0, 0, 0,   100, 100, 0,
                  0, 100, 100, 100, 0, 0, 100, 100, 100, 0,
                  0, 100, 100, 100, 0, 0, 0,   0,   0,   0};
  auto res = testee.mapToOneD(twoDMap);

  std::array<int, 6> expected{{EMP, OCC, OCC, OCC, OCC, EMP}};
  ASSERT_MAP_CONTENT_EQUALS(expected, res.data, res.info.height * res.info.width);
}

TEST_F(OneDParkinglotMapperTest, MapToOneD_BeginningRamp_FiltersOutRamp) {
  nav_msgs::OccupancyGrid twoDMap;
  twoDMap.info.height = 6;
  twoDMap.info.width = 5;
  twoDMap.info.resolution = 1;
  twoDMap.data = {0, 0,   0,   0,   0, 0, 0,   0,   100, 0,
                  0, 100, 100, 100, 0, 0, 100, 100, 100, 0,
                  0, 100, 100, 100, 0, 0, 0,   0,   0,   0};
  auto res = testee.mapToOneD(twoDMap);

  std::array<int, 6> expected{{EMP, FIL, OCC, OCC, OCC, EMP}};
  ASSERT_MAP_CONTENT_EQUALS(expected, res.data, res.info.height * res.info.width);
}


TEST_F(OneDParkinglotMapperTest, MapToOneD_EndingRamp_FiltersOutRamp) {
  nav_msgs::OccupancyGrid twoDMap;
  twoDMap.info.height = 6;
  twoDMap.info.width = 5;
  twoDMap.info.resolution = 1;
  twoDMap.data = {0, 0,   0,   0,   0, 0, 100, 100, 100, 0,
                  0, 100, 100, 100, 0, 0, 100, 100, 100, 0,
                  0, 0,   0,   100, 0, 0, 0,   0,   0,   0};
  auto res = testee.mapToOneD(twoDMap);

  std::array<int, 6> expected{{EMP, OCC, OCC, OCC, FIL, EMP}};
  ASSERT_MAP_CONTENT_EQUALS(expected, res.data, res.info.height * res.info.width);
}


TEST_F(OneDParkinglotMapperTest, MapToOneD_LongStartingAndEndingRamps_FiltersOutRamps) {
  nav_msgs::OccupancyGrid twoDMap;
  twoDMap.info.height = 7;
  twoDMap.info.width = 5;
  twoDMap.info.resolution = 1;
  twoDMap.data = {0,   0,   0,   0,   100, 0,   0,   100, 100, 100, 100, 100,
                  100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100,
                  100, 0,   0,   100, 100, 100, 0,   0,   0,   0,   100};
  auto res = testee.mapToOneD(twoDMap);

  std::array<int, 7> expected{{FIL, FIL, OCC, OCC, OCC, FIL, FIL}};
  ASSERT_MAP_CONTENT_EQUALS(expected, res.data, res.info.height * res.info.width);
}

TEST_F(OneDParkinglotMapperTest, MapToOneD_TooShortObstacle_FiltersOutObstacle) {
  nav_msgs::OccupancyGrid twoDMap;
  twoDMap.info.height = 3;
  twoDMap.info.width = 2;
  twoDMap.info.resolution = 1;
  twoDMap.data = {0, 0, 0, 100, 0, 0};
  auto res = testee.mapToOneD(twoDMap);

  std::array<int, 3> expected{{EMP, FIL, EMP}};
  ASSERT_MAP_CONTENT_EQUALS(expected, res.data, res.info.height * res.info.width);
}


TEST_F(OneDParkinglotMapperTest, MapToOneD_TooShortObstacleFillingMap_FiltersOutObstacle) {
  nav_msgs::OccupancyGrid twoDMap;
  twoDMap.info.height = 2;
  twoDMap.info.width = 2;
  twoDMap.info.resolution = 1;
  twoDMap.data = {
      0, 100, 0, 100,
  };
  auto res = testee.mapToOneD(twoDMap);

  std::array<int, 2> expected{{FIL, FIL}};
  ASSERT_MAP_CONTENT_EQUALS(expected, res.data, res.info.height * res.info.width);
}

TEST_F(OneDParkinglotMapperTest,
       MapToOneD_LongStartingAndEndingRampsAndTooShortFiltered_FiltersOutCompleteObstacle) {
  nav_msgs::OccupancyGrid twoDMap;
  twoDMap.info.height = 6;
  twoDMap.info.width = 5;
  twoDMap.info.resolution = 1;
  twoDMap.data = {0,   0,   0,   0,   100, 0,   0,   100, 100, 100,
                  100, 100, 100, 100, 100, 100, 100, 100, 100, 100,
                  0,   0,   100, 100, 100, 0,   0,   0,   0,   100};
  auto res = testee.mapToOneD(twoDMap);

  std::array<int, 7> expected{{FIL, FIL, FIL, FIL, FIL, FIL, FIL}};
  ASSERT_MAP_CONTENT_EQUALS(expected, res.data, res.info.height * res.info.width);
}
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
