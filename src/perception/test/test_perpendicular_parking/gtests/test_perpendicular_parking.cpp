#include <common/macros.h>
#include "parking_lot_generator.h"

THIRD_PARTY_HEADERS_BEGIN
#include <gtest/gtest.h>
THIRD_PARTY_HEADERS_END

DISABLE_SUGGEST_OVERRIDE_WARNING

namespace perpendicular_parking {
namespace testing {
class PerpendicularParkingTest : public ::testing::Test {
  // helper methods etc go here
 public:
  PerpendicularParkingTest() {}
};

/**!
 * This is a placeholder for you REAL implementation unit tests.
 */
TEST_F(PerpendicularParkingTest, areTestsImplemented) {
  const bool test_is_implemented = true;
  const std::vector<GeneratedOccupationState> dummy_occ_states_ = {
      {GeneratedOccupationState::FREE,
       GeneratedOccupationState::X,
       GeneratedOccupationState::OBSTACLE_SMALL,
       GeneratedOccupationState::OBSTACLE_MEDIUM,
       GeneratedOccupationState::OBSTACLE_LARGE}};
  ParkingLotGenerator dummy_generator_(dummy_occ_states_);
  ASSERT_TRUE(test_is_implemented);
}

}  // namespace testing
}  // namespace perpendicular_parking

// nothing to do here
int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
