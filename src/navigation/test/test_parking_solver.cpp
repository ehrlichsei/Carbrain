#include <common/macros.h>

THIRD_PARTY_HEADERS_BEGIN
#include <gtest/gtest.h>
#include <cmath>
THIRD_PARTY_HEADERS_END

#include "../src/parking/parkingpathsolver.h"

DISABLE_SUGGEST_OVERRIDE_WARNING

TEST(TestParkingSolver, testPlausibleValues) {
    VehicleState start_state(-0.50f, 0.72f, 0);
    VehicleState end_state(-0.05f, 0.10f, - M_PI / 16); // 70cm parking spot
    ParkingPathSolver solver(start_state, end_state);
    solver.solve();
    ASSERT_LE(solver.getPreBeginAngle(), start_state.angle);
    ASSERT_LE(solver.getTurningPointAngle(), 0);
    ASSERT_GE(solver.getTurningPointAngle(), -M_PI_2);
}

// Run all the tests that were declared with TEST()
int main(int argc, char **argv){
    testing::InitGoogleTest(&argc, argv);
        return RUN_ALL_TESTS();
}
