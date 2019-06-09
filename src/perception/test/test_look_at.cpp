#include <gtest/gtest.h>

class LookAtTest : public ::testing::Test {
  // helper methods etc go here
};

/**!
 * This is a placeholder for you REAL implementation unit tests.
 */
TEST_F(LookAtTest, areTestsImplemented) {
  const bool test_is_implemented = false;
  ASSERT_TRUE(test_is_implemented);
}



// nothing to do here
int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
