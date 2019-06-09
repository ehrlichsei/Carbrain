#include <common/macros.h>

THIRD_PARTY_HEADERS_BEGIN
#include <gtest/gtest.h>
#include <boost/static_assert.hpp>
#include <Eigen/Dense>
#include <limits>
THIRD_PARTY_HEADERS_END

#include "../src/road_watcher/object_tracking/hungarian_algorithm.h"

DISABLE_SUGGEST_OVERRIDE_WARNING

class HungarianAlgorithmTest : public ::testing::Test {

 protected:
  HungarianAlgorithm testee;

  HungarianAlgorithmTest() : ::testing::Test() {}
};

using Eigen::MatrixXd;
using Eigen::VectorXi;

TEST_F(HungarianAlgorithmTest, OneWorkerOneTask) {
  MatrixXd cost_mat(1, 1);
  cost_mat << std::numeric_limits<double>::max();

  VectorXi best_assigment = testee.findBestAssigment(cost_mat);

  EXPECT_EQ(best_assigment.size(), 1);
  EXPECT_EQ(best_assigment[0], 0);
}

TEST_F(HungarianAlgorithmTest, OneWorkerManyTasks) {
  MatrixXd cost_mat(1, 10);
  cost_mat << 1.0000000000001, 213.4, 355, 12, 112312, 1, 233, 42, 123, 3432423424;

  VectorXi best_assigment = testee.findBestAssigment(cost_mat);

  EXPECT_EQ(best_assigment.size(), 1);
  EXPECT_EQ(best_assigment[0], 5);
}

TEST_F(HungarianAlgorithmTest, ManyWorkersOneTask) {
  MatrixXd cost_mat(10, 1);
  // clang-format off
  cost_mat << 4,
							3,
							2,
							1,
							10123,
							132.012,
							1.000001,
							1.000002,
							99999999,
							7;
  // clang-format on

  VectorXi best_assigment = testee.findBestAssigment(cost_mat);

  EXPECT_EQ(best_assigment.size(), 10);

  for (int i = 0; i < 10; ++i) {
    if (i == 3) {
      EXPECT_EQ(best_assigment[i], 0);
      EXPECT_NE(best_assigment[i], HungarianAlgorithm::UNASSINGED);
    } else {
      EXPECT_EQ(best_assigment[i], HungarianAlgorithm::UNASSINGED);
    }
  }
}


TEST_F(HungarianAlgorithmTest, FourTasksFourWorkers) {
  MatrixXd cost_mat(4, 4);
  // clang-format off
  cost_mat << 1, 1, 1, 2,
							3, 2, 4, 1,
							4, 4, 2, 4,
							2, 3, 3, 3;
  // clang-format on

  VectorXi best_assigment = testee.findBestAssigment(cost_mat);

  EXPECT_EQ(best_assigment.size(), 4);
  EXPECT_EQ(best_assigment[0], 1);
  EXPECT_EQ(best_assigment[1], 3);
  EXPECT_EQ(best_assigment[2], 2);
  EXPECT_EQ(best_assigment[3], 0);
}

TEST_F(HungarianAlgorithmTest, InvalidCostMat) {
  MatrixXd cost_mat_with_negative_element(4, 4);
  // clang-format off
  cost_mat_with_negative_element << 1, 1, 1, 2,
             										    3, 2, 4, 1,
             										    4, 4, 2, 4,
             										    2, 3, 3, -3;
  // clang-format on

  ASSERT_THROW(testee.findBestAssigment(cost_mat_with_negative_element), std::invalid_argument);


  MatrixXd cost_mat_with_nan_element(2, 1);
  cost_mat_with_nan_element << std::numeric_limits<double>::quiet_NaN(), 1;

  ASSERT_THROW(testee.findBestAssigment(cost_mat_with_nan_element), std::invalid_argument);
}

TEST_F(HungarianAlgorithmTest, SixWorkersThreeTasks) {
  MatrixXd cost_mat(6, 3);
  // clang-format off
  cost_mat << 100, 5,    13,
							3,   9.8,  18,
							0,   7,    2,
							7,   11,   0.8,
							7,   9,    10,
							12,  13.1, 150;
  // clang-format on

  VectorXi best_assigment = testee.findBestAssigment(cost_mat);

  EXPECT_EQ(best_assigment.size(), 6);
  EXPECT_EQ(best_assigment[0], 1);
  EXPECT_EQ(best_assigment[1], HungarianAlgorithm::UNASSINGED);
  EXPECT_EQ(best_assigment[2], 0);
  EXPECT_EQ(best_assigment[3], 2);
  EXPECT_EQ(best_assigment[4], HungarianAlgorithm::UNASSINGED);
  EXPECT_EQ(best_assigment[5], HungarianAlgorithm::UNASSINGED);
}

TEST_F(HungarianAlgorithmTest, TenTasksTenWorkers) {
  MatrixXd cost_mat(10, 10);
  // clang-format off
  cost_mat << 74, 69, 50, 99, 25, 97, 33, 13, 29, 10,
							50, 48, 18, 12, 3,  34, 12, 59, 43, 50,
							13, 89, 41, 49, 36, 68, 76, 72, 67, 29,
							26, 45, 19, 35, 15, 74, 81, 83, 43, 30,
							13, 85, 68, 98, 38, 54, 50, 40, 11, 50,
							64, 91, 35, 71, 84, 89, 83, 31, 80, 7,
							48, 28, 55, 93, 76, 14, 72, 91, 37, 7,
							30, 32, 38, 66, 28, 57, 90, 36, 81, 20,
							3,  3,  91, 96, 68, 85, 12, 8,  42, 81,
							26, 2,  72, 84, 66, 77, 64, 53, 8,  18;
  // clang-format on

  VectorXi best_assigment = testee.findBestAssigment(cost_mat);

  EXPECT_EQ(best_assigment.size(), 10);
  EXPECT_EQ(best_assigment[0], 7);
  EXPECT_EQ(best_assigment[1], 3);
  EXPECT_EQ(best_assigment[2], 0);
  EXPECT_EQ(best_assigment[3], 2);
  EXPECT_EQ(best_assigment[4], 8);
  EXPECT_EQ(best_assigment[5], 9);
  EXPECT_EQ(best_assigment[6], 5);
  EXPECT_EQ(best_assigment[7], 4);
  EXPECT_EQ(best_assigment[8], 6);
  EXPECT_EQ(best_assigment[9], 1);
}

TEST_F(HungarianAlgorithmTest, ZeroTasksZeroWorkers) {
  MatrixXd cost_mat(0, 0);

  VectorXi best_assigment = testee.findBestAssigment(cost_mat);

  EXPECT_EQ(best_assigment.size(), 0);
}

TEST_F(HungarianAlgorithmTest, IsValid) {
  MatrixXd nans(3, 4);
  nans.fill(std::numeric_limits<double>::quiet_NaN());
  EXPECT_FALSE(hungarian_algorithm::isValid(nans));
  EXPECT_TRUE(hungarian_algorithm::isValid(MatrixXd::Zero(4, 5)));
  MatrixXd valid(2, 3);
  valid << 4, 1, 0.3, 5, 0, 0;
  EXPECT_TRUE(hungarian_algorithm::isValid(valid));
  MatrixXd invalid(1, 3);
  invalid << 3, -42, 5;
  EXPECT_FALSE(hungarian_algorithm::isValid(invalid));
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
