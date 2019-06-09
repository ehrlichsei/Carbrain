#include "common/normal_shift.h"
#include "common/macros.h"

THIRD_PARTY_HEADERS_BEGIN
#include <gtest/gtest.h>
#include <boost/range/algorithm/equal.hpp>
THIRD_PARTY_HEADERS_END

DISABLE_SUGGEST_OVERRIDE_WARNING

TEST(NormalShift, polynomialNormalShiftNoop) {
  const common::DynamicPolynomial p{std::vector<double>{1, 5, 2}};
  std::vector<Eigen::Vector2d> ps;
  common::normalShift(p, 1.0, common::DiscretizationParams{}, &ps);
  EXPECT_TRUE(ps.empty());
}

TEST(NormalShift, polynomialNormalShiftZeroIsDiscretization) {
  const common::DynamicPolynomial p{std::vector<double>{1, 5, 2}};
  const common::DiscretizationParams params{0, 5.0, 0.5};
  std::vector<Eigen::Vector2d> nps;
  common::normalShift(p, 0.0, params, &nps);
  const auto ps = common::discretisize<Eigen::Vector2d>(p, params);
  EXPECT_TRUE(boost::equal(nps, ps));
}

TEST(NormalShift, polynomialNormalShiftPointWiseNormalShiftEquvalenz) {
  const common::DynamicPolynomial p{std::vector<double>{1, 5, 2}};
  const common::DiscretizationParams params{0, 1.0, 0.0001};
  std::vector<Eigen::Vector2d> nps, ops;
  common::normalShift(p, 1.0, params, &nps);
  const auto ps = common::discretisize<Eigen::Vector2d>(p, params);
  common::normalShift(ps, 1.0, &ops);
  EXPECT_EQ(nps.size(), ops.size() + 1);
  for (std::size_t i = 0; i < ops.size(); i++) {
    EXPECT_LE((nps[i] - ops[i]).norm(), 1e-3);
  }
}

TEST(NormalShift, polynomialNormalShiftPointWiseNormalShiftEquvalenz3D) {
  const common::DynamicPolynomial p{std::vector<double>{1, 5, 2}};
  const common::DiscretizationParams params{0, 1.0, 0.0001};
  std::vector<Eigen::Vector3d> nps, ops;
  common::normalShift(p, 1.0, params, &nps);
  const auto ps = common::discretisize<Eigen::Vector3d>(p, params);
  common::normalShift(ps, 1.0, &ops);
  EXPECT_EQ(nps.size(), ops.size() + 1);
  for (std::size_t i = 0; i < ops.size(); i++) {
    EXPECT_LE((nps[i] - ops[i]).norm(), 1e-3);
  }
}

TEST(NormalShift, polynomialNormalShiftPointWiseNormalShiftEquvalenz3DLifted) {
  const common::DynamicPolynomial polynomial{std::vector<double>{1, 5, 2}};
  const common::DiscretizationParams params{0, 1.0, 0.0001};
  std::vector<Eigen::Vector3d> nps, ops;
  common::normalShift(polynomial, 1.0, params, &nps);
  for (auto& np : nps) {
    np.z() += 0.5;
  }
  auto ps = common::discretisize<Eigen::Vector3d>(polynomial, params);
  for (auto& p : ps) {
    p.z() += 0.5;
  }
  common::normalShift(ps, 1.0, &ops);
  EXPECT_EQ(nps.size(), ops.size() + 1);
  for (std::size_t i = 0; i < ops.size(); i++) {
    EXPECT_LE((nps[i] - ops[i]).norm(), 1e-3);
  }
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
