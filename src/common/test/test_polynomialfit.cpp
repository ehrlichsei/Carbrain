#include "common/macros.h"
#include "common/polynomialfit.h"

THIRD_PARTY_HEADERS_BEGIN
#include <gtest/gtest.h>
#include <boost/math/distributions/chi_squared.hpp>
#include <limits>
#include <random>
THIRD_PARTY_HEADERS_END

#include "common/discretisize.h"

DISABLE_SUGGEST_OVERRIDE_WARNING

TEST(PolynomialFit, absoluteFittingErrorTrivial) {
  const common::DynamicPolynomial p{std::vector<double>{1, 5, 2, 9}};
  const auto ps = common::discretisize<Eigen::Vector3d>(p, {-1, 1, 0.1});
  const double e = common::computeAbsoluteFittingError(ps, p);
  EXPECT_EQ(0.0, e);
}

TEST(PolynomialFit, absoluteFittingErrorConstantOffset) {
  const common::DynamicPolynomial polynomial{std::vector<double>{3, 1, 6, 3}};
  auto ps = common::discretisize<Eigen::Vector3d>(polynomial, {-1, 1, 0.1});
  for (auto& p : ps) {
    p.y() += 0.1;
  }
  const double e = common::computeAbsoluteFittingError(ps, polynomial);
  EXPECT_NEAR(0.1, e, std::numeric_limits<double>::epsilon());
}

TEST(PolynomialFit, absoluteFittingErrorAlternatingOffset) {
  const common::DynamicPolynomial polynomial{std::vector<double>{3, 1, 6, 3}};
  auto ps = common::discretisize<Eigen::Vector3d>(polynomial, {-1, 1, 0.1});
  double sign = 1.0;
  for (auto& p : ps) {
    p.y() += sign * 0.1;
    sign = -sign;
  }
  const double e = common::computeAbsoluteFittingError(ps, polynomial);
  EXPECT_NEAR(0.1, e, std::numeric_limits<double>::epsilon());
}

TEST(PolynomialFit, polynomialFitDynamic) {
  const common::DynamicPolynomial p{std::vector<double>{3, 6, 1, 5}};
  const auto ps = common::discretisize<Eigen::Vector2d>(p, {-1, 1, 0.1});
  const auto fitted = common::fitToPoints(ps, 3);
  const double e = common::computeAbsoluteFittingError(ps, fitted);
  EXPECT_NEAR(0.0, e, std::numeric_limits<double>::epsilon() * 10);
}

TEST(PolynomialFit, polynomialFitStatic) {
  const common::CubicPolynomial p{std::vector<double>{3, 6, 1, 5}};
  const auto ps = common::discretisize<Eigen::Vector2d>(p, {-1, 1, 0.1});
  const auto fitted = common::fitToPoints<3>(ps);
  const double e = common::computeAbsoluteFittingError(ps, fitted);
  EXPECT_NEAR(0.0, e, std::numeric_limits<double>::epsilon() * 10);
}

TEST(PolynomialFit, polynomialFitDynamicWeightedExactStdVector) {
  const common::DynamicPolynomial p{std::vector<double>{3, 6, 1, 5}};
  const auto ps = common::discretisize<Eigen::Vector2d>(p, {-1, 1, 0.1});
  std::vector<double> weights(ps.size());
  std::mt19937 rand_gen;
  std::uniform_real_distribution<> variance_distr(0, 16);
  std::generate(
      weights.begin(), weights.end(), [&]() { return variance_distr(rand_gen); });
  const auto fitted = common::fitToPointsWeighted(ps, weights, 3);
  const double e = common::computeAbsoluteFittingError(ps, fitted);
  EXPECT_NEAR(0.0, e, std::numeric_limits<double>::epsilon() * 10);
}

TEST(PolynomialFit, polynomialFitStaticWeightedExactStdVector) {
  const common::CubicPolynomial p{std::vector<double>{3, 6, 1, 5}};
  const auto ps = common::discretisize<Eigen::Vector2d>(p, {-1, 1, 0.1});
  std::vector<double> weights(ps.size());
  std::mt19937 rand_gen;
  std::uniform_real_distribution<> variance_distr(0, 16);
  std::generate(
      weights.begin(), weights.end(), [&]() { return variance_distr(rand_gen); });
  const auto fitted = common::fitToPointsWeighted<3>(ps, weights);
  const double e = common::computeAbsoluteFittingError(ps, fitted);
  EXPECT_NEAR(0.0, e, std::numeric_limits<double>::epsilon() * 10);
}

TEST(PolynomialFit, polynomialFitDynamicWeightedExactEigenVector) {
  const common::DynamicPolynomial p{std::vector<double>{3, 6, 1, 5}};
  const auto ps = common::discretisize<Eigen::Vector2d>(p, {-1, 1, 0.1});
  Eigen::VectorXd weights = Eigen::VectorXd::LinSpaced(ps.size(), 0., 8.);
  const auto fitted = common::fitToPointsWeighted(ps, weights, 3);
  const double e = common::computeAbsoluteFittingError(ps, fitted);
  EXPECT_NEAR(0.0, e, std::numeric_limits<double>::epsilon() * 80);
}

TEST(PolynomialFit, polynomialFitStaticWeightedExactEigenVector) {
  const common::CubicPolynomial p{std::vector<double>{3, 6, 1, 5}};
  const auto ps = common::discretisize<Eigen::Vector2d>(p, {-1, 1, 0.1});
  Eigen::VectorXd weights = Eigen::VectorXd::LinSpaced(ps.size(), 0., 8.);
  const auto fitted = common::fitToPointsWeighted<3>(ps, weights);
  const double e = common::computeAbsoluteFittingError(ps, fitted);
  EXPECT_NEAR(0.0, e, std::numeric_limits<double>::epsilon() * 80);
}

TEST(PolynomialFit, polynomialFitDynamicWeightedExactStdArray) {
  const common::DynamicPolynomial p{std::vector<double>{3, 6, 1, 5}};
  auto ps = common::discretisize<Eigen::Vector2d>(p, {-1, 1, 0.1});
  ps.resize(18);
  std::array<double, 18> weights;
  std::mt19937 rand_gen;
  std::uniform_real_distribution<> variance_distr(0, 16);
  std::generate(
      weights.begin(), weights.end(), [&]() { return variance_distr(rand_gen); });
  const auto fitted = common::fitToPointsWeighted(ps, weights, 3);
  const double e = common::computeAbsoluteFittingError(ps, fitted);
  EXPECT_NEAR(0.0, e, std::numeric_limits<double>::epsilon() * 10);
}

TEST(PolynomialFit, polynomialFitStaticWeightedExactStdArray) {
  const common::CubicPolynomial p{std::vector<double>{3, 6, 1, 5}};
  auto ps = common::discretisize<Eigen::Vector2d>(p, {-1, 1, 0.1});
  ps.resize(18);
  std::array<double, 18> weights;
  std::mt19937 rand_gen;
  std::uniform_real_distribution<> variance_distr(0, 16);
  std::generate(
      weights.begin(), weights.end(), [&]() { return variance_distr(rand_gen); });
  const auto fitted = common::fitToPointsWeighted<3>(ps, weights);
  const double e = common::computeAbsoluteFittingError(ps, fitted);
  EXPECT_NEAR(0.0, e, std::numeric_limits<double>::epsilon() * 10);
}

TEST(PolynomialFit, polynomialFitDynamicCovariance) {
  const common::DynamicPolynomial p{std::vector<double>{3, 6, 1, 5}};
  auto ps = common::discretisize<Eigen::Vector2d>(p, {-1, 1, 0.1});
  Eigen::MatrixXd covariance = Eigen::MatrixXd::Identity(ps.size(), ps.size());
  covariance.diagonal() = Eigen::VectorXd::LinSpaced(ps.size(), 0.02, 20'000.);
  const auto fitted = common::fitToPointsCovariance(ps, covariance, 3);
  const double e = common::computeAbsoluteFittingError(ps, fitted);
  EXPECT_NEAR(0.0, e, 10e-10);
}

TEST(PolynomialFit, polynomialFitStaticCovariance) {
  const common::CubicPolynomial p{std::vector<double>{3, 6, 1, 5}};
  auto ps = common::discretisize<Eigen::Vector2d>(p, {-1, 1, 0.1});
  Eigen::MatrixXd covariance = Eigen::MatrixXd::Identity(ps.size(), ps.size());
  covariance.diagonal() = Eigen::VectorXd::LinSpaced(ps.size(), 0.02, 20'000.);
  const auto fitted = common::fitToPointsCovariance<3>(ps, covariance);
  const double e = common::computeAbsoluteFittingError(ps, fitted);
  EXPECT_NEAR(0.0, e, 10e-10);
}

TEST(PolynomialFit, polynomialFitDynamicWeightedNoise) {
  const common::DynamicPolynomial p{std::vector<double>{3, 6, 1, 5}};
  std::mt19937 rand_gen;

  for (int i = 0; i < 40; i++) {
    auto ps = common::discretisize<Eigen::Vector2d>(p, {-1, 1, 0.1});
    std::vector<double> weights(ps.size());
    std::uniform_real_distribution<> variance_distr(0.1, 16.);
    std::generate(weights.begin(),
                  weights.end(),
                  [&]() { return variance_distr(rand_gen); });

    std::normal_distribution<> error_distr(0.);
    std::transform(ps.begin(),
                   ps.end(),
                   weights.begin(),
                   ps.begin(),
                   [&](const auto& mean, const auto& weight) {
                     return Eigen::Vector2d(
                         mean.x(),
                         error_distr(rand_gen,
                                     std::normal_distribution<>::param_type(
                                         mean.y(), 1. / std::sqrt(weight))));
                   });

    const auto fitted = common::fitToPointsWeighted(ps, weights, 3);
    Eigen::MatrixXd error_variance_inverse =
        Eigen::MatrixXd::Zero(weights.size(), weights.size());
    error_variance_inverse.diagonal()
        << Eigen::Map<Eigen::VectorXd>(weights.data(), weights.size());

    const auto estimation_covariance = common::calculateEstimationCovariance(
        ps.begin(), ps.end(), 3, error_variance_inverse.inverse());

    const Eigen::Vector4d polynomial_coeffs_actual(p.getCoefficients().data());
    const Eigen::Vector4d polynomial_coeffs_estimated(fitted.getCoefficients().data());
    const double mahalanobis_distance =
        std::sqrt((polynomial_coeffs_actual - polynomial_coeffs_estimated).transpose() *
                  estimation_covariance.inverse() *
                  (polynomial_coeffs_actual - polynomial_coeffs_estimated));

    // 99% confidence interval, formulas from
    // https://core.ac.uk/download/pdf/16412844.pdf ("On the Mahalanobis
    // Distance Classification Criterion for Multidimensional Normal
    // Distributions" by Gallego et al.)
    const boost::math::chi_squared_distribution<> chi_squared(4);
    const double mahalanobis_quantile =
        std::sqrt(boost::math::quantile(chi_squared, 0.99));
    EXPECT_LE(mahalanobis_distance, mahalanobis_quantile);
  }
}


TEST(PolynomialFit, numberOfDistinctXValuesEmpty) {
  EXPECT_EQ(0, common::numberOfDistinctXValues(std::vector<Eigen::Vector2d>{}));
}

TEST(PolynomialFit, numberOfDistinctXValuesAllDistinct) {
  const std::vector<Eigen::Vector2d> vs = {{0, 0}, {1, 1}, {3, 3}, {4, 4}};
  EXPECT_EQ(vs.size(), common::numberOfDistinctXValues(vs, 0.99));
}

TEST(PolynomialFit, numberOfDistinctXValuesIdenticalValuesIsOneDistincValue) {
  const std::vector<Eigen::Vector2d> vs = {{1, 1}, {1, 1}, {1, 1}};
  EXPECT_EQ(1, common::numberOfDistinctXValues(vs));
}

TEST(PolynomialFit, numberOfDistinctXValuesViewDistince) {
  const std::vector<Eigen::Vector2d> vs = {{1, 1}, {1.5, 1}, {2, 1}, {2.5, 1}};
  EXPECT_EQ(2, common::numberOfDistinctXValues(vs, 0.6));
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
