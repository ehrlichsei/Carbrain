#ifndef POLYFIT_H
#define POLYFIT_H

#include <vector>

#include <boost/range/algorithm/sort.hpp>
#include <boost/range/algorithm_ext/push_back.hpp>
#include "common/adaptors.h"
#include "common/eigen_functors.h"
#include "common/polynomial.h"
#include "common/polynomial_degree.h"
#include "common/unique_erase.h"
#include "types.h"

namespace common {
namespace polynomial {

/*!
 * \brief numberOfDistinctXValues returns the number of distinct x-values in
 *range.
 * Two x-values are distinct, if their absolute difference is smaller than eps.
 *
 * \param range the range.
 * \param eps the minimal distance between two x-values to be distinct.
 * \return the number of distinct x-values.
 */
template <class Range>
std::size_t numberOfDistinctXValues(const Range &range,
                                    double eps = std::numeric_limits<double>::epsilon()) {
  std::vector<double> xs;
  boost::push_back(xs, range | common::x_values);
  boost::sort(xs);
  common::unique_erase(
      xs, [eps](const auto &a, const auto &b) { return std::abs(a - b) < eps; });
  return xs.size();
}

/*!
 * \brief numberOfDistinctXValues returns the number of distinct x-values in
 *range.
 * Two x-values are distinct, if their absolute difference is smaller than eps.
 *
 * \param begin the begin of the range.
 * \param end the end of the range.
 * \param eps the minimal distance between two x-values to be distinct.
 * \return the number of distinct x-values.
 */
template <class IT>
std::size_t numberOfDistinctXValues(IT begin,
                                    IT end,
                                    double eps = std::numeric_limits<double>::epsilon()) {
  return numberOfDistinctXValues(boost::make_iterator_range(begin, end), eps);
}

namespace detail {

template <class IT>
std::pair<Eigen::VectorXd, Eigen::MatrixXd> calculateRegressionSystem(
    IT begin, IT end, const PolynomialDegree degree) {
  const int num_points = std::distance(begin, end);
  const int num_params = degree + 1;
  Eigen::MatrixXd vandermonde(num_points, num_params);
  Eigen::VectorXd y(num_points);

  // fill the vandermonde matrix
  for (int i = 0; begin != end; ++i, ++begin) {
    vandermonde(i, 0) = 1.0;
    // y-coordinate is the x-argument
    const double x = (*begin)(0);
    for (int j = 1; j < num_params; j++) {
      vandermonde(i, j) = vandermonde(i, j - 1) * x;
    }
    // x-coordinate is the y-argument
    y(i) = (*begin)(1);
  }
  return {y, vandermonde};
}

template <PolynomialDegree DEGREE = PolynomialDegrees::Dynamic, class IT, class WeightMatrix>
auto fitToPointsWeightMatrix(IT begin,
                             IT end,
                             const WeightMatrix &weight_matrix,
                             const PolynomialDegree degree = DEGREE) {
  static_assert(DEGREE != PolynomialDegrees::MinusInfinity, "");
  assert(!(DEGREE == PolynomialDegrees::Dynamic) || degree != DEGREE);
  assert(!(DEGREE != PolynomialDegrees::Dynamic) || degree == DEGREE);
  const int num_points = std::distance(begin, end);
  // make sure that the vandermonde-matrix is invertable.
  assert(num_points > degree);
  assert(degree >= 0);
  assert(numberOfDistinctXValues(begin, end, std::numeric_limits<double>::epsilon()) >
         static_cast<std::size_t>(degree));
  assert(weight_matrix.rows() == num_points && weight_matrix.cols() == num_points);

  const int num_params = degree + 1;
  Eigen::MatrixXd vandermonde(num_points, num_params);
  Eigen::VectorXd y(num_points);
  std::tie(y, vandermonde) = calculateRegressionSystem(begin, end, degree);

  // solve the least-square problem
  Eigen::VectorXd coeffs(num_params);
  coeffs = (vandermonde.transpose() * weight_matrix * vandermonde)
               .ldlt()
               .solve(vandermonde.transpose() * weight_matrix * y);

  // store the fitted coefficients of polynomial
  return Polynomial<DEGREE, double>(coeffs.data(), coeffs.data() + coeffs.size());
}
}

/*!
 * \brief fitToPointsCovariance uses Eigens Cholesky decomposition for
 * Generalized Least-Squares polynomial fit of 2D data
 *
 * This function implements the Generalized Least-Squares for the general case
 * of correlated errors/noise in the y-values. For uncorrelated, but not
 * identically distributed (different variances) errors use
 * fitToPointsWeighted() for a weighted least squares fit. To get a standard
 * Least-Squares-Fit (in case of independent identically distributed (iid)
 * errors use fitToPoints().
 *
 * See Eigen Library for more on the underlying functions:
 *   http://eigen.tuxfamily.org/dox/group__TutorialLinearAlgebra.html#title4
 * for Cholesky decomposition see
 *  * http://eigen.tuxfamily.org/dox/group__Cholesky__Module.html
 *  * http://eigen.tuxfamily.org/dox-devel/group__LeastSquares.html
 *
 * This code has been derived from the example code at
 *   https://forum.kde.org/viewtopic.php?f=74&t=118177
 *
 * \tparam DEGREE the degree of the target polynomial (defaults to Dynamic).
 * \tparam CovarianceMatrix the type of the covariance argument, most symmetric
 * Eigen Matrix types should work
 * \param begin the begin of the range.
 * \param end the end of the range.
 * \param covariance covariance matrix of the y-errors: diagonal matrix for
 * uncorrelated errors (use Eigen::DiagonalMatrix for fast inversion), identity
 * matrix for independent and identically distributed (iid) errors (standard
 * least squares), general symmetric positive definite matrix for any other
 * error distribution
 * \param degree the degree of the polynom (defaults to DEGREE)
 * \return the polynom fitted to the given points.
 * \pre std::distance(begin, end) > degree.
 * \pre if DEGREE == PolynomialDegree::Dynamic then degree must be specified.
 * \pre if DEGREE != PolynomialDegree::Dynamic then degree == DEGREE (degree
 *  should not be specified).
 * \pre the number of distinct x-values must be greater than degree.
 * \pre degree >= 0.
 * \pre covariance.rows() == covariance.cols() == std::distance(begin, end)
 * \pre covariance is symmetric
 * \pre covariance is positive definite
 */
template <PolynomialDegree DEGREE = PolynomialDegrees::Dynamic, class IT, class CovarianceMatrix>
auto fitToPointsCovariance(IT begin,
                           IT end,
                           const CovarianceMatrix &covariance,
                           const PolynomialDegree degree = DEGREE) {
  const CovarianceMatrix weight_matrix = covariance.inverse();
  return detail::fitToPointsWeightMatrix<DEGREE>(begin, end, weight_matrix, degree);
}

/*!
 * \brief fitToPointsCovariance uses Eigens Cholesky decomposition for
 * Generalized Least-Squares polynomial fit of 2D data
 *
 * This function implements the Generalized Least-Squares for the general case
 * of correlated errors/noise in the y-values. For uncorrelated, but not
 * identically distributed (different variances) errors use
 * fitToPointsWeighted() for a weighted least squares fit. To get a standard
 * Least-Squares-Fit (in case of independent identically distributed (iid)
 * errors use fitToPoints().
 *
 * See Eigen Library for more on the underlying functions:
 *   http://eigen.tuxfamily.org/dox/group__TutorialLinearAlgebra.html#title4
 * for Cholesky decomposition see
 *  * http://eigen.tuxfamily.org/dox/group__Cholesky__Module.html
 *  * http://eigen.tuxfamily.org/dox-devel/group__LeastSquares.html
 *
 * This code has been derived from the example code at
 *   https://forum.kde.org/viewtopic.php?f=74&t=118177
 *
 * \tparam DEGREE the degree of the target polynomial (defaults to Dynamic).
 * \tparam CovarianceMatrix the type of the covariance argument, most symmetric
 * Eigen Matrix types should work
 * \param points the source points.
 * \param covariance covariance matrix of the y-errors: diagonal matrix for
 * uncorrelated errors (use Eigen::DiagonalMatrix for fast inversion), identity
 * matrix for independent and identically distributed (iid) errors (standard
 * least squares), general symmetric positive definite matrix for any other
 * error distribution
 * \param degree the degree of the polynom (defaults to DEGREE)
 * \return the polynom fitted to the given points.
 * \pre std::distance(begin, end) > degree.
 * \pre if DEGREE == PolynomialDegree::Dynamic then degree must be specified.
 * \pre if DEGREE != PolynomialDegree::Dynamic then degree == DEGREE (degree
 *  should not be specified).
 * \pre the number of distinct x-values must be greater than degree.
 * \pre degree >= 0.
 * \pre covariance.rows() == covariance.cols() == std::distance(begin, end)
 * \pre covariance is symmetric
 * \pre covariance is positive definite
 */
template <PolynomialDegree DEGREE = PolynomialDegrees::Dynamic, class Range, class CovarianceMatrix>
auto fitToPointsCovariance(const Range &points,
                           const CovarianceMatrix &covariance,
                           const PolynomialDegree degree = DEGREE) {
  return fitToPointsCovariance<DEGREE>(
      std::begin(points), std::end(points), covariance, degree);
}

/*!
 * \brief fitToPointsWeighted uses Eigens Cholesky decomposition for a weighted
 * Least-Squares polynomial fit of 2D data
 *
 * This function implements a weighted Least-Squares for uncorrelated but not
 * identically distributed (different variances) errors/noise in the y-values.
 * For correlated errors use fitToPointsCovariance() for a generalized least
 * squares fit. To get a standard Least-Squares-Fit (in case of independent
 * identically distributed (iid) errors use fitToPoints().
 *
 * See Eigen Library for more on the underlying functions:
 *   http://eigen.tuxfamily.org/dox/group__TutorialLinearAlgebra.html#title4
 * for Cholesky decomposition see
 *  * http://eigen.tuxfamily.org/dox/group__Cholesky__Module.html
 *  * http://eigen.tuxfamily.org/dox-devel/group__LeastSquares.html
 *
 * This code has been derived from the example code at
 *   https://forum.kde.org/viewtopic.php?f=74&t=118177
 *
 * \tparam DEGREE the degree of the target polynomial (defaults to Dynamic).
 * \param begin the begin of the range.
 * \param end the end of the range.
 * \param weights weights for the datapoints (inverse of the error variance of
 * the respective datapoint)
 * \param degree the degree of the polynom (defaults to DEGREE)
 * \return the polynom fitted to the given points.
 * \pre std::distance(begin, end) > degree.
 * \pre if DEGREE == PolynomialDegree::Dynamic then degree must be specified.
 * \pre if DEGREE != PolynomialDegree::Dynamic then degree == DEGREE (degree
 *  should not be specified).
 * \pre the number of distinct x-values must be greater than degree.
 * \pre degree >= 0.
 * \pre NUMBER_OF_WEIGHTS == Eigen::Dynamic || NUMBER_OF_WEIGHTS ==
 * std::distance(begin, end)
 * \pre weights.rows() == std::distance(begin, end)
 * \pre all elements in weights have to be positive
 */
template <PolynomialDegree DEGREE = PolynomialDegrees::Dynamic, class IT, typename T, int NUMBER_OF_WEIGHTS = Eigen::Dynamic>
auto fitToPointsWeighted(IT begin,
                         IT end,
                         const Eigen::Matrix<T, NUMBER_OF_WEIGHTS, 1> &weights,
                         const PolynomialDegree degree = DEGREE) {
  assert(NUMBER_OF_WEIGHTS == Eigen::Dynamic || std::distance(begin, end) == NUMBER_OF_WEIGHTS);
  assert(std::distance(begin, end) == weights.rows());

  return detail::fitToPointsWeightMatrix<DEGREE>(
      begin, end, Eigen::DiagonalMatrix<T, NUMBER_OF_WEIGHTS>(weights), degree);
}

/*!
 * \brief fitToPointsWeighted uses Eigens Cholesky decomposition for a weighted
 * Least-Squares polynomial fit of 2D data
 *
 * This function implements a weighted Least-Squares for uncorrelated but not
 * identically distributed (different variances) errors/noise in the y-values.
 * For correlated errors use fitToPointsCovariance() for a generalized least
 * squares fit. To get a standard Least-Squares-Fit (in case of independent
 * identically distributed (iid) errors use fitToPoints().
 *
 * See Eigen Library for more on the underlying functions:
 *   http://eigen.tuxfamily.org/dox/group__TutorialLinearAlgebra.html#title4
 * for Cholesky decomposition see
 *  * http://eigen.tuxfamily.org/dox/group__Cholesky__Module.html
 *  * http://eigen.tuxfamily.org/dox-devel/group__LeastSquares.html
 *
 * This code has been derived from the example code at
 *   https://forum.kde.org/viewtopic.php?f=74&t=118177
 *
 * \tparam DEGREE the degree of the target polynomial (defaults to Dynamic).
 * \param points the source points.
 * \param weights weights for the datapoints (inverse of the error variance of
 * the respective datapoint)
 * \param degree the degree of the polynom (defaults to DEGREE)
 * \return the polynom fitted to the given points.
 * \pre std::distance(begin, end) > degree.
 * \pre if DEGREE == PolynomialDegree::Dynamic then degree must be specified.
 * \pre if DEGREE != PolynomialDegree::Dynamic then degree == DEGREE (degree
 *  should not be specified).
 * \pre the number of distinct x-values must be greater than degree.
 * \pre degree >= 0.
 * \pre NUMBER_OF_WEIGHTS == Eigen::Dynamic || NUMBER_OF_WEIGHTS ==
 * std::distance(begin, end)
 * \pre weights.rows() == std::distance(begin, end)
 * \pre all elements in weights have to be positive
 */
template <PolynomialDegree DEGREE = PolynomialDegrees::Dynamic, class Range, typename T, int NUMBER_OF_WEIGHTS = Eigen::Dynamic>
auto fitToPointsWeighted(const Range &points,
                         const Eigen::Matrix<T, NUMBER_OF_WEIGHTS, 1> &weights,
                         const PolynomialDegree degree = DEGREE) {
  return fitToPointsWeighted<DEGREE>(std::begin(points), std::end(points), weights, degree);
}


/*!
 * \brief fitToPoints uses Eigens Cholesky decomposition for Least-Squares
 * polynomial fit of 2D data
 *
 * See Eigen Library for more on the underlying functions:
 *   http://eigen.tuxfamily.org/dox/group__TutorialLinearAlgebra.html#title4
 * for Cholesky decomposition see
 *  * http://eigen.tuxfamily.org/dox/group__Cholesky__Module.html
 *  * http://eigen.tuxfamily.org/dox-devel/group__LeastSquares.html
 *
 * This code has been derived from the example code at
 *   https://forum.kde.org/viewtopic.php?f=74&t=118177
 *
 * \tparam DEGREE the degree of the target polynomial (defaults to Dynamic).
 * \param degree the degree of the polynom (defaults to DEGREE)
 * \param begin the begin of the range.
 * \param end the end of the range.
 * \param weights weights for the datapoints (inverse of the error variance of
 * the respective datapoint)
 * \return the polynom fitted to the given points.
 * \pre std::distance(begin, end) > degree.
 * \pre if DEGREE == PolynomialDegree::Dynamic then degree must be specified.
 * \pre if DEGREE != PolynomialDegree::Dynamic then degree == DEGREE (degree
 *  should not be specified).
 * \pre the number of distinct x-values must be greater than degree.
 * \pre degree >= 0.
 * \pre weights.size() == std::distance(begin, end)
 * \pre all elements in weights have to be positive
 */
template <PolynomialDegree DEGREE = PolynomialDegrees::Dynamic, class IT, typename WT>
auto fitToPointsWeighted(IT begin,
                         IT end,
                         const std::vector<WT> &weights,
                         const PolynomialDegree degree = DEGREE) {
  assert(weights.size() == static_cast<size_t>(std::distance(begin, end)));
  return detail::fitToPointsWeightMatrix<DEGREE>(
      begin,
      end,
      Eigen::DiagonalMatrix<WT, Eigen::Dynamic>(
          Eigen::Map<const Eigen::Matrix<WT, Eigen::Dynamic, 1>>(
              weights.data(), weights.size())),
      degree);
}

/*!
 * \brief fitToPoints uses Eigens Cholesky decomposition for Least-Squares
 * polynomial fit of 2D data
 *
 * See Eigen Library for more on the underlying functions:
 *   http://eigen.tuxfamily.org/dox/group__TutorialLinearAlgebra.html#title4
 * for Cholesky decomposition see
 *  * http://eigen.tuxfamily.org/dox/group__Cholesky__Module.html
 *  * http://eigen.tuxfamily.org/dox-devel/group__LeastSquares.html
 *
 * This code has been derived from the example code at
 *   https://forum.kde.org/viewtopic.php?f=74&t=118177
 *
 * \tparam DEGREE the degree of the target polynomial (defaults to Dynamic).
 * \param degree the degree of the polynom (defaults to DEGREE)
 * \param points the source points.
 * \param weights weights for the datapoints (inverse of the error variance of
 * the respective datapoint)
 * \return the polynom fitted to the given points.
 * \pre std::distance(begin, end) > degree.
 * \pre if DEGREE == PolynomialDegree::Dynamic then degree must be specified.
 * \pre if DEGREE != PolynomialDegree::Dynamic then degree == DEGREE (degree
 *  should not be specified).
 * \pre the number of distinct x-values must be greater than degree.
 * \pre degree >= 0.
 * \pre weights.size() == std::distance(begin, end)
 * \pre all elements in weights have to be positive
 */
template <PolynomialDegree DEGREE = PolynomialDegrees::Dynamic, class Range, typename WT>
auto fitToPointsWeighted(const Range &points,
                         const std::vector<WT> &weights,
                         const PolynomialDegree degree = DEGREE) {
  return fitToPointsWeighted<DEGREE>(std::begin(points), std::end(points), weights, degree);
}

/*!
 * \brief fitToPoints uses Eigens Cholesky decomposition for Least-Squares
 * polynomial fit of 2D data
 *
 * See Eigen Library for more on the underlying functions:
 *   http://eigen.tuxfamily.org/dox/group__TutorialLinearAlgebra.html#title4
 * for Cholesky decomposition see
 *  * http://eigen.tuxfamily.org/dox/group__Cholesky__Module.html
 *  * http://eigen.tuxfamily.org/dox-devel/group__LeastSquares.html
 *
 * This code has been derived from the example code at
 *   https://forum.kde.org/viewtopic.php?f=74&t=118177
 *
 * \tparam DEGREE the degree of the target polynomial (defaults to Dynamic).
 * \param degree the degree of the polynom (defaults to DEGREE)
 * \param begin the begin of the range.
 * \param end the end of the range.
 * \param weights weights for the datapoints (inverse of the error variance of
 * the respective datapoint)
 * \return the polynom fitted to the given points.
 * \pre std::distance(begin, end) > degree.
 * \pre if DEGREE == PolynomialDegree::Dynamic then degree must be specified.
 * \pre if DEGREE != PolynomialDegree::Dynamic then degree == DEGREE (degree
 *  should not be specified).
 * \pre the number of distinct x-values must be greater than degree.
 * \pre degree >= 0.
 * \pre std::distance(begin, end) ==  NUMBER_OF_WEIGHTS)
 * \pre all elements in weights have to be positive
 */
template <PolynomialDegree DEGREE = PolynomialDegrees::Dynamic, class IT, typename WT, size_t NUMBER_OF_WEIGHTS>
auto fitToPointsWeighted(IT begin,
                         IT end,
                         const std::array<WT, NUMBER_OF_WEIGHTS> &weights,
                         const PolynomialDegree degree = DEGREE) {
  static_assert(static_cast<long long>(NUMBER_OF_WEIGHTS) > DEGREE,
                "weights has to have at least (end-begin)+1 elements, so at "
                "least DEGREE + 1");
  assert(std::distance(begin, end) == NUMBER_OF_WEIGHTS);
  return detail::fitToPointsWeightMatrix<DEGREE>(
      begin,
      end,
      Eigen::DiagonalMatrix<WT, NUMBER_OF_WEIGHTS>(
          Eigen::Map<const Eigen::Matrix<WT, NUMBER_OF_WEIGHTS, 1>>(weights.data())),
      degree);
}

/*!
 * \brief fitToPoints uses Eigens Cholesky decomposition for Least-Squares
 * polynomial fit of 2D data
 *
 * See Eigen Library for more on the underlying functions:
 *   http://eigen.tuxfamily.org/dox/group__TutorialLinearAlgebra.html#title4
 * for Cholesky decomposition see
 *  * http://eigen.tuxfamily.org/dox/group__Cholesky__Module.html
 *  * http://eigen.tuxfamily.org/dox-devel/group__LeastSquares.html
 *
 * This code has been derived from the example code at
 *   https://forum.kde.org/viewtopic.php?f=74&t=118177
 *
 * \tparam DEGREE the degree of the target polynomial (defaults to Dynamic).
 * \param degree the degree of the polynom (defaults to DEGREE)
 * \param points the source points.
 * \param weights weights for the datapoints (inverse of the error variance of
 * the respective datapoint)
 * \return the polynom fitted to the given points.
 * \pre std::distance(begin, end) > degree.
 * \pre if DEGREE == PolynomialDegree::Dynamic then degree must be specified.
 * \pre if DEGREE != PolynomialDegree::Dynamic then degree == DEGREE (degree
 *  should not be specified).
 * \pre the number of distinct x-values must be greater than degree.
 * \pre degree >= 0.
 * \pre std::distance(begin, end) ==  NUMBER_OF_WEIGHTS)
 * \pre all elements in weights have to be positive
 */
template <PolynomialDegree DEGREE = PolynomialDegrees::Dynamic, class Range, typename WT, size_t NUMBER_OF_WEIGHTS>
auto fitToPointsWeighted(const Range &points,
                         const std::array<WT, NUMBER_OF_WEIGHTS> &weights,
                         const PolynomialDegree degree = DEGREE) {
  return fitToPointsWeighted<DEGREE>(std::begin(points), std::end(points), weights, degree);
}

/*!
 * \brief fitToPoints uses Eigens Cholesky decomposition for Least-Squares
 * polynomial fit of 2D data
 *
 * See Eigen Library for more on the underlying functions:
 *   http://eigen.tuxfamily.org/dox/group__TutorialLinearAlgebra.html#title4
 * for Cholesky decomposition see
 *  * http://eigen.tuxfamily.org/dox/group__Cholesky__Module.html
 *  * http://eigen.tuxfamily.org/dox-devel/group__LeastSquares.html
 *
 * This code has been derived from the example code at
 *   https://forum.kde.org/viewtopic.php?f=74&t=118177
 *
 * \tparam DEGREE the degree of the target polynomial (defaults to Dynamic).
 * \param degree the degree of the polynom (defaults to DEGREE)
 * \param begin the begin of the range.
 * \param end the end of the range.
 * \return the polynom fitted to the given points.
 * \pre std::distance(begin, end) > degree.
 * \pre if DEGREE == PolynomialDegree::Dynamic then degree must be specified.
 * \pre if DEGREE != PolynomialDegree::Dynamic then degree == DEGREE (degree
 *  should not be specified).
 * \pre the number of distinct x-values must be greater than degree.
 * \pre degree >= 0.
 */
template <PolynomialDegree DEGREE = PolynomialDegrees::Dynamic, class IT>
auto fitToPoints(IT begin, IT end, const PolynomialDegree degree = DEGREE) {
  Eigen::DiagonalMatrix<double, Eigen::Dynamic> weights(std::distance(begin, end));
  weights.setIdentity();
  return detail::fitToPointsWeightMatrix<DEGREE>(begin, end, weights, degree);
}


/*!
 * \brief fitToPoints uses Eigens Cholesky decomposition for Least-Squares
 * polynomial fit of 2D data
 *
 * See Eigen Library for more on the underlying functions:
 *   http://eigen.tuxfamily.org/dox/group__TutorialLinearAlgebra.html#title4
 * for Cholesky decomposition see
 *  * http://eigen.tuxfamily.org/dox/group__Cholesky__Module.html
 *  * http://eigen.tuxfamily.org/dox-devel/group__LeastSquares.html
 *
 * This code has been derived from the example code at
 *   https://forum.kde.org/viewtopic.php?f=74&t=118177
 *
 * \param degree the degree of the dynamic polynom.
 * \param points the source points.
 * \pre points.size() > degree.
 * \pre the number of distinct x-values in points must be greater than degree.
 * \return the fitted polynom (type: DynamicPolynomial).
 */
template <class Range>
DynamicPolynomial fitToPoints(const Range &points, const PolynomialDegree degree) {
  return fitToPoints(std::begin(points), std::end(points), degree);
}

/*!
 * \brief fitToPoints uses Eigens Cholesky decomposition for Least-Squares
 * polynomial fit of 2D data
 *
 * See Eigen Library for more on the underlying functions:
 *   http://eigen.tuxfamily.org/dox/group__TutorialLinearAlgebra.html#title4
 * for Cholesky decomposition see
 *  * http://eigen.tuxfamily.org/dox/group__Cholesky__Module.html
 *  * http://eigen.tuxfamily.org/dox-devel/group__LeastSquares.html
 *
 * This code has been derived from the example code at
 *   https://forum.kde.org/viewtopic.php?f=74&t=118177
 *
 * \tparam DEGREE the degree of the static polynom.
 * \param points the source points.
 * \pre points.size() > DEGREE.
 * \pre the number of distinct x-values in points must be greater than DEGREE.
 * \return the fitted polynom (type: Polynomial<DEGREE, double>).
 */
template <PolynomialDegree DEGREE, class Range>
Polynomial<DEGREE, double> fitToPoints(const Range &points) {
  return fitToPoints<DEGREE>(std::begin(points), std::end(points), DEGREE);
}

/*!
 * \brief absError calculates the absolute y-error of a polynom to a point.
 * \param polynomial the polynomial
 * \param point the point
 * \return the absolute error.
 */
template <class Point>
double absError(const Point &point, const DynamicPolynomial &polynomial) {
  return std::abs(polynomial(point.x()) - point.y());
}

/*!
 * \brief computeFittingError
 *
 *  Computes the average absolute error of the polynom to the source points.
 *
 * \param points source points
 * \param polynomial the polynome
 * \pre points.size() > 0.
 * \return the average absolute error
 */
template <class PointsVector>  // models a vector of Eigen::Vectors
double computeAbsoluteFittingError(const PointsVector &points,
                                   const DynamicPolynomial &polynomial) {
  assert(!points.empty());
  double error = 0.0;
  for (const auto &point : points) {
    error += absError(point, polynomial);
  }
  return error / static_cast<double>(points.size());
}

template <class IT, class CovarianceMatrix>
Eigen::MatrixXd calculateEstimationCovariance(IT begin,
                                              IT end,
                                              const PolynomialDegree degree,
                                              const CovarianceMatrix &covariance) {
  const int num_points = std::distance(begin, end);
  const int num_params = degree + 1;
  Eigen::MatrixXd vandermonde(num_points, num_params);
  std::tie(std::ignore, vandermonde) =
      detail::calculateRegressionSystem(begin, end, degree);
  return (vandermonde.transpose() * covariance.inverse() * vandermonde).inverse();
}
}  // namespace polynomial;
using namespace polynomial;
}  // namespace common;

#endif  // POLYFIT_H
