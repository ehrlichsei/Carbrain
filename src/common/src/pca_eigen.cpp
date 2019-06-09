#include <eigen3/Eigen/SVD>

#include "common/pca_eigen.h"
#include "common/math.h"

#include <cmath>

namespace common {
namespace pca_eigen {

using namespace Eigen;

void principle_component_analysis(const Eigen::MatrixXd& data_points,
                                  Eigen::MatrixXd* eigenvectors,
                                  Eigen::VectorXd* eigenvalues) {
  // Compute centered covariance matrix
  const MatrixXd centered = data_points.rowwise() - data_points.colwise().mean();
  const MatrixXd cov =
      centered.adjoint() * centered / static_cast<double>(data_points.cols());

  // Perform eigendecomposition
  SelfAdjointEigenSolver<Eigen::MatrixXd> eigen_solver(cov);

  // Output the sorted eigenvectors and -values
  *eigenvectors = eigen_solver.eigenvectors();
  *eigenvalues = eigen_solver.eigenvalues();
}

void pca_svd(const Eigen::MatrixXd& data_points,
             Eigen::MatrixXd* components,
             Eigen::VectorXd* scores) {
  assert(data_points.cols() <= data_points.rows() &&
         "Not enough data points for pca!");
  const MatrixXd centered = data_points.rowwise() - data_points.colwise().mean();
  JacobiSVD<MatrixXd> svd(centered, ComputeThinV);
  svd.computeV();
  *scores = svd.singularValues();
  *components = svd.matrixV();
}

double getAngle(const Eigen::Vector2d& a, const Eigen::Vector2d& b) {
  return wrapAngleMinusPiToPi(std::atan2(a(1), a(0)) - std::atan2(b(1), b(0)));
}

Vector2d getPrincipalComponent(const Eigen::MatrixXd& data_points) {
  MatrixXd components;
  VectorXd scores;
  pca_svd(data_points, &components, &scores);

  return {components(0, 0), components(1, 0)};
}

double getAngleToPrincipalComponent(const Eigen::Vector2d& reference,
                                    const Eigen::MatrixXd& data_points) {
  const Vector2d principal_component = getPrincipalComponent(data_points);
  return getAngle(principal_component, reference);
}

}  // namespace pca_eigen
}  // namespace common
