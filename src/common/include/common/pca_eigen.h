#ifndef PCA_EIGEN_H
#define PCA_EIGEN_H

#include "common/types.h"
#include "common/type_traits.h"
#include "common/eigen_utils.h"
#include "common/functors.h"
#include <boost/range/algorithm/sort.hpp>

namespace common {
// clang-format off
/*!
 * \namespace common::pca_eigen
 * \brief pca_eigen contains functionality related to the
 * <a href="https://en.wikipedia.org/wiki/Principal_component_analysis">principle_component_analysis</a>.
 */
// clang-format on
namespace pca_eigen {
/*!
 * \brief principle_component_analysis
 *
 * PCA implementation found at http://forum.kde.org/viewtopic.php?f=74&t=110265
 *
 * \param data_points the points the PCA will be computed on.
 * \param eigenvectors the computed eigen vectors.
 * \param eigenvalues the computed eigen values.
 * \pre data_points must contain at least as many data points as the dimension
 *  of a data point.
 */
void principle_component_analysis(const Eigen::MatrixXd& data_points,
                                  Eigen::MatrixXd* eigenvectors,
                                  Eigen::VectorXd* eigenvalues);

/*!
 * \brief pca_svd performs a principal component analysis by computing the
 * singular value decomposition of the input matrix.
 *
 * \param data_points the matrix, containing the sample points as rows
 * \param components the output matrix that contains the component vectors
 *  column-wise.
 * \param scores the scores of the pca.
 * \pre data_points must contain at least as many data points as the dimension
  of a data point.
 */
void pca_svd(const Eigen::MatrixXd& data_points,
             Eigen::MatrixXd* components,
             Eigen::VectorXd* scores);

/*!
 * \brief getPrincipalComponent returns the largest component of the passed data
 * points. Internally, pca_svd is used.
 * \param data_points the input data organized in a matrix.
 * \return the vector representing the direction of the largest component.
 * \pre data_points must contain at least 2 points.
 */
Eigen::Vector2d getPrincipalComponent(const Eigen::MatrixXd& data_points);

/*!
 * \brief getPrincipalComponent returns the largest component of the passed data
 * points. Internally, pca_svd is used.
 * \param range the input data organized in a range (e.g. a std::vector<>).
 * \return the vector representing the direction of the largest component.
 * \pre range must contain at least 2 points.
 */
template <class Range, typename = std::enable_if_t<common::is_range<Range>()>>
Eigen::Vector2d getPrincipalComponent(const Range& range) {
  const Eigen::MatrixXd data = toMatrix2D(range);
  return getPrincipalComponent(data);
}

/*!
 * \brief returns the angle between a reference vector and the principal
 * component vector of a set of points.
 * \param reference the rotation vector specifying the reference rotation.
 * \param data_points the set of 2-dimensional points of which the principal
 * component is calculated.
 * \return the angle between the principal component vector and the reference
 * vector.
 * \pre data_points must contain at least 2 points.
 */
double getAngleToPrincipalComponent(const Eigen::Vector2d& reference,
                                    const Eigen::MatrixXd& data_points);
/*!
 * \brief returns the angle between a reference vector and the principal
 * component vector of a set of points.
 * \param reference the rotation vector specifying the reference rotation.
 * \param range the set of 2-dimensional points of which the principal component
 * is calculated.
 * \return the angle between the principal component vector and the reference
 * vector.
 * \pre range must contain at least 2 points.
 */
template <class Range, typename = std::enable_if_t<common::is_range<Range>()>>
double getAngleToPrincipalComponent(const Eigen::Vector2d& reference, const Range& range) {
  const Eigen::MatrixXd data = toMatrix2D(range);
  return getAngleToPrincipalComponent(reference, data);
}

/**
 * @brief getAngle computes the angle between 2 2D directions.
 * @param a the first direction vector
 * @param b the second direction vector
 * @return the angle between a and b, in the range [-PI;PI]
 */
double getAngle(const Eigen::Vector2d& a, const Eigen::Vector2d& b);

/*!
 * \brief sortAlgonPrincipalComponent sorts given vector of points along its
 * principal component.
 * \param points the vector to sort.
 */
template <typename V>
void sortAlongPrincipalComponent(EigenAlignedVector<V>* points) {
  if (points->size() < 2) {
    return;
  }

  V v = V::Zero();
  v.template head<2>() = getPrincipalComponent(toMatrix2D(*points));
  // dodge antiparallel case to avoid complete inversion of order.
  const V v_p = ensureSameOrientation<V>(v, V::UnitX());
  boost::sort(*points, accordingTo([&v_p](const V& a) { return a.dot(v_p); }));
}

}  // namespace pca

using namespace pca_eigen;

}  // namespace common
#endif  // PCA_EIGEN_H
