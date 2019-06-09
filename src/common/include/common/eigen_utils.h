#ifndef EIGEN_UTILS_H
#define EIGEN_UTILS_H

#include "common/types.h"
#include <cassert>
#include <Eigen/Dense>

namespace common {
namespace eigen_detail {
template <class Matrix>
inline constexpr bool is_vector() {
  return Eigen::internal::traits<Matrix>::ColsAtCompileTime == 1;
}

}  // namespace detail

/*!
 * \namespace common::eigen_utils
 * \brief eigen_utils contains some useful function to ease the use of Eigen.
 */
namespace eigen_utils {
// clang-format off
/*!
 * \brief toVector converts the given expression to an Eigen vector.
 *
 * This is useful in function-templates. Eigen relieas heavly on 
 * <a href="https://en.wikipedia.org/wiki/Expression_templates">Expression Templates</a>.
 * If one needs a real Eigen-vector and not an expression which
 * represents a vector, toVector() is useful to get this vector.
 *
 * \param b the expression to convert to a vector.
 * \return a vector-representation of the given expression.
 */
// clang-format on
template <class Matrix>
inline Eigen::Matrix<double, Matrix::RowsAtCompileTime, 1> toVector(const Matrix& b) {
  static_assert(eigen_detail::is_vector<Matrix>(),
                "only vectors (no RowVectors) supported!");
  return Eigen::Matrix<double, Matrix::RowsAtCompileTime, 1>(b);
}

/*!
 * \brief to converts a given vector the given vector-type.
 * \param s the vector to convert.
 * \return the converted vector.
 */
template <class Target,
          class Source,
          typename = std::enable_if_t<eigen_detail::is_vector<Target>()>,
          typename = std::enable_if_t<eigen_detail::is_vector<Source>()>>
Target to(const Source& s) {
  constexpr int rows = (int(Source::RowsAtCompileTime) < int(Target::RowsAtCompileTime))
                           ? int(Source::RowsAtCompileTime)
                           : int(Target::RowsAtCompileTime);
  Target t = Target::Zero();
  t.template head<rows>() =
      s.template cast<typename Target::Scalar>().template head<rows>();
  return t;
}

/*!
 * \brief to3D converts a 2D vector to a 3D vector.
 * \param v the 2D vector.
 * \return a 3D vector.
 */
template <class Source, typename = std::enable_if_t<eigen_detail::is_vector<Source>()>>
inline auto to3D(const Source& v) {
  return to<Eigen::Matrix<typename Source::Scalar, 3, 1>>(v);
}

/*!
 * \brief to2D converts a given vector to a 2D vector.
 * \param v an vector.
 * \return the 2D vector.
 */
template <class Source, typename = std::enable_if_t<eigen_detail::is_vector<Source>()>>
inline auto to2D(const Source& v) {
  return to<Eigen::Matrix<typename Source::Scalar, 2, 1>>(v);
}

/*!
 * \brief toMatrix2D converts a range to an Eigen::Matrix with 2 columns.
 * \param begin the iterator to the first element of the range.
 * \param end the iterator after the last element.
 * \return a Eigen::Matrix (default: Eigen::MatrixXd) containing the elements
 *  of the given range.
 *  \tparam TargetMatrix the desired matrix type (default: Eigen::MatirxXd).
 */
template <typename TargetMatrix = Eigen::MatrixXd, typename IT>
inline auto toMatrix2D(IT begin, IT end) {
  const std::size_t size = std::distance(begin, end);
  using T = typename TargetMatrix::Scalar;
  TargetMatrix points_matrix(size, 2);
  for (size_t i = 0; i < size; i++, begin++) {
    points_matrix.row(i) = begin->template head<2>().template cast<T>();
  }
  return points_matrix;
}

/*!
 * \brief toMatrix2D converts a range to an Eigen::Matrix with 2 columns.
 * \param range the range.
 * \return a Eigen::Matrix (default: Eigen::MatrixXd) containing the elements
 *  of the given range.
 *  \tparam TargetMatrix the desired matrix type (default: Eigen::MatirxXd).
 */
template <typename TargetMatrix = Eigen::MatrixXd, typename V>
inline auto toMatrix2D(const V& range) {
  return toMatrix2D(std::begin(range), std::end(range));
}

/*!
 * \brief ensureSameOrientation makes sure that the first given vector points in
 * the same direction as the second given vector.
 * \param v the vector whose orientation will be ensured.
 * \param pd the reference vector.
 * \return v, if v points in the same direction a pd, else -v.
 * \post ret == v or ret == -v and ret.dot(pd) >= 0.0.
 */
template <typename V, typename V2>
inline auto ensureSameOrientation(const V& v, const V2& pd) {
  static_assert(eigen_detail::is_vector<V2>(),
                "only vectors (no RowVectors) supported!");
  static_assert(static_cast<int>(Eigen::internal::traits<V>::RowsAtCompileTime) ==
                    static_cast<int>(Eigen::internal::traits<V2>::RowsAtCompileTime),
                "");
  const auto vv = toVector(v);
  return (v.dot(pd) < 0.0) ? -vv : vv;
}

/*!
 * \brief to2D converts a given 3D-pose to a 2D-pose (projection on the
 * xy-plane).
 * \param p the pose to project  on the xy-plane.
 * \return the projection of p on the xy-plane.
 */
inline Eigen::Affine2d to2D(const Eigen::Affine3d& p) {
  return Eigen::Translation2d(p.translation().topRows<2>()) *
         p.linear().topLeftCorner<2, 2>();
}

/*!
 * \brief to3D converts a given 2D-pose to a 3D-pose.
 * \param p the 3D-pose to be converted.
 * \return the 2D-pose.
 */
inline Eigen::Affine3d to3D(const Eigen::Affine2d& p) {
  auto p3 = Eigen::Affine3d::Identity();
  p3.translation().head<2>() = p.translation();
  p3.linear().topLeftCorner<2, 2>() = p.linear();
  return p3;
}

/*!
 * \brief toHomogenous converts a given point to a homogenous vector.
 * \param p the point to convert.
 * \return a homogenous representation of p: [p, 1]
 */
inline Eigen::Vector3d toHomogenous(const Eigen::Vector2d& p) {
  return {p.x(), p.y(), 1};
}

/*!
 * \brief toHomogenous converts a given point to a homogenous vector.
 * \param p the point to convert.
 * \return a homogenous representation of p: [p, 1]
 */
inline Eigen::Vector4d toHomogenous(const Eigen::Vector3d& p) {
  return {p.x(), p.y(), p.z(), 1};
}

/*!
 * \brief round rounds a given vector and converts it to int.
 * \param p the vector to convert.
 * \return the rounded vector.
 */
inline Eigen::Vector2i round(const Eigen::Vector2d& p) {
  return {static_cast<int>(std::round(p.x())), static_cast<int>(std::round(p.y()))};
}

}  // namespace eigen_utils
using namespace eigen_utils;
}  // namespace common
using common::to3D;
using common::to2D;
#endif  // EIGEN_UTILS_H
