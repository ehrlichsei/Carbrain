#ifndef TYPES_H
#define TYPES_H

#include <vector>
#include <eigen3/Eigen/Dense>

namespace common {
/*!
 * \namespace common::types
 * \brief types contains definitions of KITcar-specific types.
 */
namespace types {
template <typename T>
using EigenAlignedVector = std::vector<T, Eigen::aligned_allocator<T>>;

typedef EigenAlignedVector<Eigen::Vector2d> Vector2dVector;
typedef EigenAlignedVector<Eigen::Vector2i> Vector2iVector;
typedef EigenAlignedVector<Eigen::Vector3d> Vector3dVector;
}  // types
using namespace types;
}  // commone

#endif  // TYPES_H
