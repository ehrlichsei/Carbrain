#ifndef BASIC_STATISTICS_EIGEN_H
#define BASIC_STATISTICS_EIGEN_H

#include "basic_statistics.h"
#include <Eigen/Dense>

namespace common {
namespace basic_statistics_detail {

template<typename _Scalar, int _Rows, int _Cols, int _Options, int _MaxRows, int _MaxCols>
struct zero<Eigen::Matrix<_Scalar, _Rows, _Cols, _Options, _MaxRows, _MaxCols>> {
 static constexpr Eigen::Matrix<_Scalar, _Rows, _Cols, _Options, _MaxRows, _MaxCols> get() {
  return Eigen::Matrix<_Scalar, _Rows, _Cols, _Options, _MaxRows, _MaxCols>::Zero();
 }
};

}
}

#endif // BASIC_STATISTICS_EIGEN_H
