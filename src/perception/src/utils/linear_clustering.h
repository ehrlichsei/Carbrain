#ifndef LINEAR_CLUSTERING_H
#define LINEAR_CLUSTERING_H

#include <utility>
#include <vector>
#include <boost/range/adaptor/reversed.hpp>
#include <eigen3/Eigen/Dense>

#include "common/math.h"

/*!
 * \brief LinearClustering implements a simple but efficient clustering
 * algorithm for sufficiently ordered data.
 *
 * This algorithm assumes, that the given array is ordered by at least one
 * dimension. This ensures a near linear runtime. For inappropriate data the runtime might be
 * quadratic.
 *
 * \tparam VectorType the vector type to use for clustering.
 * \tparam Metric the metric to use. The metric defines under which conditions
 * two vectors are part of the same cluster.
 * \tparam ScalarType the scalar type to use, defaults to VectorType's scalar
 *type.
 */
template <class VectorType, typename Metric, typename ScalarType = typename VectorType::Scalar>
class LinearClustering {

 public:
  typedef std::vector<VectorType, Eigen::aligned_allocator<VectorType> > ClusterType;

  /*!
   * \brief LinearClustering constructs a clustering-object from a metric.
   * \param is_inlayer the metric to use.
   */
  LinearClustering(const Metric& is_inlayer) : is_inlayer(is_inlayer) {}

  /*!
   * \brief LinearClustering constructs a clustering-object and its metric in
   * place.
   */
  template <typename... Args>
  LinearClustering(Args... args)
      : is_inlayer(Metric(std::forward<Args...>(args...))) {}

  LinearClustering() = delete;

  /*!
   * \brief cluster
   *
   * performs a modified DBSCAN on the given points. This algorithm assumes,
   * that the given array is ordered by at least one dimension. This ensures
   * a near linear runtime. For inappropriate data the runtime might be
   * quadratic.
   *
   * \param in_points the input points.
   * \param clusters the resulting clusters.
   */
  inline void cluster(const ClusterType& in_points, std::vector<ClusterType>* clusters) const {

    for (const VectorType& point : in_points) {
      // find first cluster, which has point as an inlayer.
      // clusters are ordered increasing by time of creation, so we start
      // at the end.
      // This loop is assumed to run way less times than in_points.size(), which
      // results in a linear runtime of the hole algorithm.
      bool added = false;
      for (ClusterType& cluster : boost::adaptors::reverse(*clusters)) {
        // if we find a cluster, add in_points[i] as last element
        if (is_inlayer(cluster.back(), point)) {
          cluster.push_back(point);
          added = true;
          break;
        }
      }

      // if there was no fitting cluster, generate a new one
      if (!added) {
        clusters->push_back({{point}});
      }
    }
  }

  /*!
   * \brief cluster
   *
   * performs a modified DBSCAN on the given points. This algorithm assumes,
   * that the given array is ordered by at least one dimension. This ensures
   * a near linear runtime. For inappropriate data the runtime might be
   * quadratic.
   *
   * \param in_points the input points.
   * \return the resulting clusters.
   */
  inline std::vector<ClusterType> cluster(const ClusterType& in_points) const {
    std::vector<ClusterType> clusters;
    cluster(in_points, &clusters);
    return clusters;
  }

 private:
  Metric is_inlayer;
};

/*!
 * \brief EuclideanTreshold is a metric intended to be used with
 * LinearClustering. It implements a thresholded euclidean norm.
 * This means: the functor returns wheter two data-vectors are colser than
 * threshold.
 */
template <typename VectorType, typename ScalarType = typename VectorType::Scalar>
class EuclideanTreshold {
 public:
  /*!
   * \brief EuclideanThreshold
   * \param threshold the threshold to use.
   */
  EuclideanTreshold(const ScalarType threshold)
      : squared_threshold(common::squared(threshold)) {}

  inline double operator()(const VectorType& x, const VectorType& y) const {
    return (x - y).squaredNorm() < squared_threshold;
  }

 private:
  ScalarType squared_threshold;
};

/*!
 * \brief EuclideanLinearClustering is LinearClustering using EuclideanTreshold.
 * It implements simple linear clustering with euclidean distance.
 */
template <typename VectorType, typename ScalarType = typename VectorType::Scalar>
using EuclideanLinearClustering =
    LinearClustering<VectorType, EuclideanTreshold<VectorType, ScalarType>, ScalarType>;

#endif  // LINEAR_CLUSTERING_H
