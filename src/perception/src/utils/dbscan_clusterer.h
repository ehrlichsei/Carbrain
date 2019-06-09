#ifndef DBSCAN_CLUSTERER_H
#define DBSCAN_CLUSTERER_H

#include "common/camera_transformation.h"
#include "common/parameter_interface.h"
#include "perception_types.h"

struct FeaturePointCluster {
 public:
  FeaturePointCluster(const common::CameraTransformation &cam_transform,
                      const ImagePoints &feature_points);
  FeaturePointCluster(const ImagePoints &feature_points_image,
                      const VehiclePoints &feature_points_vehicle);
  FeaturePointCluster(const common::CameraTransformation &cam_transform,
                      const VehiclePoints &feature_points);

  /**
   * \brief image coordinates of feature points
   */
  ImagePoints feature_points_img;

  /**
   * \brief vehicle coordinates of feature points
   */
  VehiclePoints feature_points_vehicle;

  /**
   * id for debug purpose
   */
  unsigned int id;

  static unsigned int nextClusterId;
};

class DBScanClusterer {
 public:
  DBScanClusterer(ParameterInterface *parameters,
                  const std::string &namespace_ = "dbscan");

  //! \brief clusters the input feature point cluster using simplified DBScan
  //! algorithm; Consider Wikipedia pseudo code as this was derived from there
  std::vector<FeaturePointCluster> cluster(const FeaturePointCluster &input_cluster) const;

 private:
  const ParameterInterface *const parameters_;

  //! \brief return all points within p's epsilon-neighborhood (including p)
  std::vector<unsigned int> neighborhoodQuery(const FeaturePointCluster &input_cluster,
                                              int p,
                                              double epsilon) const;

  //! \brief expand cluster using core points given in 'core_pts'
  //! marks all core points and their satellites with 'cluster_no' in
  //! 'fp_assignment'
  void expandCluster(const FeaturePointCluster &input_cluster,
                     std::vector<unsigned int> &core_pts,
                     std::vector<bool> &pt_visited,
                     std::vector<int> &pt_assignment,
                     int cluster_no) const;

  //! \brief creates vector of output clusters by dividing 'input_cluster'
  //! using 'pt_assignment'
  std::vector<FeaturePointCluster> createClusters(const FeaturePointCluster &input_cluster,
                                                  const std::vector<int> &pt_assignment,
                                                  int clusters) const;

  const ParameterString<double> DBSCAN_CORE_EPSILON;
  const ParameterString<int> DBSCAN_MIN_POINTS_TO_BE_CORE;
};

#endif  // DBSCAN_CLUSTERER_H
