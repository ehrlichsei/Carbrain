#include "dbscan_clusterer.h"
#include <common/macros.h>

THIRD_PARTY_HEADERS_BEGIN
#include <ros/console.h>
THIRD_PARTY_HEADERS_END

unsigned int FeaturePointCluster::nextClusterId = 0;

FeaturePointCluster::FeaturePointCluster(const common::CameraTransformation &cam_transform,
                                         const ImagePoints &feature_points) {
  feature_points_img = feature_points;
  cam_transform.transformImageToGround(feature_points_img, &feature_points_vehicle);
  id = nextClusterId;
  nextClusterId++;
}

FeaturePointCluster::FeaturePointCluster(const ImagePoints &feature_points_image,
                                         const VehiclePoints &feature_points_vehicle)
    : feature_points_img(feature_points_image),
      feature_points_vehicle(feature_points_vehicle),
      id(nextClusterId) {
  nextClusterId++;
}

FeaturePointCluster::FeaturePointCluster(const common::CameraTransformation &cam_transform,
                                         const VehiclePoints &feature_points)
    : feature_points_vehicle(feature_points), id(nextClusterId) {
  cam_transform.transformGroundToImage(feature_points_vehicle, &feature_points_img);
  nextClusterId++;
}

DBScanClusterer::DBScanClusterer(ParameterInterface *parameters, const std::string &namespace_)
    : parameters_(parameters),
      DBSCAN_CORE_EPSILON(namespace_ + "/core_epsilon"),
      DBSCAN_MIN_POINTS_TO_BE_CORE(namespace_ + "/min_points_to_be_core") {
  parameters->registerParam(DBSCAN_CORE_EPSILON);
  parameters->registerParam(DBSCAN_MIN_POINTS_TO_BE_CORE);
}

std::vector<FeaturePointCluster> DBScanClusterer::cluster(const FeaturePointCluster &input_cluster) const {

  const double core_epsilon = parameters_->getParam(DBSCAN_CORE_EPSILON);
  const size_t min_pts_to_be_core =
      static_cast<size_t>(parameters_->getParam(DBSCAN_MIN_POINTS_TO_BE_CORE));
  //! \brief current number of cluster
  int cluster_no = -1;
  std::vector<bool> pt_visited(input_cluster.feature_points_vehicle.size(), false);
  //! \brief contains assignment of feature points to clusters (int = cluster
  //! no)
  //! -2 signals not assigned
  //! -1 signals noise cluster (will be ommitted here)
  std::vector<int> pt_assignment(input_cluster.feature_points_vehicle.size(), -2);

  for (size_t i = 0; i < input_cluster.feature_points_vehicle.size(); i++) {
    if (pt_visited[i])
      continue;
    pt_visited[i] = true;
    std::vector<unsigned int> core_pts = neighborhoodQuery(input_cluster, i, core_epsilon);
    if (core_pts.size() < min_pts_to_be_core) {
      // mark as noise
      pt_assignment[i] = -1;
    } else {
      // create new cluster:
      cluster_no += 1;
      pt_assignment[i] = cluster_no;
      expandCluster(input_cluster, core_pts, pt_visited, pt_assignment, cluster_no);
    }
  }
  return createClusters(input_cluster, pt_assignment, cluster_no);
}

std::vector<unsigned int> DBScanClusterer::neighborhoodQuery(
    const FeaturePointCluster &input_cluster, int p, double epsilon) const {
  std::vector<unsigned int> epsilon_neighbors;
  for (unsigned int i = 0; i < input_cluster.feature_points_vehicle.size(); i++) {
    if ((input_cluster.feature_points_vehicle[i] - input_cluster.feature_points_vehicle[p])
            .squaredNorm() < epsilon * epsilon) {
      epsilon_neighbors.push_back(i);
    }
  }
  return epsilon_neighbors;
}

void DBScanClusterer::expandCluster(const FeaturePointCluster &input_cluster,
                                    std::vector<unsigned int> &core_pts,
                                    std::vector<bool> &pt_visited,
                                    std::vector<int> &pt_assignment,
                                    int cluster_no) const {
  const double core_epsilon = parameters_->getParam(DBSCAN_CORE_EPSILON);
  const size_t min_pts_to_be_core =
      static_cast<size_t>(parameters_->getParam(DBSCAN_MIN_POINTS_TO_BE_CORE));
  bool changed = true;
  while (changed) {
    changed = false;
    for (unsigned int i = 0; i < core_pts.size(); i++) {
      if (!pt_visited[core_pts[i]]) {
        pt_visited[core_pts[i]] = true;
        pt_assignment[core_pts[i]] = cluster_no;
        std::vector<unsigned int> epsilon_neighbors =
            neighborhoodQuery(input_cluster, core_pts[i], core_epsilon);
        if (epsilon_neighbors.size() > min_pts_to_be_core) {
          // append found core pts
          core_pts.insert(
              core_pts.end(), epsilon_neighbors.begin(), epsilon_neighbors.end());
          changed = true;
        }
      }
      if (pt_assignment[core_pts[i]] != -2)  // if not assigned yet
        pt_assignment[core_pts[i]] = cluster_no;
    }
  }
}

std::vector<FeaturePointCluster> DBScanClusterer::createClusters(const FeaturePointCluster &input_cluster,
                                                                 const std::vector<int> &pt_assignment,
                                                                 int clusters) const {
  assert(input_cluster.feature_points_vehicle.size() == pt_assignment.size());
  std::vector<FeaturePointCluster> fp_clusters;

  if (clusters < 0) {
    return fp_clusters;
  }

  const size_t min_pts_to_be_core =
      static_cast<size_t>(parameters_->getParam(DBSCAN_MIN_POINTS_TO_BE_CORE));
  fp_clusters.reserve(clusters);
  for (int i = 0; i <= clusters; i++) {
    ImagePoints feature_points_image;
    VehiclePoints feature_points_vehicle;
    for (unsigned int j = 0; j < pt_assignment.size(); j++) {
      if (pt_assignment[j] == i) {
        feature_points_image.push_back(input_cluster.feature_points_img[j]);
        feature_points_vehicle.push_back(input_cluster.feature_points_vehicle[j]);
      }
    }
    if (feature_points_image.size() < min_pts_to_be_core)
      ROS_WARN_STREAM("feature_points2i.size()=" << feature_points_image.size());
    fp_clusters.emplace_back(feature_points_image, feature_points_vehicle);
  }
  return fp_clusters;
}
