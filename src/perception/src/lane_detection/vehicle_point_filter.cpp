#include "vehicle_point_filter.h"
#include "common/console_colors.h"

#include <boost/range/algorithm_ext/push_back.hpp>

#include "../utils/linear_clustering.h"

using EuclideanVehiclePointClustering = EuclideanLinearClustering<VehiclePoint>;

VehiclePointFilter::VehiclePointFilter(ParameterInterface* parameter_interface,
                                       const std::string& sub_name_space)
    : parameter_ptr(parameter_interface),
      POINT_FILTERING_DIST_THRESH(sub_name_space + "/dist_thresh"),
      POINT_FILTERING_COUNT_TRESH(sub_name_space + "/count_thresh"),
      POINT_FILTERING_COUNT_THRESH_RATIO(sub_name_space +
                                         "/count_thresh_ratio"),
      POINT_FILTERING_NEAR_FIELD_DIST(sub_name_space + "/near_field_dist"),
      POINT_FILTERING_PRE_FITLERING_COUNT_TRESH(sub_name_space +
                                                "/prefiltering/count_thresh"),
      POINT_FILTERING_PRE_FILTERING_DIST_THRESH(sub_name_space +
                                                "/prefiltering/dist_thresh") {
  registerParameters(parameter_interface);
}

void VehiclePointFilter::registerParameters(ParameterInterface* parameter_interface) {
  parameter_interface->registerParam(POINT_FILTERING_DIST_THRESH);
  parameter_interface->registerParam(POINT_FILTERING_COUNT_TRESH);
  parameter_interface->registerParam(POINT_FILTERING_COUNT_THRESH_RATIO);
  parameter_interface->registerParam(POINT_FILTERING_NEAR_FIELD_DIST);
  parameter_interface->registerParam(POINT_FILTERING_PRE_FITLERING_COUNT_TRESH);
  parameter_interface->registerParam(POINT_FILTERING_PRE_FILTERING_DIST_THRESH);
}

VehiclePoints VehiclePointFilter::prefilter(const VehiclePoints& ps) const {
  const double dist_thresh =
      parameter_ptr->getParam(POINT_FILTERING_PRE_FILTERING_DIST_THRESH);
  const std::size_t count_thresh =
      parameter_ptr->getParam(POINT_FILTERING_PRE_FITLERING_COUNT_TRESH);

  const auto cs = EuclideanVehiclePointClustering(dist_thresh).cluster(ps);

  VehiclePoints pps;
  pps.reserve(ps.size());
  for (auto& c : cs) {
    ROS_DEBUG_STREAM(COLOR_NORMAL << "prefiltering cluster size: " << c.size() << COLOR_DEBUG);
    if (c.size() > count_thresh) {
      boost::push_back(pps, c);
    }
  }
  return pps;
}

void VehiclePointFilter::filterPoints(const VehiclePoints& in, VehiclePoints* out) const {

  if (in.empty()) {
    return;
  }

  const VehiclePoints pps = prefilter(in);

  const double dist_thresh = parameter_ptr->getParam(POINT_FILTERING_DIST_THRESH);
  const double count_thresh_ratio =
      parameter_ptr->getParam(POINT_FILTERING_COUNT_THRESH_RATIO);
  const std::size_t count_thresh =
      static_cast<std::size_t>(parameter_ptr->getParam(POINT_FILTERING_COUNT_TRESH));
  const double near_field_dist = parameter_ptr->getParam(POINT_FILTERING_NEAR_FIELD_DIST);

  const auto clusters = EuclideanVehiclePointClustering(dist_thresh).cluster(pps);

  out->reserve(in.size());
  const std::size_t thresh = std::max(
      static_cast<std::size_t>(std::floor(static_cast<double>(pps.size()) * count_thresh_ratio)),
      count_thresh);
  ROS_DEBUG("Thresh: %zu", thresh);
  for (const VehiclePoints& current_cluster : clusters) {
    ROS_DEBUG("size: %zu", current_cluster.size());
    if (((current_cluster.front()(0) > near_field_dist && current_cluster.back()(0) > near_field_dist) ||
         count_thresh > current_cluster.size()) &&
        thresh > current_cluster.size()) {
      ROS_DEBUG("Discarded");
      continue;
    }

    ROS_DEBUG("Taken");
    if (current_cluster.size() > 2) {
      out->insert(out->end(), current_cluster.begin() + 1, current_cluster.end() - 1);  // Start end Endpoints tend to be outlayers, lets drop them overall.
    } else {
      out->insert(out->end(), current_cluster.begin(), current_cluster.end());  // Just not if they are the only points in the cluster.
    }
  }
}
