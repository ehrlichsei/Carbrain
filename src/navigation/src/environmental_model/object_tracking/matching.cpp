#include "matching.h"
#include "common/best_score.h"
#include "common/eigen_functors.h"
#include <common/macros.h>
THIRD_PARTY_HEADERS_BEGIN
#include <unordered_set>
#include <numeric>
#include <boost/range/algorithm_ext/erase.hpp>
THIRD_PARTY_HEADERS_END

namespace environmental_model {

Matching::Matching(const std::shared_ptr<TrackingElementParameter> &tracking_elmement_parameter)
    : tracking_elmement_parameter_(tracking_elmement_parameter) {}

void Matching::assignNewElementsToTracks(std::vector<std::shared_ptr<RoadElement>> &new_road_elements,
                                         std::vector<TrackingElement> &tracking_elements,
                                         const perception_msgs::Unidentifieds &unidentifieds,
                                         const Eigen::Affine3d &vehicle_pose) {
  sortElements(new_road_elements);
  std::vector<std::vector<std::shared_ptr<RoadElement>>> clusters_per_tracking_element(
      tracking_elements.size());
  for (const auto &element : new_road_elements) {
    assignBestTrackingElement(element, tracking_elements, clusters_per_tracking_element);
  }
  const auto positions_unidentifieds = calcPositionsUnidentifieds(unidentifieds);

  for (size_t i = 0; i < tracking_elements.size(); ++i) {
    if (clusters_per_tracking_element[i].empty()) {
      Eigen::Vector3d centroid;
      if (!calcCentroid(tracking_elements[i].getBaseArea(), centroid)) {
        tracking_elements[i].decay(10.0);
        continue;
      }

      if (tracking_elements[i].stopDecreasingProb(vehicle_pose)) {
        continue;
      }

      double min_dist_unidentified = 10.0;
      if (!positions_unidentifieds.empty()) {
        min_dist_unidentified =
            (*common::min_score(positions_unidentifieds, common::distanceTo(centroid)) -
             centroid).norm();
      }
      tracking_elements[i].decay(min_dist_unidentified);
      continue;
    }
    tracking_elements[i].update(clusters_per_tracking_element[i], vehicle_pose);
  }
  boost::remove_erase_if(
      tracking_elements,
      [&](const auto &element) { return element.hasInvalidRoadElement(); });
  mergeTrackingElements(tracking_elements, vehicle_pose);
}

void Matching::assignBestTrackingElement(
    const std::shared_ptr<RoadElement> &road_element,
    std::vector<TrackingElement> &tracking_elements,
    std::vector<std::vector<std::shared_ptr<RoadElement>>> &new_elements_per_tracking_element) {
  auto new_element_polygon = createPolygon(road_element->getBaseArea());
  static unsigned int next_id = 0;
  const double max_distance = 0.15;
  if (!tracking_elements.empty()) {
    size_t index_best_tracking_element = 0;
    auto dist_to_best_tracking_element = distanceForMatching(
        new_element_polygon, tracking_elements.front().getBaseArea());
    for (size_t i = 0; i < tracking_elements.size(); ++i) {
      const auto current_distance = distanceForMatching(
          new_element_polygon, tracking_elements[i].getBaseArea());
      if (dist_to_best_tracking_element.first > current_distance.first ||
          (current_distance.first == 0 &&
           current_distance.second < dist_to_best_tracking_element.second)) {
        dist_to_best_tracking_element = current_distance;
        index_best_tracking_element = i;
      }
    }
    if (dist_to_best_tracking_element.first < max_distance) {
      new_elements_per_tracking_element[index_best_tracking_element].push_back(road_element);
      return;
    }
  }
  TrackingElement::ProbabilityVector probabilities(
      TrackingElement::ProbabilityVector::Zero());
  road_element->setProbabilityFromLastMsg(probabilities);

  bool create_new_tracking_element = false;
  for (int i = 0; i < probabilities.size(); ++i) {
    if (probabilities[i] > tracking_elmement_parameter_->min_prob_new_tracking_[i]) {
      create_new_tracking_element = true;
      break;
    }
  }
  if (!create_new_tracking_element) {
    return;
  }

  tracking_elements.emplace_back(next_id++, tracking_elmement_parameter_);
  new_elements_per_tracking_element.emplace_back();
  tracking_elements.back().setBaseArea(new_element_polygon);
  new_elements_per_tracking_element.back().push_back(road_element);
}

void Matching::sortElements(std::vector<std::shared_ptr<RoadElement>> &new_road_elements) {
  std::sort(new_road_elements.begin(),
            new_road_elements.end(),
            [](const auto &a, const auto &b) {
              return a->getCertainty() > b->getCertainty();
            });
}

void Matching::mergeTrackingElements(std::vector<TrackingElement> &tracking_elements,
                                     const Eigen::Affine3d &vehicle_pose) {
  if (tracking_elements.empty()) {
    return;
  }
  for (size_t i = 0; i < tracking_elements.size() - 1; ++i) {
    tracking_elements.erase(
        std::remove_if(
            tracking_elements.begin() + i + 1,
            tracking_elements.end(),
            [&](auto &other_element) {
              auto distance = distanceForMatching(
                  tracking_elements[i].getBaseArea(), other_element.getBaseArea());
              bool merge = distance.first == 0;
              merge |= distance.second < TrackingElement::getMaxMergeDistance(
                                             tracking_elements[i], other_element);
              if (merge) {
                tracking_elements[i].merge(other_element, vehicle_pose);
                return true;
              }
              return false;
            }),
        tracking_elements.end());
  }
}

std::vector<Eigen::Vector3d> Matching::calcPositionsUnidentifieds(
    const perception_msgs::Unidentifieds &unidentifieds) {
  std::vector<Eigen::Vector3d> positions_unidentifieds;
  positions_unidentifieds.reserve(unidentifieds.sub_messages.size());
  const auto unidentifiedToPosition = [](const auto &unidentified) {
    if (unidentified.hull_polygon.empty()) {
      return Eigen::Vector3d(0, 0, 0);
    }

    Eigen::Vector3d sum_hull_polygon(Eigen::Vector3d::Zero());
    for (const geometry_msgs::Point &point : unidentified.hull_polygon) {
      Eigen::Vector3d v = Eigen::Vector3d::Zero();
      tf2::fromMsg(point, v);
      sum_hull_polygon += v;
    }
    const Eigen::Vector3d estimated_center_hull_polygon =
        sum_hull_polygon / unidentified.hull_polygon.size();
    return estimated_center_hull_polygon;
  };
  std::transform(unidentifieds.sub_messages.begin(),
                 unidentifieds.sub_messages.end(),
                 std::back_inserter(positions_unidentifieds),
                 unidentifiedToPosition);
  return positions_unidentifieds;
}
}  // namespace environmental_model
