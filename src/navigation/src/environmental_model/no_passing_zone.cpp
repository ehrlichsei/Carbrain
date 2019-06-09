#include "navigation/no_passing_zone.h"

#include <boost/range/algorithm_ext/erase.hpp>
#include <boost/range/algorithm/sort.hpp>
#include <boost/range/adaptor/filtered.hpp>
#include <common/unique_erase.h>

using boost::adaptors::filtered;

const ParameterString<double> NoPassingZonesParameter::PARAM_DECREASE_FACTOR(
    "no_passing_zone/decrease_factor");
const ParameterString<double> NoPassingZonesParameter::PARAM_MERGE_DISTANCE(
    "no_passing_zone/merge_distance");
const ParameterString<double> NoPassingZonesParameter::PARAM_THRESHOLD(
    "no_passing_zone/threshold");
const ParameterString<int> NoPassingZonesParameter::PARAM_NUMBER_MSG_CONSECUTIVE(
    "no_passing_zone/number_msg_consecutive");

NoPassingZones::NoPassingZones(const std::shared_ptr<NoPassingZonesParameter> &parameter)
    : parameter_(parameter) {}

navigation_msgs::NoPassingZones NoPassingZones::calcNoPassingZones(
    const Eigen::Affine3d &vehicle_pose,
    const perception_msgs::NoPassingZones &no_passing_msgs,
    const std::vector<NoPassingZone> &no_passing_zones) {
  if (resetted > 0) {
    resetted--;
    return {};
  }
  const double fix_perception_zone_before_car_distance = 0.7;
  const double delete_distance_behind = 1.0;

  boost::remove_erase_if(no_passing_zones_perception_,
                         [&](const NoPassingZone &zone) {
                           return (vehicle_pose.inverse() * zone.getStart()).x() >
                                      fix_perception_zone_before_car_distance ||
                                  (vehicle_pose.inverse() * zone.getEnd()).x() <
                                      -delete_distance_behind;
                         });
  for (NoPassingZone &zone : no_passing_zones_perception_) {
    if ((vehicle_pose.inverse() * zone.getEnd()).x() > fix_perception_zone_before_car_distance) {
      zone.setEnd(vehicle_pose *
                  Eigen::Vector3d(fix_perception_zone_before_car_distance, 0.25, 0));
    }
    if ((vehicle_pose.inverse() * zone.getStart()).x() < -delete_distance_behind) {
      zone.setStart(vehicle_pose * Eigen::Vector3d(-delete_distance_behind, 0.25, 0));
    }
  }
  const auto new_no_passing_perc = NoPassingZones::fromMessage(no_passing_msgs);
  no_passing_zones_perception_.insert(no_passing_zones_perception_.end(),
                                      new_no_passing_perc.begin(),
                                      new_no_passing_perc.end());
  ROS_DEBUG_STREAM("no_passing_zones_perception_  "
                   << no_passing_zones_perception_.size());

  if (!no_passing_zones_perception_.empty()) {
    mergeNoPassingZones(no_passing_zones_perception_, vehicle_pose);
  }
  current_zones_ = no_passing_zones_perception_;
  current_zones_.insert(
      current_zones_.end(), no_passing_zones.begin(), no_passing_zones.end());
  if (current_zones_.empty()) {
    return {};
  }
  mergeNoPassingZones(current_zones_, vehicle_pose);
  ROS_DEBUG_STREAM("current zones  " << current_zones_.size());
  return toMessage(current_zones_);
}



void NoPassingZones::reset(const int ignore_number_messages_after_reset) {
  resetted = ignore_number_messages_after_reset;
  no_passing_zones_perception_.clear();
  current_zones_.clear();
}


std::vector<NoPassingZone> NoPassingZones::fromMessage(const navigation_msgs::NoPassingZones &no_passing_zones) {
  std::vector<NoPassingZone> zones;
  for (const auto &zone : no_passing_zones.sub_messages) {
    zones.push_back(NoPassingZone::fromMessage(zone));
  }
  return zones;
}
std::vector<NoPassingZone> NoPassingZones::fromMessage(const perception_msgs::NoPassingZones &no_passing_zones) {
  std::vector<NoPassingZone> zones;
  for (const auto &zone : no_passing_zones.sub_messages) {
    zones.push_back(NoPassingZone::fromMessage(zone));
  }
  return zones;
}

bool NoPassingZones::hasNoPassingZones() { return !current_zones_.empty(); }

navigation_msgs::NoPassingZones NoPassingZones::toMessage(const std::vector<NoPassingZone> &zones) const {
  navigation_msgs::NoPassingZones no_passing_zones_msg;
  no_passing_zones_msg.header.frame_id = "world";
  no_passing_zones_msg.header.stamp = ros::Time::now();
  if (number_msg_consecutive < parameter_->number_msg_consecutive) {
    return no_passing_zones_msg;
  }
  no_passing_zones_msg.sub_messages.reserve(zones.size());
  for (const NoPassingZone &zone : zones) {
    no_passing_zones_msg.sub_messages.push_back(zone.toMessage());
  }
  return no_passing_zones_msg;
}

void NoPassingZones::mergeNoPassingZones(std::vector<NoPassingZone> &no_passing_zones,
                                         const Eigen::Affine3d &vehicle_pose) {
  const double merge_distance = parameter_->merge_distance;
  boost::sort(no_passing_zones,
              [&](const auto &a, const auto &b) {
                return (vehicle_pose.inverse() * a.getStart()).x() <
                       (vehicle_pose.inverse() * b.getStart()).x();
              });

  for (size_t i = 0; i < no_passing_zones.size() - 1; ++i) {
    auto zone_direction =
        (no_passing_zones[i].getEnd() - no_passing_zones[i].getStart()).normalized();
    if (zone_direction.dot(no_passing_zones[i + 1].getStart() -
                           no_passing_zones[i].getEnd()) < merge_distance) {
      if ((vehicle_pose.inverse() * no_passing_zones[i].getEnd()).x() <
          (vehicle_pose.inverse() * no_passing_zones[i + 1].getEnd()).x()) {
        no_passing_zones[i].setEnd(no_passing_zones[i + 1].getEnd());
      }
      no_passing_zones.erase(no_passing_zones.begin() + i + 1);
      i--;
    }
  }
}

NoPassingZone::NoPassingZone(const Eigen::Vector3d &start, const Eigen::Vector3d &end)
    : start(start), end(end) {}

void NoPassingZone::setStart(const Eigen::Vector3d &start) {
  this->start = start;
}

void NoPassingZone::setEnd(const Eigen::Vector3d &end) { this->end = end; }

Eigen::Vector3d NoPassingZone::getStart() const { return start; }

Eigen::Vector3d NoPassingZone::getEnd() const { return end; }

bool NoPassingZone::pointIsInNoPassingZone(const Eigen::Vector3d &point) const {
  Eigen::Vector3d direction = end - start;
  double zone_length = (end - start).norm();
  Eigen::Vector3d v = point - start;
  return v.norm() < zone_length && v.dot(direction) > 0;
}

double NoPassingZone::getSafetyMarginAfterZone() const {
  return safety_margin_after_zone;
}

navigation_msgs::NoPassingZone NoPassingZone::toMessage() const {
  navigation_msgs::NoPassingZone zone;
  zone.start = tf2::toMsg(start);
  zone.end = tf2::toMsg(end);
  return zone;
}

NoPassingZone NoPassingZone::fromMessage(const navigation_msgs::NoPassingZone &no_passing_zone) {
  NoPassingZone zone;
  tf2::fromMsg(no_passing_zone.start, zone.start);
  tf2::fromMsg(no_passing_zone.end, zone.end);
  return zone;
}
NoPassingZone NoPassingZone::fromMessage(const perception_msgs::NoPassingZone &no_passing_zone) {
  NoPassingZone zone;
  tf2::fromMsg(no_passing_zone.start, zone.start);
  tf2::fromMsg(no_passing_zone.end, zone.end);
  return zone;
}

NoPassingZonesParameter::NoPassingZonesParameter(common::node_base::ParameterInterface *parameters) {
  parameters->registerParam(PARAM_DECREASE_FACTOR);
  parameters->registerParam(PARAM_MERGE_DISTANCE);
  parameters->registerParam(PARAM_THRESHOLD);
  parameters->registerParam(PARAM_NUMBER_MSG_CONSECUTIVE);
  updateParameter(parameters);
}

void NoPassingZonesParameter::updateParameter(common::node_base::ParameterInterface *parameters) {
  merge_distance = parameters->getParam(PARAM_MERGE_DISTANCE);
  number_msg_consecutive = parameters->getParam(PARAM_NUMBER_MSG_CONSECUTIVE);
  decrease_factor = parameters->getParam(PARAM_DECREASE_FACTOR);
  threshold = parameters->getParam(PARAM_THRESHOLD);
}
