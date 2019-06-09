#ifndef NO_PASSING_ZONE_H
#define NO_PASSING_ZONE_H

#include <common/macros.h>
#include "common/parameter_handler.h"

THIRD_PARTY_HEADERS_BEGIN
#include <ros/console.h>
#include <nav_msgs/Path.h>
#include <tf2_eigen/tf2_eigen.h>
#include <algorithm>
#include <navigation_msgs/NoPassingZones.h>
#include <navigation_msgs/NoPassingZone.h>
#include <Eigen/Geometry>
#include <perception_msgs/NoPassingZones.h>
THIRD_PARTY_HEADERS_END

typedef std::pair<Eigen::Vector3d, double> NoPassingPoint;
class NoPassingZone {
 public:
  NoPassingZone() = default;
  NoPassingZone(const Eigen::Vector3d &start, const Eigen::Vector3d &end);
  void setStart(const Eigen::Vector3d &start);
  void setEnd(const Eigen::Vector3d &end);
  Eigen::Vector3d getStart() const;
  Eigen::Vector3d getEnd() const;

  bool pointIsInNoPassingZone(const Eigen::Vector3d &point) const;
  navigation_msgs::NoPassingZone toMessage() const;
  static NoPassingZone fromMessage(const navigation_msgs::NoPassingZone &no_passing_zone);
  static NoPassingZone fromMessage(const perception_msgs::NoPassingZone &no_passing_zone);
  static constexpr double safety_margin_after_zone =
      0.5;  // ensures that the corridor is always valid (without it: the
  // corridor can be not driveable tue to no space between obstacle and end of
  // no passing zone)
  double getSafetyMarginAfterZone() const;

 private:
  Eigen::Vector3d start = Eigen::Vector3d::Zero(), end = Eigen::Vector3d::Zero();
};

struct NoPassingZonesParameter {
  NoPassingZonesParameter(ParameterInterface *parameters);
  void updateParameter(ParameterInterface *parameters);
  double merge_distance = 0;
  int number_msg_consecutive = 0;
  double decrease_factor = 0;
  double threshold = 0;

 private:
  static const ParameterString<double> PARAM_DECREASE_FACTOR;
  static const ParameterString<double> PARAM_THRESHOLD;
  static const ParameterString<double> PARAM_MERGE_DISTANCE;
  static const ParameterString<int> PARAM_NUMBER_MSG_CONSECUTIVE;
};

class NoPassingZones {
 public:
  NoPassingZones(const std::shared_ptr<NoPassingZonesParameter> &parameter);
  navigation_msgs::NoPassingZones calcNoPassingZones(
      const Eigen::Affine3d &vehicle_pose,
      const perception_msgs::NoPassingZones &no_passing_msgs,
      const std::vector<NoPassingZone> &no_passing_zones);
  void reset(const int ignore_number_messages_after_reset);

  static std::vector<NoPassingZone> fromMessage(const navigation_msgs::NoPassingZones &no_passing_zones);
  static std::vector<NoPassingZone> fromMessage(const perception_msgs::NoPassingZones &no_passing_zones);
  static void registerParameters(ParameterInterface *parameters);
  bool hasNoPassingZones();



 private:
  navigation_msgs::NoPassingZones toMessage(const std::vector<NoPassingZone> &zones) const;
  std::vector<NoPassingZone> createNoPassingZones();
  void mergeNoPassingZones(std::vector<NoPassingZone> &no_passing_zones,
                           const Eigen::Affine3d &vehicle_pose);

  std::shared_ptr<NoPassingZonesParameter> parameter_;
  std::vector<NoPassingZone> no_passing_zones_perception_;
  std::vector<NoPassingZone> current_zones_;
  int number_msg_consecutive = 0;
  static constexpr double distance_behind = -0.8;
  static constexpr double distance_before = 0.4;
  int resetted = 0;
};



#endif  // NO_PASSING_ZONE_H
