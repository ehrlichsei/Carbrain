#ifndef GATE_UTILS_H
#define GATE_UTILS_H

#include <common/macros.h>
#include <common/polynomial.h>
#include <common/types.h>

THIRD_PARTY_HEADERS_BEGIN
#include <vector>
#include <Eigen/Geometry>

#include "navigation_msgs/Gate.h"
THIRD_PARTY_HEADERS_END

#include "navigation/gate.h"
#include "lane_utils.h"

/**
 * @brief functionality for gates used in path preprocessing
 */
class GateUtils {
 public:
  /**
   * @brief encapsulates intersection between a gate and path of lane marking
   * points
   */
  class Intersection {
   public:
    bool has_intersection = false;
    Eigen::Vector3d point = Eigen::Vector3d::Zero();
    Eigen::Vector3d direction = Eigen::Vector3d::Zero();
  };

  /**
   * @brief intersections of one gate with all lane markings
   */
  struct Intersections {
    Intersection left;
    Intersection middle;
    Intersection right;
  };

  GateUtils() = delete;

  /**
   * \brief calculates intersection point of gate with path
   *
   * \param gate the gate.
   * \param path interpreted as linear spline
   * \return the intersection.
   */
  static Intersection getIntersection(const Gate& gate, const LaneUtils::Lane& path);

  /**
   * \brief returns wheather the gate has an intersection with one of the lane
   *markings
   *
   * \param gate the gate.
   * \param lanes lane markings interpreted as linear spline
   * \return wheather the gate has an intersection with one of the lane markings
   */
  static bool hasIntersection(const Gate& gate, const LaneUtils::Lanes& lanes);

  /**
   * @brief adapt lane boundaries in gate to current lane markings
   *
   * The adaption uses the current boundaries of the gate, the tracked lane
   *information, percepted gates, information about the current lane width,
   *etc.
   *
   * @param gate to adapt lane boundaries
   * @param current_lane_width_left estimated width of left lane, will be
   *updated after call
   * @param current_lane_width_right estimated width of right lane, will be
   *updated after call
   * @param param_lane_width mean of lane width in regulations
   * @param distance_to_vehicle distance of gate center to vehicle
   * @param weight_middle_intersection how the middle lane should influence the
   *other lanes
   * @param weight_own_intersection how the lane should influence the
   *corresponding boundary
   * @param weight_opposite_intersection how the lane should influence the
   *opposite boundary using estimated lane width
   * @param old_lanes estimated lane markings tracked from last update
   * @param lanes currently perceptd lane markings
   * @return whether the gate boundaries has changed
   */
  static bool adaptGateToLane(Gate* gate,
                              Eigen::Vector3d* lane_direction,
                              double* current_lane_width_left,
                              double* current_lane_width_right,
                              const double param_lane_width,
                              const double distance_to_vehicle,
                              const double weight_middle_intersection,
                              const double weight_own_intersection,
                              const double weight_opposite_intersection,
                              const LaneUtils::Lanes& old_lanes,
                              const LaneUtils::Lanes& lanes);

  /*
   * @brief find a possible lane width based on the given intersections
   *
   * Use some assumptions like intersections of the gate with the lane poins and
   * min and max lane width. Tries to estimate the best lane width.
   *
   * @param lane_width_left lane width of left lane
   * @param lane_width_right lane width of right lane
   * @param min_lane_width minimum value for lane width
   * @param max_lane_width maximum value for  lane width
   * @param intersections
   */
  static void calculateLaneWidth(double* lane_width_left,
                                 double* lane_width_right,
                                 const double min_lane_width,
                                 const double max_lane_width,
                                 const GateUtils::Intersections& intersections);

  /**
   * \brief adjust lane point to given point
   *
   * \param gate the gate to adjust the lane center point
   * \param take_new_point whether use new point as new lane center point
   * \param new_point the point to adjust the lane center point to
   * \param distance_to_vehicle the distance between gate and vehicle, used for
   *weighted ajdustment
   */
  template <Gate::Pole pole>
  static void adjustLaneBoundary(Gate* gate,
                                 bool take_new_point,
                                 const Eigen::Vector3d& new_point,
                                 double distance_to_vehicle) {
    if (take_new_point) {
      gate->set<pole>(new_point);
    } else {
      double weight = 1.0 + std::pow(distance_to_vehicle, 2);
      gate->set<pole>((getPole<pole>(*gate) + new_point * weight) / (1.0 + weight));
    }
  }

  /**
   * \brief create new gate from lane polynomial
   *
   * Creates a gate from a polynomial following the lane by evaluating the
   *polynomial at point x.
   *
   * \param lane_polynomial polynomial which follows the lane
   * \param x point to evaluate the lane polynomial at
   * \param path_to_world_transform transformation from polynomial points to
   *world points
   * \param lane_width lane width used for the width of new gate
   * \param id id of gate
   * \return the new gate
   */
  static Gate fromPolynomial(const common::DynamicPolynomial& lane_polynomial,
                             const double x,
                             const Eigen::Affine3d& path_to_world_transform,
                             const double lane_width,
                             const unsigned int id);

  /**
   * \brief create new gate from previous gate
   *
   * Creates a new gate based on a gate behind the new one.
   *
   * \param gate the gate to create a new one from
   * \param direction the direction of the lane at the new gate
   * \param lane_width lane width used for the width of new gate
   * \param id id of gate
   * \return the new gate
   */
  static Gate fromGate(const Gate& gate,
                       const Eigen::Vector3d& direction,
                       const double distance,
                       const double lane_width,
                       const unsigned int id);

  /**
   * @brief estimates current lane direction based on last gates
   *
   * @return estimated lane direction for next gate
   */
  static Eigen::Vector3d calculateNewDirection(const Gate::GateList& gates, const size_t count);

 private:
  static Eigen::Vector3d adaptGateToTrackedLane(
      Gate* gate,
      double* current_lane_width_left,
      double* current_lane_width_right,
      const double min_lane_width,
      const double max_lane_width,
      GateUtils::Intersections* tracked);

  static Eigen::Vector3d adaptGatePerceptedLane(
      Gate* gate,
      double* current_lane_width_left,
      double* current_lane_width_right,
      const double min_lane_width,
      const double max_lane_width,
      const double weight_middle_intersection,
      const double weight_own_intersection,
      const double weight_opposite_intersection,
      const double distance_to_vehicle,
      const Eigen::Vector3d& initial_gate_direction,
      const GateUtils::Intersections& tracked,
      GateUtils::Intersections* percepted);
};

#endif  // GATE_UTILS_H
