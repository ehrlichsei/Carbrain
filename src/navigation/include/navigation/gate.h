#ifndef GATE_H
#define GATE_H
#include <common/macros.h>

THIRD_PARTY_HEADERS_BEGIN
#include <vector>
#include <Eigen/Geometry>
#include <boost/range/algorithm/transform.hpp>

#include "navigation_msgs/Gate.h"
THIRD_PARTY_HEADERS_END

#include "common/types.h"

#include "line_segment.h"

/**
 * \brief the Gate class represents an infinitesimal small segment of the road
 *
 * The gate representation is a direction vector which is perpendicular to the
 *road. It is represented by the left and right lane boundary and some points
 *between
 *them.
 *
 * The most important points between the lane boundaries are the left and right
 *pole. These poles describes the actual gate, whitch should be passed by the
 *vehicle. Acually only the center of the vehicle (origin of vehicle coordinate
 *system) should pass the gate inside the poles which makes it easier to
 *consider the dimensions of the car by shrinking the gate accordingly.
 */
class Gate {
 public:
  enum Pole : int { LEFT, RIGHT, CENTER, LEFT_LANE, MIDDLE_LANE, RIGHT_LANE };

  constexpr static Pole other(const Pole& p) {
    return (p == LEFT) ? RIGHT : LEFT;
  }

  typedef common::EigenAlignedVector<Gate> GateList;

  /**
   * \brief creates gate with gate poles equals lane boundary
   *
   * \param left_lane_boundary defines the lane boundary on the outer left side
   * \param right_lane_boundary defines the lane boundary on the outer right
   *side
   */
  Gate(int id, const Eigen::Vector3d& left_lane_boundary, const Eigen::Vector3d& right_lane_boundary);

  /**
   * \brief creates new gate with defined pole inside the gate
   *
   * \param left_pole parametric representation of left pole of the gate
   * \param right_pole parametric representation of right pole of the gate
   * \param left_lane_boundary defines the lane boundary on the outer left side
   * \param right_lane_boundary defines the lane boundary on the outer right
   *side
   */
  Gate(int id,
       double left_pole,
       double right_pole,
       const Eigen::Vector3d& left_lane_boundary,
       const Eigen::Vector3d& right_lane_boundary,
       double lane_center_point);

  int getId() const;
  void setId(const int new_id);

  /**
   * \brief returns the left pole
   *
   * \return the position of the left pole in world coordinates
   */
  Eigen::Vector3d getLeftPole() const;

  /**
   * \brief returns the right pole
   *
   * \return the position of the right pole in world coordinates
   */
  Eigen::Vector3d getRightPole() const;

  /**
   * \brief returns the left lane boundary on the outer left side of the gate
   *
   * \return the position of the left lane boundary in world coordinates
   */
  const Eigen::Vector3d& getLeftLaneBoundary() const {
    return left_lane_boundary_;
  }

  /**
    * \brief returns the right lane boundary on the outer right side of the gate
    *
    * \return the position of the right lane boundary in world coordinates
    */
  const Eigen::Vector3d& getRightLaneBoundary() const {
    return right_lane_boundary_;
  }

  /**
   * \brief returns the prefered path point
   *
   * returns the point on the gate which is the preferably point to drive
   *through by the car when not look at the other
   * gates
   *
   * \return the position of the prefered path point in world coordinates
   */
  Eigen::Vector3d getPreferedPathPoint() const;

  /**
   * \brief returns the prefered path point parameter
   *
   * returns the parameter of the point on the gate which is the preferably
   *point to drive
   * through by the car when not look at the other
   * gates
   *
   * \return the position of the prefered path point in world coordinates
   */
  double getPreferedPathPointParameter() const;

  /**
   * weight between 0 and 1
   * 0: prefered path point is not used as optimization parameter
   * 1: prefered path point is used as optimization parameter
   */
  double getPreferedPathWeight() const;

  /**
   * \brief gets the center of the gate
   *
   * gets the center point which lays between the left and right pole of the
   *gate
   *
   * \return the position of the center point in world coordinates
   */
  Eigen::Vector3d getCenter() const;

  /**
   * \brief gets the center of the lane described by the gate
   *
   * gets the center point of the lane which idealy lays on the middle lane
   *marking
   *
   * \return the position of the center point of the lane in world coordinates
   */
  Eigen::Vector3d getLaneCenter() const;

  /**
   * \brief the center of the gate projected to the ground
   *
   * \return the center point projected to the ground
   */
  Eigen::Vector2d getCenterProjection() const {
    return getCenter().topRows<2>();
  }

  /**
   * \brief the left pole of the gate projected to the ground
   *
   * \return the left pole point projected to the ground
   */
  Eigen::Vector2d getLeftPoleProjection() const {
    return getLeftPole().topRows<2>();
  }

  /**
   * \brief the right pole of the gate projected to the ground
   *
   * \return the right pole point projected to the ground
   */
  Eigen::Vector2d getRightPoleProjection() const {
    return getRightPole().topRows<2>();
  }

  /**
   * \brief the center of the lane projected to the ground
   *
   * \return the center point of the lane projected to the ground
   */
  Eigen::Vector2d getLaneCenterProjection() const {
    return getLaneCenter().topRows<2>();
  }

  /**
   * \brief gets the left lane boundary projected on the ground
   *
   * \return the point of the left lane boundary projected on the ground
   */
  Eigen::Vector2d getLeftLaneBoundaryProjection() const {
    return left_lane_boundary_.topRows<2>();
  }

  /**
   * \brief gets the right lane boundary projected on the ground
   *
   * \return the point of the right lane boundary projected on the ground
   */
  Eigen::Vector2d getRightLaneBoundaryProjection() const {
    return right_lane_boundary_.topRows<2>();
  }

  /**
   * \brief gets the width of the gate
   *
   * gets the witch of the gate which is equal to the distance between left and
   *right pole
   *
   * \returns the width in meters
   */
  double getWidth() const;

  bool isDegenerated() const;

  /**
   * \brief whether the point is in behind the gate
   *
   * \param point the given point
   * \return true if the point will be passed before the gate
   */
  bool isPointBehind(Eigen::Vector3d point) const;

  /**
   * \brief whether the point is in in front of the gate
   *
   * \param point the given point
   * \return true if the point will be passed after the gate
   */
  bool isPointInFront(Eigen::Vector3d point) const;

  /**
   * \brief whether the given point is inside the left and right pole
   *
   * \param param parametric representation of an point
   */
  bool contains(const double param) const;

  /*!
   * \brief Calculates a transformation from a virtual gate frame
   * The transformation is calculated so that the point (0, 0, 0) would be
   * transformed
   * to the location of the left_pole.
   * \return the transformation from the gate frame.
   */
  Eigen::Affine3d getTransformationFromGateFrame() const;

  /*!
   * \brief Calculates a transformation to a virtual gate frame
   * The transformation is calculated so that the left_pole would be transformed
   * to (0, 0, 0) and the right_pole lies on the positiv x-axis.
   * \return the transformation to the gate frame
   */
  Eigen::Affine3d getTransformationToGateFrame() const;

  /*!
   * \brief Calculates the rotation of the gate relative to the X-axis
   * \return A quaternion representing the rotation
   */
  Eigen::Quaterniond getRotation() const;

  template <Gate::Pole pole>
  inline void set(const Eigen::Vector3d& p);

  void setLeftPole(const Eigen::Vector3d& left_pole);

  void setRightPole(const Eigen::Vector3d& right_pole);

  void setLeftLaneBoundary(const Eigen::Vector3d& left_lane_boundary);

  void setRightLaneBoundary(const Eigen::Vector3d& right_lane_boundary);

  void setLaneCenter(const Eigen::Vector3d& lane_center_point);

  void setPreferedPathPoint(const Eigen::Vector3d& prefered_path_point);

  void setPreferedPathPoint(double prefered_path_point);

  /**
   * weight between 0 and 1
   * 0: prefered path point is not used as optimization parameter
   * 1: prefered path point is used as optimization parameter
   */
  void setPreferedPathWeight(double prefered_path_weight);

  /**
   * \brief collapses the gate poles the gate center
   */
  void collapseToCenter();

  void shrinkBy(double shrink_distance);

  void shrinkFromLeftBy(double shrink_distance);

  void shrinkFromRightBy(double shrink_distance);

  /**
   * \brief transforms the gate with the given transformation
   */
  void transform(const Eigen::Affine3d& transformation);

  /**
   * \brief creates a gate from a gate message
   *
   * converts a gate message to a gate
   *
   * \param gate_msg the ros message of the gate
   * \return the created gate
   */
  static Gate fromMessage(const navigation_msgs::Gate& gate_msg);

  template <typename C>
  static GateList fromMessages(const C& gate_msgs) {
    GateList gate;
    gate.reserve(gate_msgs.size());
    boost::transform(gate_msgs, std::back_inserter(gate), &Gate::fromMessage);
    return gate;
  }
  /**
   * \brief converts the gate to the corresponding ros message
   *
   * \return the converted message
   */
  navigation_msgs::Gate toMessage() const;

  friend bool operator<(const Gate& first_gate, const Gate& second_gate);
  friend bool operator>(const Gate& first_gate, const Gate& second_gate);

  /**
   * \brief gets line segment from gate
   */
  LineSegment getLineSegment() const;

  /**
   * \brief gets line segment from lane gate
   */
  LineSegment getLaneLineSegment() const;

  /**
   * \brief converts from parametric representation to point
   *
   * converts from parametric representation to point in wold coordinates
   */
  Eigen::Vector3d toPoint(double param) const;

  /**
   * \brief gets the not normalized direction of the gate
   *
   * gets the not normalized direction of the gate, which is specified as the vector from the
   *left to the right lane boundary
   *
   * \return the not normalized direction vector in world coordinates
   */
  Eigen::Vector3d getVectorLeftToRight() const;

  template <Pole pole>
  inline double getParam() const;
  template <Pole pole>
  inline void setParam(const double new_param);

  /**
   * converts from point to parametric representation
   */
  double toParam(const Eigen::Vector3d& point) const;

 private:
  Gate();  

  Eigen::Vector3d left_lane_boundary_;
  Eigen::Vector3d right_lane_boundary_;
  unsigned int id_;
  double lane_center_point_ = 0.5;
  double left_pole_ = 0.0;
  double right_pole_ = 1.0;
  double prefered_path_point_ = 3.0 / 4.0;

  /**
   * weight between 0 and 1
   * 0: prefered path point is not used as optimization parameter
   * 1: prefered path point is used as optimization parameter
   */
  double prefered_path_weight_ = 1.0;
};

bool operator<(const Gate& first_gate, const Gate& second_gate);
bool operator>(const Gate& first_gate, const Gate& second_gate);

template <>
inline double Gate::getParam<Gate::LEFT>() const {
  return left_pole_;
}
template <>
inline double Gate::getParam<Gate::RIGHT>() const {
  return right_pole_;
}
template <>
inline double Gate::getParam<Gate::CENTER>() const {
  return (left_pole_ + right_pole_) / 2.0;
}
template <>
inline double Gate::getParam<Gate::MIDDLE_LANE>() const {
  return lane_center_point_;
}

template <>
inline void Gate::setParam<Gate::LEFT>(const double new_param) {
  left_pole_ = new_param;
}
template <>
inline void Gate::setParam<Gate::RIGHT>(const double new_param) {
  right_pole_ = new_param;
}

inline double Gate::getPreferedPathPointParameter() const {
  return prefered_path_point_;
}

template <Gate::Pole pole>
inline Eigen::Vector3d getPole(const Gate& gate);

template <>
inline Eigen::Vector3d getPole<Gate::LEFT>(const Gate& gate) {
  return gate.getLeftPole();
}
template <>
inline Eigen::Vector3d getPole<Gate::RIGHT>(const Gate& gate) {
  return gate.getRightPole();
}
template <>
inline Eigen::Vector3d getPole<Gate::LEFT_LANE>(const Gate& gate) {
  return gate.getLeftLaneBoundary();
}
template <>
inline Eigen::Vector3d getPole<Gate::MIDDLE_LANE>(const Gate& gate) {
  return gate.getLaneCenter();
}
template <>
inline Eigen::Vector3d getPole<Gate::RIGHT_LANE>(const Gate& gate) {
  return gate.getRightLaneBoundary();
}

template <Gate::Pole pole>
inline Eigen::Vector2d getPoleProjection(const Gate& gate);

template <>
inline Eigen::Vector2d getPoleProjection<Gate::LEFT>(const Gate& gate) {
  return gate.getLeftPoleProjection();
}
template <>
inline Eigen::Vector2d getPoleProjection<Gate::RIGHT>(const Gate& gate) {
  return gate.getRightPoleProjection();
}

template <>
inline void Gate::set<Gate::LEFT_LANE>(const Eigen::Vector3d& p) {
  setLeftLaneBoundary(p);
}
template <>
inline void Gate::set<Gate::MIDDLE_LANE>(const Eigen::Vector3d& p) {
  setLaneCenter(p);
}
template <>
inline void Gate::set<Gate::RIGHT_LANE>(const Eigen::Vector3d& p) {
  setRightLaneBoundary(p);
}

template <Gate::Pole pole>
inline bool isPointOnSide(const LineSegment&, const Eigen::Vector2d&);

template <>
inline bool isPointOnSide<Gate::LEFT>(const LineSegment& line, const Eigen::Vector2d& v) {
  return line.isPointOnLeftSide(v);
}
template <>
inline bool isPointOnSide<Gate::RIGHT>(const LineSegment& line, const Eigen::Vector2d& v) {
  return line.isPointOnRightSide(v);
}


#endif  // GATE_H
