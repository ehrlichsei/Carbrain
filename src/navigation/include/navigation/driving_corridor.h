#ifndef DRIVING_CORRIDOR_H
#define DRIVING_CORRIDOR_H
#include <common/macros.h>

THIRD_PARTY_HEADERS_BEGIN
#include <vector>
#include "navigation_msgs/DrivingCorridor.h"

#include "Eigen/Geometry"
THIRD_PARTY_HEADERS_END

#include "gate.h"


class DrivingCorridor {
 public:
  using Iterator = Gate::GateList::iterator;
  using ReverseIterator = Gate::GateList::reverse_iterator;
  using ConstIterator = Gate::GateList::const_iterator;
  using ConstReverseIterator = Gate::GateList::const_reverse_iterator;

  /*!
   * creates an empty corridor
   */
  DrivingCorridor() = default;

  /*!
   * \brief creates corridor from a list of gates
   *
   * \param gates the gates to create the corridor from
   */
  DrivingCorridor(Gate::GateList gates);


  /*!
   * \brief access gate
   *
   * returns the gate at position index in the corridor
   *
   * \param index position of an gate in the corridor
   * \return the gate at the specified position
   */
  Gate& at(size_t index);

  /*!
   * \brief access gate
   *
   * returns the gate at position index in the corridor
   *
   * \param index position of an gate in the corridor
   * \return the gate at the specified position
   */
  const Gate& at(size_t index) const;

  /*!
   * \brief number of gates in the corridor
   */
  size_t size() const;

  /*!
   * \brief empty if corridor is empty
   */
  bool empty() const;

  /*!
   * \brief clears gates in the corridor
   */
  void clear();

  /*!
   * \brief returns iterator to the first gate in the corridor
   */
  Iterator begin();

  /*!
   * \brief returns iterator to the first gate in the corridor
   */
  ConstIterator begin() const;

  /*!
   * \brief returns iterator referring to the past-the-end element in the list
   * container
   */
  Iterator end();

  /*!
   * \brief returns iterator referring to the past-the-end element in the list
   * container
   */
  ConstIterator end() const;


  ReverseIterator rbegin();

  ConstReverseIterator rbegin() const;

  ReverseIterator rend();

  ConstReverseIterator rend() const;

  /*!
   * \brief returns reference to the last gate in the corridor
   */
  Gate& back();

  /*!
   * \brief returns reference to the last gate in the corridor
   */
  const Gate& back() const;

  /*!
   * \brief adds gate at the end of the corridor
   */
  void push_back(const Gate& gate);

  /*!
   * \brief removes the last element in the vector, effectively reducing the
   * container size by one
   */
  void pop_back();

  void reserve(size_t new_cap);

  void erase(const ConstIterator& first, const ConstIterator& last);

  /*!
   * \brief Returns true if the point is contained in the corridor
   * May return conservatively true even if the corridor does not contain the
   * given point.
   * \param point the point to check.
   * \return whether the point is aproximate contained.
   */
  bool isPointApproximateContained(const Eigen::Vector3d& point) const;

  /*!
   * \brief Returns true if the point is contained in the corridor
   * Returns true even if the point lies on the border of the corridor.
   * If the corridor is composed out of less then two gates
   * this method returns always false.
   * \param point the point ot check.
   * \return whether the point is contained.
   */
  bool isPointContained(const Eigen::Vector3d& point) const;

  /*!
   * \brief Returns true if the point lies more on the right lane
   * rather than the left lane.
   * Does not check whether the point is contained in the corridor.
   * \param point the point to check.
   * \return whether the point is on the right lane.
   */
  bool isPointOnRightLane(const Eigen::Vector3d& point) const;
  /*!
   * \brief Sorts the gates by the x-values of their center
   * \param transform Transformation to the path coordinate system
   */
  void sort(const Eigen::Affine3d& transform);

  /*!
   * \brief Merges close gates into one gate
   * Expects the corridor to be sorted.
   * \param min_distance Minimal allowed distance between to close gate centers
   * \return A new DrivingCorridor with less or the same number of gates as this
   * corridor
   */
  DrivingCorridor simplified(const double min_distance) const;

  /*!
   * \brief creates driving corridor from ros driving corridor message
   *
   * \param driving_corridor_msg the message
   * \return the created driving corridor
   */
  static DrivingCorridor fromMessage(const navigation_msgs::DrivingCorridor::ConstPtr& driving_corridor_msg);

  /*!
   * \brief converts this driving corridor to ros message
   *
   * \return the converted message
   */
  navigation_msgs::DrivingCorridor toMessage() const;

 private:
  Gate::GateList gates_;
};

#endif  // DRIVING_CORRIDOR_H
