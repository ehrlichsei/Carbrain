#ifndef PERCEPTION_LANE_DETECTION_LANE_MODEL_H
#define PERCEPTION_LANE_DETECTION_LANE_MODEL_H

#include "common/polynomial.h"
#include <array>
#include <boost/optional/optional.hpp>
#include <boost/range/algorithm/count.hpp>
#include <boost/algorithm/cxx11/all_of.hpp>

/*!
 * \brief LineData_t
 * \todo generalize to support multiple line models
 */
typedef typename std::array<boost::optional<common::DynamicPolynomial>, LINESPEC_N> LaneModel;

inline int number(const LaneModel& lane_model) {
  return lane_model.size() - boost::count(lane_model, boost::none);
}

inline bool empty(const LaneModel& lane_model) {
  return boost::algorithm::all_of_equal(lane_model, boost::none);
}

typedef typename std::array<int, LINESPEC_N> LineDegree;

#endif  // PERCEPTION_LANE_DETECTION_LANE_MODEL_H
