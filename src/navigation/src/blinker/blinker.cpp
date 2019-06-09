#include "blinker.h"
#include <common/macros.h>
#include <navigation/driving_corridor.h>
#include <boost/range/algorithm_ext/erase.hpp>

const ParameterString<double> Blinker::VEHICLE_WIDTH(
    "/car_specs/vehicle_width");
const ParameterString<double> Blinker::VEHICLE_WIDTH_SUBTRACTIION(
    "vehicle_width_subtraction");

Blinker::LanePosition getGatePosition(const Gate &gate) {
  const double delta = 0.1;
  if (gate.getParam<Gate::LEFT>() < delta && (1.0 - gate.getParam<Gate::RIGHT>()) < delta) {
    return Blinker::LanePosition::NONE;
  }
  return gate.getParam<Gate::LEFT>() < (1.0 - gate.getParam<Gate::RIGHT>())
             ? Blinker::LanePosition::LEFT
             : Blinker::LanePosition::RIGHT;
}

Blinker::LanePosition getCorridorPositionAtVehicle(const DrivingCorridor &safe_corridor,
                                                   const DrivingCorridor::ConstIterator &first_gate_ahead_iterator) {
  const size_t gates_before =
      std::distance(safe_corridor.begin(), first_gate_ahead_iterator);
  const size_t gates_after =
      std::distance(first_gate_ahead_iterator, safe_corridor.end());
  const size_t gate_area = 5;
  if (gates_before <= gate_area || gates_after <= gate_area) {
    return getGatePosition(*first_gate_ahead_iterator);
  }
  for (size_t i = 0; i < gate_area; ++i) {
    if (getGatePosition(*(first_gate_ahead_iterator + i)) != Blinker::LanePosition::NONE) {
      return getGatePosition(*(first_gate_ahead_iterator + i));
    }
    if (getGatePosition(*(first_gate_ahead_iterator - i)) != Blinker::LanePosition::NONE) {
      return getGatePosition(*(first_gate_ahead_iterator - i));
    }
  }
  return Blinker::LanePosition::NONE;
}

BlinkerDecision Blinker::decideBlink(DrivingCorridor &safe_corridor,
                                     const Eigen::Affine3d &vehicle_pose) {
  safe_corridor.erase(std::remove_if(safe_corridor.begin(),
                                     safe_corridor.end(),
                                     [](const Gate &gate) {
                                       if (gate.getLaneLineSegment().getLength() < 0.4) {
                                         return true;
                                       }
                                       return gate.getParam<Gate::MIDDLE_LANE>() < 0.2 ||
                                              gate.getParam<Gate::MIDDLE_LANE>() > 0.8;
                                     }),
                      safe_corridor.end());
  const auto gate_is_ahead = [&vehicle_pose](const Gate &gate) {
    const Eigen::Vector3d gate_in_vehicle_pose =
        vehicle_pose.inverse() * gate.getCenter();
    return gate_in_vehicle_pose.x() > 0.0;
  };
  const auto first_gate_ahead =
      std::find_if(safe_corridor.begin(), safe_corridor.end(), gate_is_ahead);
  if (first_gate_ahead == safe_corridor.end()) {
    return BlinkerDecision::NO_DECISION;
  }
  LanePosition vehicle_position =
      getCorridorPositionAtVehicle(safe_corridor, first_gate_ahead);
  if (vehicle_position == LanePosition::NONE &&
      last_vehicle_lane_position != LanePosition::NONE) {
    // this case occurs e.g. if an obstacle is detected near the car
    std::swap(vehicle_position, last_vehicle_lane_position);
  } else {
    last_vehicle_lane_position = vehicle_position;
  }
  std::vector<std::tuple<LanePosition, int> > gate_positions_ahead;
  gate_positions_ahead.push_back(
      std::tuple<LanePosition, int>(getGatePosition(*first_gate_ahead), 1));
  std::for_each(first_gate_ahead + 1, safe_corridor.end(), [&gate_positions_ahead](const Gate &gate) {
    LanePosition next_position = getGatePosition(gate);
    if (std::get<0>(gate_positions_ahead.back()) != next_position) {
      gate_positions_ahead.push_back(std::tuple<LanePosition, int>(next_position, 1));
    } else {
      std::get<1>(gate_positions_ahead.back())++;
    }
  });

  boost::remove_erase_if(gate_positions_ahead,
                         [](const std::tuple<LanePosition, int> &gate_position) {
                           return std::get<1>(gate_position) < DECISION_THRESHOLD;
                         });

  if (gate_positions_ahead.size() < 2) {
    return BlinkerDecision::ABORT_BLINKING;
  }
  if (vehicle_position != LanePosition::NONE) {
    const auto next_not_none =
        std::find_if(gate_positions_ahead.begin() + 1,
                     gate_positions_ahead.end(),
                     [](const std::tuple<LanePosition, int> next) {
                       return std::get<0>(next) != LanePosition::NONE;
                     });
    if (next_not_none == gate_positions_ahead.end()) {
      return vehicle_position == LanePosition::LEFT ? BlinkerDecision::BLINK_RIGHT
                                                    : BlinkerDecision::BLINK_LEFT;
    }
    if (std::get<0>(*next_not_none) == vehicle_position) {
      return BlinkerDecision::NO_DECISION;
    }
    return vehicle_position == LanePosition::LEFT ? BlinkerDecision::BLINK_RIGHT
                                                  : BlinkerDecision::BLINK_LEFT;
  }
  assert(vehicle_position == LanePosition::NONE);
  const auto next_not_none =
      std::find_if(gate_positions_ahead.begin() + 1,
                   gate_positions_ahead.end(),
                   [](const std::tuple<LanePosition, int> next) {
                     return std::get<0>(next) != LanePosition::NONE;
                   });
  /*
   * assert valid, because gate_positions_ahead.size >= 2. Therefore, another value than LanePosition::NONE can be found by find_if.
   */
  assert(next_not_none != gate_positions_ahead.end());
  return std::get<0>(*next_not_none) == LanePosition::LEFT
             ? BlinkerDecision::BLINK_LEFT
             : BlinkerDecision::BLINK_RIGHT;
}
