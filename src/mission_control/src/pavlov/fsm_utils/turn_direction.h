#ifndef TURN_DIRECTION_H
#define TURN_DIRECTION_H

#include "common/macros.h"

THIRD_PARTY_HEADERS_BEGIN
#include <iostream>
THIRD_PARTY_HEADERS_END

enum class TurnDirection {
  LEFT, RIGHT
};

inline std::ostream& operator<<(std::ostream& stream, const TurnDirection& d) {
  switch (d) {
    case TurnDirection::LEFT:
      stream << "Left";
      break;
    case TurnDirection::RIGHT:
      stream << "Right";
      break;
    default:
      stream << "Unknown Direction (" << static_cast<int>(d) << ")";
  }

  return stream;
}

#endif
