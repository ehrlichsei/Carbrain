#ifndef MESSAGE_CONVERTER_H
#define MESSAGE_CONVERTER_H

#include "common/macros.h"

THIRD_PARTY_HEADERS_BEGIN
#include <tf2_eigen/tf2_eigen.h>
#include <vector>
THIRD_PARTY_HEADERS_END

namespace look_at {

template <typename Message, typename RoadObject, typename SubMessageVector, typename Vector>
class MessageConverter {
 public:
  SubMessageVector toMsg(const Vector &vector) const {
    SubMessageVector submessage_vector;
    submessage_vector.reserve(vector.size());
    for (const auto &object : vector) {
      submessage_vector.push_back(tf2::toMsg(object));
    }
    return submessage_vector;
  }

  //  template <class Message, class RoadObject>
  Message createMsg(const RoadObject &road_object, const std::size_t id) const {
    Message message;
    message.header.frame_id = "world";
    message.header.stamp = road_object.timestamp;
    message.certainty = static_cast<float>(road_object.score);
    message.id = id;
    message.base_hull_polygon = toMsg(road_object.base_hull_polygon_in_world);
    return message;
  }
};
}  // namespace look_at

#endif  // MESSAGE_CONVERTER_H
