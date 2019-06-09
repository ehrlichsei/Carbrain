#include "longitudinal_controller_node_debug.h"

THIRD_PARTY_HEADERS_BEGIN
#include <memory>
THIRD_PARTY_HEADERS_END

#include "longitudinal_control_debug.h"

LongitudinalControllerNodeDebug::LongitudinalControllerNodeDebug(ros::NodeHandle& node_handle)
    : LongitudinalControllerNode(node_handle) {
  longitudinal_control_ = std::make_unique<LongitudinalControlDebug>(
      std::move(*longitudinal_control_), *this);
}

void LongitudinalControllerNodeDebug::startModule() {
  LongitudinalControllerNode::startModule();
}

void LongitudinalControllerNodeDebug::stopModule() {
  LongitudinalControllerNode::stopModule();
}
