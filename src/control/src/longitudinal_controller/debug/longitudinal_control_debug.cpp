#include "longitudinal_control_debug.h"

THIRD_PARTY_HEADERS_BEGIN
#include <memory>
THIRD_PARTY_HEADERS_END

#include "longitudinal_controller_node_debug.h"

LongitudinalControlDebug::LongitudinalControlDebug(LongitudinalControl&& longitudinal_control,
                                                   LongitudinalControllerNode& longitudinal_controller_node)
    : LongitudinalControl(std::move(longitudinal_control)) {
  curvature_controller = std::make_unique<CurvatureControllerDebug>(
      std::move(*curvature_controller), longitudinal_controller_node);
}
