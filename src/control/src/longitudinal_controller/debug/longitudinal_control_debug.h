#ifndef LONGITUDINAL_CONTROL_DEBUG_H
#define LONGITUDINAL_CONTROL_DEBUG_H

#include <common/macros.h>

THIRD_PARTY_HEADERS_BEGIN

THIRD_PARTY_HEADERS_END

#include "../longitudinal_control.h"
#include "curvature_controller_debug.h"

class LongitudinalControlDebug : public LongitudinalControl {
 public:
  LongitudinalControlDebug(LongitudinalControl&& longitudinal_control,
                           LongitudinalControllerNode& longitudinal_controller_node);
};

#endif  // LONGITUDINAL_CONTROL_DEBUG_H
