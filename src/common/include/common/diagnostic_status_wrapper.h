#ifndef COMMON_DIAGNOSTIC_STATUS_WRAPPER_H
#define COMMON_DIAGNOSTIC_STATUS_WRAPPER_H
#include <common/macros.h>

THIRD_PARTY_HEADERS_BEGIN
#include <diagnostic_updater/diagnostic_updater.h>
THIRD_PARTY_HEADERS_END

namespace common {

class DiagnosticStatusWrapper : public diagnostic_updater::DiagnosticStatusWrapper {
 public:
  inline void addVoice() { add("voice", "true"); }

  inline void addLed(float red, float green, float blue) {
    add("led_r", std::to_string(red));
    add("led_g", std::to_string(green));
    add("led_b", std::to_string(blue));
  }
};
} // namespace common

#endif  // COMMON_DIAGNOSTIC_STATUS_WRAPPER_H
