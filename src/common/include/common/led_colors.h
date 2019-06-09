#ifndef LED_COLORS_H
#define LED_COLORS_H

#include "common/macros.h"

THIRD_PARTY_HEADERS_BEGIN
#include "std_msgs/ColorRGBA.h"
THIRD_PARTY_HEADERS_END

namespace led_color {

static std_msgs::ColorRGBA Color(const float r, const float g, const float b, const float a) {
  std_msgs::ColorRGBA color;
  color.r = r;
  color.g = g;
  color.b = b;
  color.a = a;
  return color;
}

// available colors
static std_msgs::ColorRGBA NONE = Color(0.0, 0.0, 0.0, 0.0);
static std_msgs::ColorRGBA RED = Color(1.0, 0.0, 0.0, 1.0);
static std_msgs::ColorRGBA GREEN = Color(0.0, 1.0, 0.0, 1.0);
static std_msgs::ColorRGBA BLUE = Color(0.0, 0.0, 1.0, 1.0);
static std_msgs::ColorRGBA WHITE = Color(1.0, 1.0, 1.0, 1.0);
static std_msgs::ColorRGBA BLACK = Color(0.0, 0.0, 0.0, 1.0);
static std_msgs::ColorRGBA YELLOW = Color(1.0, 1.0, 0.0, 1.0);
static std_msgs::ColorRGBA MAGENTA = Color(1.0, 0.0, 1.0, 1.0);
static std_msgs::ColorRGBA CYAN = Color(0.0, 1.0, 1.0, 1.0);

// special KITcar colors
static std_msgs::ColorRGBA KITCAR_GREEN = Color(0.17, 0.66, 0.29, 1.0);
static std_msgs::ColorRGBA KITCAR_RED =
    Color(242.0 / 255.0, 59.0 / 255.0, 32.0 / 255.0, 1.0);
static std_msgs::ColorRGBA KITCAR_BLUE = Color(36.0 / 255.0, 0.255, 1.0, 1.0);
static std_msgs::ColorRGBA KITCAR_GREY = Color(0.18, 0.23, 0.23, 1.0);

}  // namespace led_color

#endif  // LED_COLORS_H
