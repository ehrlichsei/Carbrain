#include "common/bresenham.h"


void common::bresenhamFunctional(float xStart,
                                 float yStart,
                                 float xEnd,
                                 float yEnd,
                                 const std::function<void(const int, const int, const bool, const bool)>& callback) {
  bool is_start, is_end;
  float x1 = xStart;
  float x2 = xEnd;
  float y1 = yStart;
  float y2 = yEnd;

  // Bresenham's line algorithm
  const bool steep = (std::fabs(y2 - y1) > std::fabs(x2 - x1));
  if (steep) {
    std::swap(x1, y1);
    std::swap(x2, y2);
  }

  if (x1 > x2) {
    std::swap(x1, x2);
    std::swap(y1, y2);
  }

  const float dx = x2 - x1;
  const float dy = std::fabs(y2 - y1);

  float error = dx / 2.0f;
  const int ystep = (y1 < y2) ? 1 : -1;
  int y = static_cast<int>(y1);

  const int maxX = static_cast<int>(x2);

  for (int x = static_cast<int>(x1); x <= maxX; x++) {
    if (steep) {
      is_start = (x == yStart && y == xStart);
      is_end = (x == yEnd && y == xEnd);
      callback(y, x, is_start, is_end);
    } else {
      is_start = (x == xStart && y == yStart);
      is_end = (x == xEnd && y == yEnd);
      callback(x, y, is_start, is_end);
    }

    error -= dy;
    if (error < 0) {
      y += ystep;
      error += dx;
    }
  }
}
