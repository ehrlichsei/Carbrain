#ifndef BRESENHAM_H
#define BRESENHAM_H

#include <cmath>
#include <algorithm>
#include <functional>

namespace common {

/*!
 * \brief bresenham algorithm for line / stepping without anti-aliasing
 * http://rosettacode.org/wiki/Bitmap/Bresenham's_line_algorithm#C.2B.2B
 * \param xStart start point x coordinate
 * \param yStart start point y coordinate
 * \param xEnd end point x coordinate
 * \param yEnd end point y coordinate
 * \param callback  callback function
 */
void bresenhamFunctional(float xStart,
                         float yStart,
                         float xEnd,
                         float yEnd,
                         const std::function<void(const int, const int, const bool, const bool)>& callback);

/*!
 * \brief bresenham algorithm for line / stepping without anti-aliasing
 * http://rosettacode.org/wiki/Bitmap/Bresenham's_line_algorithm#C.2B.2B
 * \param x1  start point x coordinate
 * \param y1  start point y coordinate
 * \param x2  end point x coordinate
 * \param y2  end point y coordinate
 * \param fb  callback function type (eg. &MyClass::cb)
 * \param obj object to call the callback on (eg. this)
 */
template <class T>
void bresenham(float x1,
               float y1,
               float x2,
               float y2,
               void (T::*fb)(const int x, const int y, const bool is_start, const bool is_end),
               T* obj) {

  std::function<void(const int, const int, const bool, const bool)> callback(std::bind(
      fb, obj, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3, std::placeholders::_4));

  bresenhamFunctional(x1, y1, x2, y2, callback);
}

}  // common

#endif  // BRESENHAM_H
