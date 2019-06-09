#ifndef TRIANGLE_H
#define TRIANGLE_H

#include "line_segment.h"
#include <Eigen/Geometry>
#include <array>

class Triangle {
 public:

  constexpr const static size_t TRI = 3;
  typedef std::array<Eigen::Vector2d, TRI>  CornerPoints;

  Triangle(const CornerPoints& points);

  Triangle(const Eigen::Vector2d& first_point, const Eigen::Vector2d& second_point, const Eigen::Vector2d& third_point);

  CornerPoints getCornerPoints() const;

  bool isPointInside(const Eigen::Vector2d& point) const;

  bool isPointOutside(const Eigen::Vector2d& point) const;

 private:

  LineSegment getLineSegment(const size_t index) const;
  CornerPoints points_;
};

#endif  // TRIANGLE_H
