#include "navigation/triangle.h"

Triangle::Triangle(const Triangle::CornerPoints& points) : points_(points) {}

Triangle::Triangle(const Eigen::Vector2d &first_point,
                   const Eigen::Vector2d &second_point,
                   const Eigen::Vector2d &third_point)
    : points_{{first_point, second_point, third_point}} {}

Triangle::CornerPoints Triangle::getCornerPoints() const { return points_; }

bool Triangle::isPointInside(const Eigen::Vector2d &point) const {
  for (size_t i = 0; i < TRI; i++) {
    if (!(getLineSegment(i).isPointOnLeftSide(point))) {
      return false;
    }
  }
  return true;
}

bool Triangle::isPointOutside(const Eigen::Vector2d &point) const {
  for (size_t i = 0; i < TRI; i++) {
    if (getLineSegment(i).isPointOnRightSide(point)) {
      return true;
    }
  }
  return false;
}

LineSegment Triangle::getLineSegment(const size_t index) const {
  const Eigen::Vector2d start_point = points_[index % 3];
  const Eigen::Vector2d end_point = points_[(index + 1) % 3];
  return LineSegment(start_point, end_point);
}
