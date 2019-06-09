#include "navigation/line_segment.h"

#include <cmath>

LineSegment::LineSegment(const Eigen::Vector2d& start_point, const Eigen::Vector2d& end_point)
    : start_point_(start_point), end_point_(end_point) {}

const Eigen::Vector2d& LineSegment::getStartPoint() const {
  return start_point_;
}

const Eigen::Vector2d& LineSegment::getEndPoint() const { return end_point_; }

double LineSegment::getLength() const {
  return (end_point_ - start_point_).norm();
}

bool LineSegment::areParallel(const LineSegment::Line_2D& a, const LineSegment::Line_2D& b) {
  const double epsilon = 1e-16;
  return std::abs(a.direction().dot(b.direction())) > 1 - epsilon;
}

bool LineSegment::intersects(const LineSegment& line_segment,
                             Eigen::Vector2d* intersection_point) const {
  return intersects_impl(line_segment, intersection_point, true);
}

bool LineSegment::intersectsRay(const LineSegment& line_segment,
                                Eigen::Vector2d* intersection_point) const {
  return intersects_impl(line_segment, intersection_point, false);
}

double LineSegment::getDistanceToPoint(const Eigen::Vector2d& point) const {
  return std::fabs(getSignedDistanceToPoint(point));
}

double LineSegment::getProjectionOfPoint(const Eigen::Vector2d& point) const {
  const LineSegment::Line_2D line = getLine();

  const Eigen::Vector2d point_line_frame = point - line.origin();
  const double parameter = line.direction().dot(point_line_frame);

  return parameter / getLength();
}

bool LineSegment::isPointOnRightSide(const Eigen::Vector2d& point) const {
  return getSignedDistanceToPoint(point) < 0;
}

bool LineSegment::isPointOnLeftSide(const Eigen::Vector2d& point) const {
  return getSignedDistanceToPoint(point) > 0;
}

bool LineSegment::containsProjectedPoint(const Eigen::Vector2d& point,
                                         Eigen::Vector2d* projected_point) const {
  const double parameter = getProjectionOfPoint(point);

  if (parameter < 0 || parameter > 1.0) {
    return false;
  }
  if (projected_point) {
    *projected_point = getLine().projection(point);
  }
  return true;
}

LineSegment LineSegment::swappedEndpoints() const {
  LineSegment copy = *this;
  copy.swapEndpoints();
  return copy;
}

LineSegment LineSegment::transformed(const Eigen::Affine2d& transformation) const {
  LineSegment copy = *this;
  copy.transform(transformation);
  return copy;
}

LineSegment& LineSegment::swapEndpoints() {
  const Eigen::Vector2d original_start_point = start_point_;
  start_point_ = end_point_;
  end_point_ = original_start_point;
  return *this;
}

LineSegment& LineSegment::transform(const Eigen::Affine2d& transformation) {
  start_point_ = transformation * start_point_;
  end_point_ = transformation * end_point_;
  return *this;
}


double LineSegment::getSignedDistanceToPoint(const Eigen::Vector2d& point) const {
  return signedDistance(getLine(), point);
}

bool LineSegment::intersects_impl(const LineSegment& line_segment,
                                  Eigen::Vector2d* intersection_point,
                                  const bool no_ray) const {
  const LineSegment::Line_2D this_line = getLine();
  const LineSegment::Line_2D other_line = line_segment.getLine();
  if (areParallel(this_line, other_line)) {
    return false;
  }
  const double this_parameter =
      this_line.intersectionParameter(Eigen::Hyperplane<double, 2>(other_line));
  if (!std::isnormal(this_parameter) || this_parameter < 0 ||
      (no_ray && this_parameter > getLength())) {
    return false;
  }
  const double other_parameter =
      other_line.intersectionParameter(Eigen::Hyperplane<double, 2>(this_line));
  if (!std::isnormal(other_parameter) || other_parameter < 0 ||
      other_parameter > line_segment.getLength()) {
    return false;
  }
  if (intersection_point) {
    *intersection_point =
        this_line.intersectionPoint(Eigen::Hyperplane<double, 2>(other_line));
  }
  return true;
}

LineSegment::Line_2D LineSegment::getLine() const {
  return LineSegment::Line_2D::Through(start_point_, end_point_);
}
