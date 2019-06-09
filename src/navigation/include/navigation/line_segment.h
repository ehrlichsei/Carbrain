#ifndef LINE_SEGMENT_H
#define LINE_SEGMENT_H
#include <common/macros.h>

THIRD_PARTY_HEADERS_BEGIN
#include <Eigen/Geometry>
THIRD_PARTY_HEADERS_END

class LineSegment {
 public:
  LineSegment() = default;
  LineSegment(const Eigen::Vector2d& start_point, const Eigen::Vector2d& end_point);

  const Eigen::Vector2d& getStartPoint() const;

  const Eigen::Vector2d& getEndPoint() const;

  double getLength() const;

  bool intersects(const LineSegment& line_segment,
                  Eigen::Vector2d* intersection_point = std::nullptr_t()) const;

  bool intersectsRay(const LineSegment& line_segment,
                     Eigen::Vector2d* intersection_point = std::nullptr_t()) const;

  double getSignedDistanceToPoint(const Eigen::Vector2d& point) const;
  /**
   * calculates the distance between the given point and the line defined by
   * line segment
   * with no respect to the end points
   */
  double getDistanceToPoint(const Eigen::Vector2d& point) const;

  /**
   * projects point to line segment and returns propotion at segment starting at
   * left point
   */
  double getProjectionOfPoint(const Eigen::Vector2d& point) const;

  bool isPointOnRightSide(const Eigen::Vector2d& point) const;

  bool isPointOnLeftSide(const Eigen::Vector2d& point) const;

  bool containsProjectedPoint(const Eigen::Vector2d& point,
                              Eigen::Vector2d* projected_point = 0) const;

  LineSegment swappedEndpoints() const;

  LineSegment transformed(const Eigen::Affine2d& transformation) const;

  LineSegment& swapEndpoints();

  LineSegment& transform(const Eigen::Affine2d& transformation);

  typedef Eigen::ParametrizedLine<double, 2> Line_2D;
  Line_2D getLine() const;
 private:


  static bool areParallel(const LineSegment::Line_2D& a, const LineSegment::Line_2D& b);


  /*!
   * \brief intersects_impl provides the combined implementation of intersets
   * and intersectsRay as they are basically the same except of the parts which
   * are turns of by the ray-flag.
   * \param line_segment the other line_segment to interset with.
   * \param intersection_point the interseciton point if one exists.
   * \param no_ray wheter to intest as a ray or as a line.
   * \return wheter this and line_segment intersect or not.
   */
  bool intersects_impl(const LineSegment& line_segment,
                       Eigen::Vector2d* intersection_point,
                       const bool no_ray) const;



  Eigen::Vector2d start_point_;
  Eigen::Vector2d end_point_;
};

inline double signedDistance(const Eigen::ParametrizedLine<double, 2>& line, const Eigen::Vector2d& point) {
 const Eigen::Vector2d point_line_frame = point - line.origin();
 const Eigen::Vector2d normal = line.direction().unitOrthogonal();
  return normal.dot(point_line_frame);
}

#endif  // LINE_SEGMENT_H
