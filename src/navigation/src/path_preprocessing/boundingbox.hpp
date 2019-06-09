#ifndef BOUNDINGBOX_HPP
#define BOUNDINGBOX_HPP
#include <common/macros.h>

THIRD_PARTY_HEADERS_BEGIN
#include <Eigen/Geometry>
THIRD_PARTY_HEADERS_END

class BoundingBox {
 private:
  Eigen::Vector2d toPoint(const Eigen::Vector2d& x) { return x; }
  Eigen::Vector2d toPoint(const Eigen::Vector3d& x) { return x.head<2>(); }

 public:
  void enlarge(const Eigen::Vector2d& point) {
    min(0) = fmin(min(0), point(0));
    min(1) = fmin(min(1), point(1));
    max(0) = fmax(max(0), point(0));
    max(1) = fmax(max(1), point(1));
  }

  void enlarge(double distance) {
    min -= Eigen::Vector2d(distance, distance);
    max += Eigen::Vector2d(distance, distance);
  }

  template <typename PointTypeCollection>
  BoundingBox(const PointTypeCollection& points) {
    assert(!points.empty());
    min = max = toPoint(points.front());
    for (const auto& point : points) {
      enlarge(toPoint(point));
    }
  }

  BoundingBox() = default;
  BoundingBox(const BoundingBox&) = default;
  BoundingBox& operator=(const BoundingBox&) = default;
  BoundingBox(BoundingBox&&) = default;
  BoundingBox& operator=(BoundingBox&&) = default;


  bool containsPoint(const Eigen::Vector2d& point) const {
    return !(point.x() < min.x() || point.x() > max.x() ||
             point.y() < min.y() || point.y() > max.y());
  }

  Eigen::Vector2d getMin() const { return min; }

  Eigen::Vector2d getMax() const { return max; }

  Eigen::Vector2d getCenter() const { return (min + max) / 2; }

 private:
  Eigen::Vector2d min = Eigen::Vector2d::Zero();
  Eigen::Vector2d max = Eigen::Vector2d::Zero();
};

#endif  // BOUNDINGBOX_HPP
