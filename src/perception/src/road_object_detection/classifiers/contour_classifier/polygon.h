#ifndef POLYGON_H
#define POLYGON_H

#include "common/macros.h"

THIRD_PARTY_HEADERS_BEGIN
#include <vector>

#include <Eigen/Core>
#include <opencv2/core/types.hpp>

#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/polygon.hpp>
THIRD_PARTY_HEADERS_END

#include "contour_template.h"

namespace road_object_detection {

using Point = boost::geometry::model::d2::point_xy<float>;
using Points = std::vector<Point>;

using Polygon = boost::geometry::model::polygon<Point, false, false>;

using Hole = Points;

Point cvPointToPolygonPoint(const cv::Point2f &cv_point);

Points contourToPoints(const contour_classifier::CvContour2D &contour);

Points contourToPoints(const contour_classifier::CvContour2D &contour,
                       const Eigen::Vector2d &shift);


Polygon contourTreeToPolygon(const contour_classifier::ContourTree &contour_tree);

//!
//! \brief iou calculates intersection over union for polygons with holes
//! \param a a polygon with holes.
//! \param b another polygon with holes.
//! \return iou score [0,1]
//!
double iou(const Polygon &a, const Polygon &b);

//!
//! \brief dumpPolygons stores polygon pairs as vector graphics in /tmp
//! \param a a polygon with holes.
//! \param b another polygon with holes.
//!
void dumpPolygons(const Polygon &a, const Polygon &b);

}  // namespace road_object_detection



#endif  // POLYGON_H
