#include "polygon.h"

THIRD_PARTY_HEADERS_BEGIN
#include <ros/console.h>
#include <boost/range/algorithm.hpp>
#include <fstream>
#include <iostream>
THIRD_PARTY_HEADERS_END

namespace road_object_detection {

Point cvPointToPolygonPoint(const cv::Point2f &cv_point) {
  return Point(cv_point.x, cv_point.y);
}

Points contourToPoints(const contour_classifier::CvContour2D &contour) {
  if (contour.empty()) {
    return {};
  }

  Points points;
  points.reserve(contour.size() + 1);
  boost::transform(contour, std::back_inserter(points), &cvPointToPolygonPoint);
  points.push_back(cvPointToPolygonPoint(contour.front()));
  return points;
}

Polygon contourTreeToPolygon(const contour_classifier::ContourTree &contour_tree) {
  Polygon poly;
  boost::geometry::assign_points(poly, contourToPoints(contour_tree.contour));

  poly.inners().reserve(contour_tree.children.size());
  for (const auto &contour : contour_tree.children) {
    poly.inners().emplace_back();
    boost::geometry::append(poly.inners().back(), contourToPoints(contour.contour));
    boost::geometry::correct(poly.inners().back());
  }

  boost::geometry::correct(poly);
  return poly;
}

double absArea(const std::deque<Polygon> &ps) {
  double sum = 0;
  for (const auto &p : ps) {
    sum += std::abs(boost::geometry::area(p));
  }
  return sum;
}

double iou(const Polygon &a, const Polygon &b) {
  std::deque<Polygon> u, i;

  if (!boost::geometry::intersects(a, b)) {
    ROS_WARN("No intersection!");
    return 0;
  }

  try {
    boost::geometry::intersection(a, b, i);
    boost::geometry::union_(a, b, u);
  } catch (const boost::geometry::overlay_invalid_input_exception &ex) {
    ROS_WARN_STREAM_THROTTLE(1.0, ex.what());
    return 0;
  }

  if (u.empty() || i.empty()) {
    ROS_WARN_THROTTLE(1.0, "No iou score can be calculated. (empty)");
    return 0;
  }

  const double u_val = absArea(u);
  const double i_val = absArea(i);

  if (u_val < i_val) {
    ROS_WARN_THROTTLE(1.0, "No iou score can be calculated. (u < i)");
    return 0;
  }

  if (u_val == 0)
    return 0;

  return i_val / u_val;
}

void dumpPolygons(const Polygon &a, const Polygon &b) {
  // Declare a stream and an SVG mapper
  static int i = 0;
  i++;

  std::ofstream svg("/tmp/contours" + std::to_string(i) + ".svg");
  boost::geometry::svg_mapper<Point, true> mapper(svg, 800, 800);

  // define the domain that should be drawn
  float w = 0.2;
  Polygon domain;
  domain.outer().push_back({-w, -w});
  domain.outer().push_back({w, -w});
  domain.outer().push_back({w, w});
  domain.outer().push_back({-w, w});
  mapper.add(domain);

  mapper.add(a);
  mapper.add(b);
  mapper.map(domain,
             "fill-opacity:0.01;fill:rgb(0,204,0);stroke:rgb(0,204,0);"
             "stroke-width:2");
  mapper.map(a,
             "fill-opacity:0.5;fill:rgb(153,204,0);stroke:rgb(153,204,0);"
             "stroke-width:2");
  mapper.map(b,
             "fill-opacity:0.3;fill:rgb(51,51,153);stroke:rgb(51,51,153);"
             "stroke-width:2");

  mapper.text(Point{-0.15, 0.15},
              "iou: " + std::to_string(iou(a, b)),
              "fill:rgb(0,0,0);font-family:Arial;font-size:50px");

  Polygon origin;
  w = 0.001;
  origin.outer().push_back({-w, -w});
  origin.outer().push_back({w, -w});
  origin.outer().push_back({w, w});
  origin.outer().push_back({-w, w});

  mapper.map(origin,
             "fill-opacity:0.8;fill:rgb(10,10,10);stroke:rgb(0,0,0);"
             "stroke-width:2");
}


}  // namespace road_object_detection
