#include "contour_template.h"
#include <common/macros.h>

THIRD_PARTY_HEADERS_BEGIN
#include <ros/console.h>

#include <functional>
#include <iterator>
#include <sstream>

#include <algorithm>
#include <boost/bind.hpp>
#include <boost/range/algorithm.hpp>
#include <boost/range/algorithm_ext.hpp>

THIRD_PARTY_HEADERS_END

#include "../../../utils/foot_finder.h"
#include "common/angle_conversions.h"
#include "common/camera_transformation.h"
#include "common/eigen_utils.h"
#include "common/pca_eigen.h"
#include "common/polynomial_utils.h"
#include "opencv_eigen_conversions.h"

#include "arrow_contour_template.h"
#include "polygon.h"
#include "speed_limit_contour_template.h"
#include "unclassified_contour_template.h"

namespace cv {

static inline void write(FileStorage &fs,
                         const String &,
                         const road_object_detection::contour_classifier::ContourTree &x) {
  x.write(&fs);
}
static inline void write(FileStorage &fs,
                         const String &,
                         const road_object_detection::contour_classifier::ContourNode &x) {
  x.write(&fs);
}

static inline void read(const FileNode &node,
                        road_object_detection::contour_classifier::ContourTree &x,
                        const road_object_detection::contour_classifier::ContourTree &default_value =
                            road_object_detection::contour_classifier::ContourTree()) {
  if (node.empty())
    x = default_value;
  else
    x.read(node);
}

static inline void read(const FileNode &node,
                        road_object_detection::contour_classifier::ContourNode &x,
                        const road_object_detection::contour_classifier::ContourNode &default_value =
                            road_object_detection::contour_classifier::ContourNode()) {
  if (node.empty())
    x = default_value;
  else
    x.read(node);
}

}  // namespace cv

namespace road_object_detection {

namespace contour_classifier {

double ContourTree::sampling_resolution = 0.0;
bool ContourTree::dump_polygons = false;

inline double centerDistance(const CvContour2D &a, const CvContour2D &b) {
  cv::Moments m_a = cv::moments(a), m_b = cv::moments(b);

  auto x_a = moments_to_center_point(m_a), x_b = moments_to_center_point(m_b);
  return (x_a - x_b).norm();
}

inline double lateralDistance(const CvContour2D &a, const CvContour2D &b) {
  cv::Moments m_a = cv::moments(a), m_b = cv::moments(b);

  auto y = [](const cv::Moments &m) { return m.m01 / m.m00; };
  return std::abs(y(m_a) - y(m_b));
}

bool sortByDistanceToMiddleLane(const ContourTree &a, const ContourTree &b) {
  return a.distance_to_middle_lane_polynomial < b.distance_to_middle_lane_polynomial;
}

void ContourTemplate::sanitizeContours() {
  // make sure, that trees can be indexed from left to right
  contour_trees_ = boost::sort(contour_trees_, sortByDistanceToMiddleLane);

  // trees should be centered in their local coordinate frame and upright
  boost::for_each(contour_trees_, std::mem_fn(&ContourTree::centerAndAlign));

  // normalize contours
  normalizeContours();
}

auto transformContourPoint(const Eigen::Affine3d &vehicle_T_contour) {
  return [v_T_c = to2D(vehicle_T_contour)](const auto &p) {
    return toCV(v_T_c * toEigen(p));
  };
}

VehiclePoints ContourTemplate::getEnclosingRectangle(const ContourTrees &contour_trees) const {
  // create a point set of all contour points in vehicle frame
  std::vector<cv::Point2f> contour_points;
  Eigen::Affine3d scale;
  for (const auto &contour_tree : contour_trees) {
    scale = Eigen::Scaling(contour_tree.scaling) * Eigen::Affine3d::Identity();
    contour_points.reserve(contour_points.size() + contour_tree.contour.size());
    boost::transform(
        contour_tree.contour,
        std::back_inserter(contour_points),
        transformContourPoint(contour_tree.vehicle_T_contour * scale.inverse()));
  }

  cv::Point2f enclosing_rect[4];
  cv::RotatedRect rot_rect = cv::minAreaRect(contour_points);
  rot_rect.points(enclosing_rect);

  // for (auto &point : enclosing_rect) {
  //  point = toCV(contour_trees[0].scaling_transformation.inverse() * toEigen(point));
  //}

  // convert to vehicle points
  VehiclePoints vehicle_contour_points;
  vehicle_contour_points.reserve(4);
  const auto toEigen3d = [](const auto &point) { return to3D(toEigen(point)); };
  boost::transform(enclosing_rect, std::back_inserter(vehicle_contour_points), toEigen3d);

  return vehicle_contour_points;
}


VehiclePoints ContourTemplate::getConvexHull(const ContourTrees &contour_trees) const {
  // create a point set of all contour points in vehicle frame
  std::vector<cv::Point2f> contour_points;
  for (const auto &contour_tree : contour_trees) {
    contour_points.reserve(contour_points.size() + contour_tree.contour.size());
    boost::transform(contour_tree.contour,
                     std::back_inserter(contour_points),
                     transformContourPoint(contour_tree.vehicle_T_contour));
  }

  // create convex hull
  std::vector<cv::Point2f> convex_hull;
  cv::convexHull(contour_points, convex_hull);

  // convert to vehicle points
  VehiclePoints vehicle_contour_points;
  vehicle_contour_points.reserve(convex_hull.size());
  const auto toEigen3d = [](const auto &point) { return to3D(toEigen(point)); };
  boost::transform(convex_hull, std::back_inserter(vehicle_contour_points), toEigen3d);

  return vehicle_contour_points;
}

ContourTemplate::ContourTemplate(const CvContours2D &contours,
                                 const CvHierarchy &hierarchy,
                                 const common::DynamicPolynomial &middle_lane_polynomial)
    : ContourTemplate(cvContoursToContourTrees(contours, hierarchy, middle_lane_polynomial)) {}

ContourTemplate::ContourTemplate(const cv::FileStorage &fs)
    : ContourTemplate(read(fs)) {}

ContourTemplate::ContourTemplate(const ContourTrees &contour_trees)
    : contour_trees_(contour_trees) {
  sanitizeContours();
}

void ContourTemplate::normalizeContours() {
  for (ContourTree &contour : contour_trees_) {
    contour.normalizeContour();
  }
}

double ContourTemplate::match(ContourTrees other_trees, const double min_zero_match_score) const {
  if (contour_trees_.size() != other_trees.size())
    return 0;

  other_trees = boost::sort(other_trees, sortByDistanceToMiddleLane);

  // Currently there are only speedlimits, that have two outer contours.

  const double score = contour_trees_.front().match(other_trees.front());

  // The second one, the zero should always be the same, therefore do not
  // consider it for the matching. The match has only has to be sufficient good
  // or it might be an outlier. This is very likely for a 10 as the 1 matches
  // many occuring patterns but the 0 hopefully won't.
  if (contour_trees_.size() == 2) {
    double score_zero = contour_trees_.back().match(other_trees.back());
    if (score_zero < min_zero_match_score) {
      ROS_DEBUG("The zero does not match! (%f < %f", score_zero, min_zero_match_score);
      return 0.0;
    }
  }

  return score;
}

bool centerAndRotateContour(const CvContour2D &contour,
                            const Eigen::Affine2d &contour_base_T_vehicle,
                            CvContour2D *centered_contour,
                            Eigen::Vector2d *contour_base_P_contour) {
  assert(centered_contour);
  assert(contour_base_P_contour);

  *centered_contour = contour;
  cv::Moments m = cv::moments(contour);

  // position of this contour in vehicle frame (original contour frame)
  const Eigen::Vector2d vehicle_P_contour = moments_to_center_point(m);

  // this will be the new contour_base_P_parent
  *contour_base_P_contour = contour_base_T_vehicle * vehicle_P_contour;

  for (auto &point : *centered_contour) {
    // position relative to parent, FIXME
    point = toCV(contour_base_T_vehicle * toEigen(point));
  }

  return true;
}


// Both inputs in local vehicle coordinate frame
bool centerAndRotateContour(const CvContour2D &contour,
                            const common::DynamicPolynomial &middle_lane_polynomial,
                            CvContour2D *centered_contour,
                            Eigen::Affine2d *contour_base_T_vehicle,
                            Eigen::Vector2d *contour_lf_P_contour_base) {
  assert(centered_contour);
  assert(contour_base_T_vehicle);

  *centered_contour = contour;
  if (contour.empty()) {
    return false;
  }

  cv::Moments m = cv::moments(contour);

  if (m.m00 == 0.0) {
    return false;
  }

  auto vehicle_P_contour = moments_to_center_point(m);

  // get the closest point on the polynomial and its tangent
  const Eigen::Vector2d vehicle_P_lf =
      utils::findLotfusspunkt(middle_lane_polynomial, vehicle_P_contour);

  // rotation in local coordinate frame
  const auto yaw = common::tangentAngle(middle_lane_polynomial, vehicle_P_lf.x());

  // first coordinate frame
  // contour lotfußpunkt with 0° orientation
  const Eigen::Rotation2Dd vehicle_R_contour_lf(yaw);
  const Eigen::Affine2d vehicle_T_contour_lf =
      Eigen::Translation2d(vehicle_P_lf) * vehicle_R_contour_lf;

  // second frame
  *contour_lf_P_contour_base = vehicle_P_contour - vehicle_P_lf;
  *contour_lf_P_contour_base = vehicle_R_contour_lf.inverse() * (*contour_lf_P_contour_base);
  const auto contour_lf_T_contour_base = Eigen::Translation2d(*contour_lf_P_contour_base);

  // combine both
  *contour_base_T_vehicle = (vehicle_T_contour_lf * contour_lf_T_contour_base).inverse();

  for (auto &point : *centered_contour) {
    point = toCV((*contour_base_T_vehicle) * toEigen(point));
  }

  assert(contour_lf_P_contour_base->array().isFinite().all());
  return true;
}

//!
//! \brief isValid checks if the given contour defines a simple area,
//! nothing like a butterfly contour with self intersection
//! \param contour the contour to check.
//! \return whether the contour is valid.
//!
inline bool isValid(const CvContour2D &contour) {
  if (contour.size() < 3) {  // at least a triangle ...
    return false;
  }

  // According to the OpenCV documentation:
  // Since the contour moments are computed using Green formula,
  // you may get seemingly odd results for contours with self-intersections,
  // e.g. a zero area (m00) for butterfly-shaped contours.
  auto m = cv::moments(contour);
  return m.m00 > 0.0;
}


ContourTrees ContourTemplate::cvContoursToContourTrees(const CvContours2D &contours,
                                                       const CvHierarchy &hierarchy,
                                                       const common::DynamicPolynomial &middle_lane_polynomial) {



  ContourTrees root_nodes;
  //  Eigen::Vector2d contour_base_P_parent(0, 0);

  auto find_children = [&](int i, const Eigen::Affine2d &contour_base_T_vehicle) {
    ContourNodes children;
    const cv::Vec4i &h = hierarchy[i];

    // pointing to the first (possible) child, can also be -1 (invalid)
    int j = h[2];

    while (j != -1) {
      const cv::Vec4i &c = hierarchy[j];

      if (isValid(contours[j])) {  // at least a triangle ...
        ContourNode node;
        centerAndRotateContour(
            contours[j], contour_base_T_vehicle, &node.contour, &node.relative_position);
        children.push_back(node);
      }

      j = c[0];  // move the index to the next possible child at same hierarchy
                 // level, can be -1 if no child left
    }
    return children;
  };

  // loop once to find roots => h[3] = -1 => no parent
  for (unsigned int i = 0; i < hierarchy.size(); ++i) {
    const cv::Vec4i &h = hierarchy[i];
    if (h[3] == -1) {  // root node detected
      ContourTree root_node;

      Eigen::Affine2d contour_base_T_vehicle;
      Eigen::Vector2d contour_lf_P_contour_base;

      if (!isValid(contours[i]))
        continue;

      if (!centerAndRotateContour(contours[i],
                                  middle_lane_polynomial,
                                  &root_node.contour,
                                  &contour_base_T_vehicle,
                                  &contour_lf_P_contour_base)) {
        continue;
      }

      // position invariant along polynomial
      root_node.children = find_children(i, contour_base_T_vehicle);

      // set vehicle_T_contour from Affine2d
      auto vehicle_T_contour_base = contour_base_T_vehicle.inverse();
      root_node.vehicle_T_contour = Eigen::Affine3d::Identity();
      root_node.vehicle_T_contour.linear().topLeftCorner<2, 2>() =
          vehicle_T_contour_base.linear();
      root_node.vehicle_T_contour.translation().topLeftCorner<2, 1>() =
          vehicle_T_contour_base.translation();

      root_node.distance_to_middle_lane_polynomial = -contour_lf_P_contour_base[1];  // positive on right lane

      if (!isValid(root_node.contour))
        continue;

      assert(root_node.vehicle_T_contour.matrix().array().isFinite().all());

      root_nodes.push_back(root_node);
    }
  }

  boost::for_each(root_nodes, std::mem_fn(&ContourTree::centerAndAlign));

  return root_nodes;
}

void ContourTemplate::write(const std::string &filename) {
  cv::FileStorage fs(filename, cv::FileStorage::WRITE);

  if (!fs.isOpened()) {
    ROS_ERROR_STREAM("Could not create file " << filename);
    return;
  }

  write(&fs);
}

void ContourTemplate::write(cv::FileStorage *fs) const {  // Write serialization for this class

  *fs << "template_id" << getTemplateId() << "contour_trees" << contour_trees_;
  writeAdditionalFields(fs);
}

ContourTrees ContourTemplate::read(const cv::FileStorage &fs) {  // Read serialization for this class
  ContourTrees contour_trees;

  if (fs["contour_trees"].empty()) {
    throw std::runtime_error(
        "Template is invalid. Required field 'contour_trees' does not exist!");
  }

  fs["contour_trees"] >> contour_trees;
  return contour_trees;
}

ContourTemplatePtr ContourTemplateFactory::createFromFile(const std::string &filename) {
  cv::FileStorage fs(filename, cv::FileStorage::READ);

  if (!fs.isOpened()) {
    ROS_ERROR_STREAM("Could not open file " << filename << " to read contour template!");
    return nullptr;
  }

  int template_id = static_cast<int>(fs["template_id"]);
  if (template_id == 0) {  // valid id is positive
    ROS_WARN_STREAM("Your template "
                    << filename
                    << " is marked as 'UNCATEGORIZED'. Consider changing the "
                       "template_id field to {1,2,3}.");
  }

  switch (static_cast<ContourTemplateType>(template_id)) {
    case ContourTemplateType::UNCATEGORIZED:
      return std::make_unique<UnclassifiedContourTemplate>(fs);
    case ContourTemplateType::ARROW:
      return std::make_unique<ArrowContourTemplate>(fs);
    case ContourTemplateType::SPEED_LIMIT:
      return std::make_unique<SpeedLimitContourTemplate>(fs);
    default:
      return nullptr;
  }
}

//!
//! \brief sampleLine samples a line defined by two points
//!
void sampleLine(const cv::Point2f &start,
                const cv::Point2f &end,
                double sampling_resolution,
                std::vector<cv::Point2f> &sampled_line) {

  auto dir = end - start;
  double d = cv::norm(dir);
  dir /= d;

  for (double e = 0; e < d; e += sampling_resolution) {
    sampled_line.push_back(start + e * dir);
  }
}

CvContour2D ContourTree::sampledOuterContour() const {
  assert(sampling_resolution > 0.0);

  if (contour.empty()) {
    return {};
  }

  CvContour2D contour_sampled;
  for (size_t i = 1; i < contour.size(); i++) {
    const auto &start = contour[i - 1];
    const auto &end = contour[i];

    sampleLine(start, end, sampling_resolution, contour_sampled);
  }

  sampleLine(contour.back(), contour.front(), sampling_resolution, contour_sampled);

  ROS_DEBUG_STREAM("Sampling from " << contour.size() << " to "
                                    << contour_sampled.size());

  return contour_sampled;
}


void applyTransformation(CvContour2D &contour, const Eigen::Affine2d &transformation) {
  for (auto &point : contour) {
    point = toCV(transformation * toEigen(point));
  }
}

void ContourTree::transform(const Eigen::Affine2d &transformation) {
  // transform both outer and innner contours
  applyTransformation(contour, transformation);
  for (auto &c : children) {
    applyTransformation(c.contour, transformation);
  }
}

void ContourTree::normalizeContour() {

  CvContour2D hull;
  cv::convexHull(contour, hull);
  double area = cv::contourArea(hull);

  scaling = std::sqrt(1.0 / area);

  Eigen::Affine2d scaling_transformation =
      Eigen::Scaling(scaling) * Eigen::Affine2d::Identity();

  transform(scaling_transformation);
}

double ContourTree::match(const ContourTree &other) const {
  auto poly_this = contourTreeToPolygon(*this);
  auto poly_other = contourTreeToPolygon(other);

  // ROS_DEBUG_STREAM("TemplateArea: " << cv::contourArea(contour) << " ContourArea: "
  //                                  << cv::contourArea(other.contour));

  double score = iou(poly_this, poly_other);

  // uncommenting this line will write a lot of files to /tmp. use it with care
  if (dump_polygons) {
    ROS_WARN_ONCE("You activated polygon dumping!");
    dumpPolygons(poly_this, poly_other);
  }

  return score;
}

void ContourNode::write(cv::FileStorage *fs) const {  // Write serialization for this class
  cv::Vec2d rp(relative_position[0], relative_position[1]);

  *fs << "{"
      << "contour" << contour << "relative_position" << rp << "}";
}

void ContourNode::read(const cv::FileNode &node) {  // Read serialization for this class
  cv::Vec2d rp;
  node["relative_position"] >> rp;
  relative_position = {rp[0], rp[1]};
  node["contour"] >> contour;
}


void ContourTree::write(cv::FileStorage *fs) const {  // Write serialization for this class
  *fs << "{"
      << "contour" << contour << "children" << children << "distance"
      << distance_to_middle_lane_polynomial << "}";
}

void ContourTree::read(const cv::FileNode &node) {  // Read serialization for this class
  node["children"] >> children;
  node["contour"] >> contour;
  node["distance"] >> distance_to_middle_lane_polynomial;

  contour.push_back(contour[0]);
}


void ContourTree::centerAndAlign() {
  // Each contour has only a small number of contour points where some might be
  // randomly spread along a (ideally) straight line.
  // Therefore, sample the contours with a fixed resolution
  CvContour2D contour_sampled = sampledOuterContour();

  auto mat = common::toMatrix2D(toEigen(contour_sampled));

  // shift contours to center
  const Eigen::Vector2d center = mat.cast<double>().colwise().mean();
  mat = mat.rowwise() - center.transpose();

  // fix contour orientation
  Eigen::MatrixXd eigenvectors;
  Eigen::VectorXd eigenvalues;
  common::principle_component_analysis(mat, &eigenvectors, &eigenvalues);

  //  auto pc = common::getPrincipalComponent(mat.cast<double>());
  double pc_angle = common::getAngle(eigenvectors.col(1), Eigen::Vector2d::UnitX());

  // could be asignment but if one calls this function several times, we might
  // want to accumulate the angle
  angle_to_middle_lane += pc_angle;

  // the eigenvalues represent the variance
  // => norm by 1/sigma
  // need reversed order here as rotation is based on largest (second)
  // eigenvalue

  // eigenvalues *= 2.0;
  // Eigen::Vector2d scaling(eigenvalues.array().rsqrt().reverse());

  Eigen::Affine2d trafo = /* Eigen::Scaling(scaling) * */ Eigen::Rotation2Dd(-pc_angle) *
                          Eigen::Translation2d(-center);

  transform(trafo);
}

Eigen::Vector2d moments_to_center_point(const cv::Moments &m) {
  assert(m.m00 != 0.0);
  return {m.m10 / m.m00, m.m01 / m.m00};
}

}  // namespace contour_classifier

}  // namespace road_object_detection
