#include "pedestrian_classifier.h"

THIRD_PARTY_HEADERS_BEGIN
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include <boost/algorithm/cxx11/all_of.hpp>
#include <boost/algorithm/cxx11/any_of.hpp>
#include <boost/range/adaptor/filtered.hpp>
#include <boost/range/algorithm/partition.hpp>
#include <boost/range/algorithm/random_shuffle.hpp>
#include <boost/range/algorithm/transform.hpp>
#include <boost/range/numeric.hpp>

#include <ros/console.h>
#include <ros/package.h>
THIRD_PARTY_HEADERS_END
#include "common/adaptors.h"
#include "common/angle_conversions.h"
#include "common/basic_statistics.h"
#include "common/basic_statistics_eigen.h"
#include "common/best_score.h"
#include "common/eigen_adaptors.h"
#include "common/eigen_utils.h"
#include "common/make_vector.h"
#include "common/minmax_element.h"
#include "common/pca_eigen.h"
#include "common/polynomialfit.h"
#include "opencv_eigen_conversions.h"

namespace road_object_detection {

using boost::adaptors::filtered;
using boost::adaptors::transformed;

const std::string PedestrianClassifier::NAMESPACE("pedestrian_classifier");

const ParameterString<double> PedestrianClassifier::C_BINARIZATION(
    NAMESPACE + "/binarization/c");
const ParameterString<double> PedestrianClassifier::SIGMA_BINARIZATION(
    NAMESPACE + "/binarization/sigma");
const ParameterString<int> PedestrianClassifier::BLOCK_SIZE_BINARIZATION(
    NAMESPACE + "/binarization/block_size");

const ParameterString<int> PedestrianClassifier::MIN_CONTOUR_SIZE(
    NAMESPACE + "/cont_min_size");
const ParameterString<double> PedestrianClassifier::SIGMA_GAUSS(
    NAMESPACE + "/gaussian_window/sigma");
const ParameterString<int> PedestrianClassifier::BLOCK_SIZE_GAUSS(
    NAMESPACE + "/gaussian_window/block_size");

const ParameterString<double> PedestrianClassifier::MIN_SCORE(NAMESPACE +
                                                              "/min_score");
const ParameterString<double> PedestrianClassifier::BOX_DEPTH(NAMESPACE +
                                                              "/box_depth");

const ParameterString<double> PedestrianClassifier::BB_WIDTH(
    NAMESPACE + "/bounding_box/width");
const ParameterString<double> PedestrianClassifier::BB_HEIGHT(
    NAMESPACE + "/bounding_box/height");
PedestrianClassifier::PedestrianClassifier(const common::CameraTransformation *const camera_transformation,
                                           common::ParameterInterface *const parameter_interface)
    : camera_transformation_(camera_transformation),
      parameters_ptr_(parameter_interface) {
  parameter_interface->registerParam(C_BINARIZATION);
  parameter_interface->registerParam(SIGMA_BINARIZATION);
  parameter_interface->registerParam(BLOCK_SIZE_BINARIZATION);
  parameter_interface->registerParam(MIN_CONTOUR_SIZE);
  parameter_interface->registerParam(SIGMA_GAUSS);
  parameter_interface->registerParam(BLOCK_SIZE_GAUSS);
  parameter_interface->registerParam(BB_WIDTH);
  parameter_interface->registerParam(BB_HEIGHT);
  parameter_interface->registerParam(MIN_SCORE);
  parameter_interface->registerParam(BOX_DEPTH);

  // read in template contour
  readInTemplate();
}

inline auto filterWithBox(const BoundingBox &box) {
  return [&box](const auto &p) {
    return cv::pointPolygonTest(box, cv::Point2f{p}, false) > 0;
  };
}

inline auto isPointInBox(const BoundingBox &bb) {
  return [&bb](const auto &p) { return cv::pointPolygonTest(bb, p, false) >= 0; };
}

inline auto toPolygonPoint() {
  return [](const auto &p) { return PolygonPoint(p.x, p.y); };
}

inline auto cast2f() {
  return [](const auto &p) { return cv::Point2f(p); };
}

inline auto shift(const cv::Point &vector) {
  return [&vector](const auto &p) { return p - vector; };
}

inline auto shift2f(const cv::Point2f &vector) {
  return [&vector](const auto &p) { return p - vector; };
}

inline auto scale(const float scale_factor) {
  return [scale_factor](const auto &p) { return p / scale_factor; };
}

inline PolygonPoints contourToPolygonPoints(const Contour2f &contour) {
  if (contour.empty()) {
    return {};
  }
  PolygonPoints polygon_points;
  polygon_points.reserve(contour.size());
  boost::transform(contour, std::back_inserter(polygon_points), toPolygonPoint());
  //  polygon_points.emplace_back(contour.front().x, contour.front().y);
  return polygon_points;
}

RoadObjects PedestrianClassifier::classify(const Features &features) {
  // this classifier is only used, when demanded by the environmental model,
  // i.e. if there's a roi defined, where the classifier has to search. If no
  // roi is defined, there shall not be any classification
  if (!features.roi_) {
    return {};
  }

  // perform binarization and extract contours in roi patch
  const auto binarized_img = binarizeImage(features.roi_->img_patch_.image);

  // extract contours
  Contours contours;
  Hierarchy hierarchy;
  extractContours(binarized_img, contours, hierarchy);

  // roi top left postition, needed for transformation to ground
  const auto roi_tl = features.roi_->img_patch_.position;

  // filter contours
  auto contour_hierarchy = filterContours(contours, hierarchy, features.roi_.get());

  RoadObjects pedestrians;
  // acces parameters always before loop --> efficiency
  const auto min_score = parameters_ptr_->getParam(MIN_SCORE);
  const auto bb_height = parameters_ptr_->getParam(BB_HEIGHT);
  const auto bb_width = parameters_ptr_->getParam(BB_WIDTH);

  for (auto &conts : contour_hierarchy) {

    const auto contour_on_objectplane =
        transformContourToObjectPlane(conts.first, conts.second, bb_height, bb_width);

    // determination of base area, based on the base line of the object plane
    const auto base_area = computeHullPolygon(conts.second, roi_tl);

    const auto pose_translation = common::mean<VehiclePoint>(base_area);
    const auto pose_angle = common::getAngle(
        Eigen::Vector2d::UnitX(), to2D(base_area.back() - base_area.front()));

    const VehiclePose pose = Eigen::Translation3d{pose_translation} *
                             Eigen::AngleAxisd{pose_angle, VehiclePoint::UnitZ()};
    const auto score = computeScore(contour_on_objectplane);

    ROS_DEBUG("pedestrian_classifier found pedestrian, score is %f", score);

    if (score < min_score) {
      continue;
    }

    pedestrians.push_back(std::make_unique<Pedestrian>(
        features.timestamp, score, pose, base_area));
  }

  return pedestrians;
}

size_t PedestrianClassifier::getClassifierId() const {
  return typeid(PedestrianClassifier).hash_code();
}

cv::Mat PedestrianClassifier::binarizeImage(const cv::Mat &img_gray) const {
  const auto blocksize_bin = parameters_ptr_->getParam(BLOCK_SIZE_BINARIZATION);
  const auto sigma_bin = parameters_ptr_->getParam(SIGMA_BINARIZATION);
  const auto c = parameters_ptr_->getParam(C_BINARIZATION);
  const auto blocksize_gauss = parameters_ptr_->getParam(BLOCK_SIZE_GAUSS);
  const auto sigma_gauss = parameters_ptr_->getParam(SIGMA_GAUSS);
  double stddev = 0.0;
  cv::Mat image_binary;

  // NOTE: adaptive threshold binarization performs well on godd illumination
  // conditions
  // when illumination is poor (e.g. in MRT-Maschinenhalle), Canny filter is
  // more stable. Therefore it is currently used for tests.

  // compute mean and std_dev
  if (sigma_bin != 0.0) {
    cv::Scalar mean_scalar, stddev_scalar;
    cv::meanStdDev(img_gray, mean_scalar, stddev_scalar);
    stddev = stddev_scalar.val[0];
  }

  cv::GaussianBlur(img_gray, image_binary, cv::Size(blocksize_gauss, blocksize_gauss), sigma_gauss);

  // binarize image
  cv::adaptiveThreshold(image_binary,
                        image_binary,
                        255,
                        cv::ADAPTIVE_THRESH_GAUSSIAN_C,
                        cv::THRESH_BINARY,
                        blocksize_bin,
                        -sigma_bin * stddev - c);

  //  cv::Canny(img_gray, image_binary, c, sigma_bin, blocksize_bin);

  return image_binary;
}

void PedestrianClassifier::extractContours(const cv::Mat &img_bin,
                                           Contours &contours,
                                           Hierarchy &hierarchy) const {

  // find all Contours in binarized roi patch
  cv::findContours(img_bin, contours, hierarchy, CV_RETR_CCOMP, CV_CHAIN_APPROX_TC89_L1);

  assert(contours.size() == hierarchy.size() &&
         "contours and hierarchy must be of the same size");
}

HierarchicalContours PedestrianClassifier::filterContours(const Contours &contours,
                                                          const Hierarchy &hierarchy,
                                                          const ROI &roi_) const {

  // construct vector of contours and related hierarchies
  auto toPair = [](const auto &c, const auto &h) { return std::make_pair(c, h); };
  ContoursWithHierarchy conts_W_hierarchy;
  conts_W_hierarchy.reserve(contours.size());
  boost::transform(contours, hierarchy, std::back_inserter(conts_W_hierarchy), toPair);

  const auto half_bb_height = 0.5 * parameters_ptr_->getParam(BB_HEIGHT);
  const auto half_bb_width = 0.5 * parameters_ptr_->getParam(BB_WIDTH);

  const auto toCVPoint = [](const auto &p) {
    return imagePointExactToCvPoint2f(to2D(p));
  };

  // roi in vehicle coordinates
  const auto roi_boundary = common::make_vector(roi_.boundary_ | transformed(toCVPoint));

  // filter contours: extract all contours, that do not have any child
  // but have a super contour
  const auto filterContours = [](const auto &ch) {
    return ch.second[2] < 0 /*&& ch.second[3] >= 0*/;
  };
  const auto toContourAndBox = [this, &roi_, &half_bb_height, &half_bb_width](const auto &ch) {
    return std::make_pair(
        ch.first,
        this->refineBoundingBox(
            roi_.pose_, roi_.img_patch_, ch.first, half_bb_height, half_bb_width));
  };

  const auto min_size_contours =
      static_cast<std::size_t>(parameters_ptr_->getParam(MIN_CONTOUR_SIZE));
  const auto filterSmallConts = [&min_size_contours](const auto &c) {
    return c.first.size() > min_size_contours;
  };

  const auto isContourInBox = [](const auto &cb) {
    return boost::algorithm::all_of(cb.first, isPointInBox(cb.second));
  };

  const auto isContourPartOfROI = [this, &roi_boundary, half_bb_height, &roi_](const auto &c) {
    return cv::pointPolygonTest(roi_boundary,
                                imagePointExactToCvPoint2f(to2D(this->getContourCenter(
                                    c.first, roi_.img_patch_, half_bb_height))),
                                false) >= 0;
  };

  return common::make_vector(conts_W_hierarchy | filtered(filterContours) |
                             transformed(toContourAndBox) | filtered(filterSmallConts) |
                             filtered(isContourInBox) | filtered(isContourPartOfROI));
}

void PedestrianClassifier::readInTemplate() {
  const std::string dir =
      ros::package::getPath("perception") +
      "/src/road_object_detection/classifiers/pedestrian_classifier/";

  // read in contour as image
  cv::Mat template_image =
      cv::imread(dir + "pedestrian_pictogram.png", cv::IMREAD_GRAYSCALE);
  // rotate in order to match coordinate aaxis' directions
  cv::rotate(template_image, template_image, cv::ROTATE_180);

  // binarize target
  cv::Mat template_bin;
  cv::threshold(template_image, template_bin, 130, 255, CV_THRESH_BINARY);

  // find contours and hierarchy
  Contours contours;
  Hierarchy hierarchy;
  cv::findContours(template_bin, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE);

  // extract innerst contour
  const auto filterHierarchy = [](const auto &h) { return h[3] > 0; };
  boost::adaptors::filter(hierarchy, filterHierarchy);

  const auto template_contour =
      common::make_vector(contours[static_cast<std::size_t>(hierarchy.front()[2])] |
                          transformed(cast2f()));

  // compute center of gravity
  const cv::Moments template_moments = cv::moments(template_contour);
  assert(std::fabs(template_moments.m00) > std::numeric_limits<double>::epsilon() &&
         "Pedestrian Template corrupted! Check if image contains valid "
         "pictogramm!");

  const cv::Point2f cog(
      static_cast<float>(template_moments.m10 / template_moments.m00),
      static_cast<float>(template_moments.m01 / template_moments.m00));

  // transform coordinate origin to mean of contour
  const auto minmax_y = common::minmax_element(template_contour | common::y_values);

  const auto height = *minmax_y.second - *minmax_y.first;
  template_contour_ = common::make_vector(
      template_contour | transformed(shift2f(cog)) | transformed(scale(height)));

  // scale template contour to height 1 and append to boost polygon
  //  const auto template_y_max = boundary.size().height;
  assign_points(template_polygon_, contourToPolygonPoints(template_contour_));
  correct(template_polygon_);
}

BoundingBox PedestrianClassifier::extractBoundingBox(const VehiclePose &pose,
                                                     const ImagePatch &roi_patch,
                                                     const double half_bb_height,
                                                     const double half_bb_width) const {
  using BoxPoint = Eigen::Vector3d;

  //  const double z_dist = parameters_ptr_->getParam(BB_HEIGHT);
  //  const double y_dist = parameters_ptr_->getParam(BB_WIDTH);
  const BoxPoint z_dist{0.0, 0.0, half_bb_height};
  const BoxPoint y_dist{0.0, half_bb_width, 0.0};

  // compute edges of box object coordinates (clockwise started from bottom
  // right point)
  const common::EigenAlignedVector<BoxPoint> edges_box{
      -y_dist - z_dist, y_dist - z_dist, y_dist + z_dist, -y_dist + z_dist};

  const auto toImage = [this](const auto &vp) {
    return camera_transformation_->transformVehicleToImageExact(vp);
  };

  const auto toCVPoint = [&roi_patch](const auto &p) {
    return imagePointExactToCvPoint(p) - roi_patch.position;
  };

  const auto bounding_box =
      common::make_vector(edges_box | common::eigen_transformed(pose) |
                          transformed(toImage) | transformed(toCVPoint));

  return bounding_box;
}

BoundingBox PedestrianClassifier::refineBoundingBox(const VehiclePose &pose,
                                                    const ImagePatch &img_patch,
                                                    const Contour &cont,
                                                    const double half_box_height,
                                                    const double half_box_width) const {

  //  const auto nearest_on_ground =
  //      camera_transformation_->transformImageToGround(cvPointToImagePoint(
  //          *common::minmax_score(cont, [](const auto &p) { return p.x; })
  //               .first));
  const auto contour_center_vehicle = getContourCenter(cont, img_patch, half_box_height);

  const VehiclePose vehicle_T_objectplane =
      Eigen::Translation3d{contour_center_vehicle} * pose.linear();
  return extractBoundingBox(vehicle_T_objectplane, img_patch, half_box_height, half_box_width);
}

Contour2f PedestrianClassifier::transformContourToObjectPlane(const Contour &contour_in_image,
                                                              const BoundingBox &bounding_box,
                                                              const double bb_height,
                                                              const double bb_width) const {
  // construct transformation from plane coordinates to image frame
  // origin of plane frame is on the contour's edge points' mean

  const auto toHomogeneousEigenPoint = [](const auto &cv_p) {
    const ImagePointExact p = cvPointToImagePointExact(cv_p);
    return common::toHomogenous(p);
  };

  const auto toNormalPoint = [](const auto &hp) {
    const ImagePointExact p = to2D(hp) / hp.z();
    return imagePointExactToCvPoint2f(p);
  };

  const auto focal_length_x =
      camera_transformation_->getIntrinsicCalibrationMatrix()(0, 0);
  const auto focal_length_y =
      camera_transformation_->getIntrinsicCalibrationMatrix()(1, 1);

  const cv::Point2f half_width(static_cast<float>(0.5 * focal_length_x * bb_width), 0.0);
  const cv::Point2f half_height(0.0, static_cast<float>(0.5 * focal_length_y * bb_height));
  const std::vector<cv::Point2f> targets = {-half_width - half_height,
                                            half_width - half_height,
                                            half_width + half_height,
                                            -half_width + half_height};

  const auto to2f = [](const auto &p) { return cv::Point2f(p); };

  const auto edges_in = common::make_vector(bounding_box | transformed(to2f));
  cv::Mat homography(2, 4, CV_32FC1);
  homography = cv::getPerspectiveTransform(edges_in, targets);

  const Eigen::Map<Eigen::Matrix3d, Eigen::ColMajor, Eigen::Stride<1, 3>> H_map(
      homography.ptr<double>());

  const Eigen::Matrix3d object_H_image = H_map;

  const common::EigenAlignedVector<Eigen::Vector3d> hom_image_points =
      common::make_eigen_vector(contour_in_image | transformed(toHomogeneousEigenPoint));

  common::EigenAlignedVector<Eigen::Vector3d> hom_obj_points;
  hom_obj_points.reserve(hom_image_points.size());
  for (const auto &p : hom_image_points) {
    hom_obj_points.push_back(object_H_image * p);
  }

  return common::make_vector(hom_obj_points | transformed(toNormalPoint));
}

inline double absArea(const std::vector<Polygon> &ps) {
  double sum = 0;
  for (const auto &p : ps) {
    sum += std::fabs(boost::geometry::area(p));
  }
  return sum;
}

double PedestrianClassifier::computeScore(Contour2f contour) const {
  // note: this has tp be width attribute since the contours height is described
  // by the x-coordinate in the object plane and this correspondents to the
  // width attribute in opencv

  // compute center of gravity
  const cv::Moments template_moments = cv::moments(contour);
  if (std::fabs(template_moments.m00) <= std::fabs(std::numeric_limits<double>::epsilon())) {
    ROS_DEBUG("Corrupted Contour, returning 0!");
    return 0;
  }
  const cv::Point2f cog(
      static_cast<float>(template_moments.m10 / template_moments.m00),
      static_cast<float>(template_moments.m01 / template_moments.m00));

  // shift polygon in order to meet coordinate frame requirements
  const auto minmax_y = common::minmax_element(contour | common::y_values);

  const auto height = *minmax_y.second - *minmax_y.first;
  const auto contour_centered = common::make_vector(
      contour | transformed(shift2f(cog)) | transformed(scale(height)));

  const auto difference_angle = common::getAngle(
      common::getPrincipalComponent(contour_centered | transformed(LIFT(toEigen))),
      common::getPrincipalComponent(template_contour_ | transformed(LIFT(toEigen))));

  cv::Mat trafo = cv::getRotationMatrix2D(
      cv::Point2f{0, 0}, common::toDegree(difference_angle), 1);
  cv::transform(contour_centered, contour_centered, trafo);

  // construct polygon (scaled to one)
  Polygon polygon;
  assign_points(polygon, contourToPolygonPoints(contour_centered));
  correct(polygon);

  std::vector<Polygon> polygon_intersection;
  std::vector<Polygon> polygon_union;

  if (!intersects(template_polygon_, polygon)) {
    ROS_WARN_THROTTLE(1.0, "No intersection!");
    return 0;
  }

  try {
    intersection(template_polygon_, polygon, polygon_intersection);
    union_(template_polygon_, polygon, polygon_union);
  } catch (const overlay_invalid_input_exception &ex) {
    ROS_WARN_STREAM_THROTTLE(1.0, ex.what());
    return 0;
  }

  // computation of i_o_u

  const auto tp_intersection = absArea(polygon_intersection);
  const auto tp_union = absArea(polygon_union);

  if (tp_intersection > tp_union || tp_union <= 0) {
    ROS_WARN_THROTTLE(1.0, "intersection larger than union, returning zero!");
    return 0;
  }

  return tp_intersection / tp_union;
}

VehiclePoints PedestrianClassifier::computeHullPolygon(const BoundingBox &bounding_box,
                                                       const cv::Point &roi_tl) const {

  const auto box_depth = parameters_ptr_->getParam(BOX_DEPTH);

  // compute front points of base area
  const VehiclePoint fp1_vec = camera_transformation_->transformImageToGround(
      cvPointToImagePoint(bounding_box.front() + roi_tl));
  const VehiclePoint fp2_vec = camera_transformation_->transformImageToGround(
      cvPointToImagePoint(*std::next(bounding_box.begin()) + roi_tl));
  // compute offset vector between front line and rear line
  const auto diff = fp2_vec - fp1_vec;
  const VehiclePoint offset =
      box_depth *
      common::ensureSameOrientation(diff.unitOrthogonal(), VehiclePoint::UnitX());

  return {{fp1_vec, fp2_vec, fp2_vec + offset, fp1_vec + offset}};
}

VehiclePoint PedestrianClassifier::getContourCenter(const Contour &cont,
                                                    const ImagePatch &img_patch,
                                                    const double half_box_height) const {
  // goal is to use a priori information about the bounding box geometry to
  // translate bounding box to the place, where the contour is located, assuming
  // the center of the contour to be the center of the box

  // compute center of gravity
  const cv::Moments template_moments = cv::moments(common::make_vector(
      cont | transformed(shift(-img_patch.position)) | transformed(cast2f())));
  if (std::fabs(template_moments.m00) <= std::fabs(std::numeric_limits<double>::epsilon())) {
    ROS_DEBUG("Corrupted Contour, returning 0-point!");
    return VehiclePoint::Zero();
  }
  const cv::Point2f cog(
      static_cast<float>(template_moments.m10 / template_moments.m00),
      static_cast<float>(template_moments.m01 / template_moments.m00));

  const auto contour_center = common::toHomogenous(cvPoint2fToImagePointExact(cog));

  const Eigen::Matrix3d A_inverse =
      camera_transformation_->getIntrinsicCalibrationMatrix().inverse();
  const Eigen::Matrix3d R_inverse = camera_transformation_->getRotationMatrix().inverse();
  const auto translation = camera_transformation_->getTranslationVector();

  // compute beam onto parallel plane to ground plane with a constant
  // x3-coordinate that's equal to the half box height
  const Eigen::Vector3d beam = R_inverse * (A_inverse * contour_center);
  const Eigen::Vector3d translation_veh = R_inverse * translation;

  assert(beam.z() != 0 &&
         "Beam is the line at infinity. Check camera calibration!");

  // given the beam and the translation vector, the vehicle coordinates of the
  // contours center can be computed
  const auto scale_factor = (half_box_height + translation_veh.z()) / beam.z();
  return VehiclePoint{scale_factor * beam.x() - translation_veh.x(),
                      scale_factor * beam.y() - translation_veh.y(),
                      half_box_height};
}

}  // namespace road_object_detection
