#include "pedestrian_classifier_debug.h"

THIRD_PARTY_HEADERS_BEGIN
#include <boost/range/adaptor/filtered.hpp>
#include <boost/range/adaptor/transformed.hpp>
#include <boost/range/numeric.hpp>

THIRD_PARTY_HEADERS_END
#include "common/adaptors.h"
#include "common/angle_conversions.h"
#include "common/eigen_utils.h"
#include "common/make_vector.h"
#include "common/minmax_element.h"
#include "common/pca_eigen.h"
#include "common/polynomial_utils.h"
#include "vehicle_scan_line.h"

namespace road_object_detection {

const auto child_contours = common::members(&HierarchicalContour::first);
const auto father_contours = common::members(&HierarchicalContour::second);

PedestrianClassifierDebug::PedestrianClassifierDebug(
    const common::CameraTransformation *const camera_transformation,
    common::node_base::ParameterInterface *const parameter_interface,
    DebugImages *debug_images)
    : PedestrianClassifier(camera_transformation, parameter_interface),
      ClassifierDebug(debug_images) {}

inline auto toPolygonPoint() {
  return [](const auto &p) { return PolygonPoint(p.x, p.y); };
}

inline auto cast2f() {
  return [](const auto &p) { return static_cast<cv::Point2f>(p); };
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
  polygon_points.reserve(contour.size() + 1);
  boost::transform(contour, std::back_inserter(polygon_points), toPolygonPoint());
  polygon_points.emplace_back(contour.front().x, contour.front().y);
  return polygon_points;
}

RoadObjects PedestrianClassifierDebug::classify(const Features &features) {
  offset_in_debug_img = cv::Point(0, 0);
  cont_count = 0;
  nr_boxes = 0;
  // for visualization of transformed contours
  const cv::Mat black(
      debug_images->getCameraImage()->size(), CV_8U, static_cast<uchar>(0));
  //  debug_images->addImagePatch(0, black);

  debug_images->addImagePatch(4, black);
  debug_images->addImagePatch(1, black);
  //  plotTemplate();
  auto ret = PedestrianClassifier::classify(features);

//  if (features.roi_) {
//    for (const auto &bp : features.roi_->boundary_) {
//      cv::circle(*debug_images->getCameraImage(),
//                 imagePointToCvPoint(camera_transformation_->transformGroundToImage(bp)),
//                 5,
//                 cv::Scalar(0, 0, 255),
//                 -1);
//    }
//  }

  return ret;
}

cv::Mat PedestrianClassifierDebug::binarizeImage(const cv::Mat &img_gray) const {
  const auto ret = PedestrianClassifier::binarizeImage(img_gray);

  double min, max;
  cv::minMaxLoc(ret, &min, &max);

  //  ROS_DEBUG("image_max is %f", max);

  cv::Mat dbg = 255 / max * ret;
  dbg.convertTo(dbg, CV_8U);

  // add image patch to debug_images
  debug_images->addImagePatch(2, dbg);

  return ret;
}

void PedestrianClassifierDebug::extractContours(const cv::Mat &img_bin,
                                                Contours &contours,
                                                Hierarchy &hierarchy) const {
  PedestrianClassifier::extractContours(img_bin, contours, hierarchy);

  //  // convert contours to cv::Mat and visualize
  //  cv::Mat debug_conts = cv::Mat::zeros(1, img_bin.size, img_bin.type());
  //  for (const auto &cont : contours) {
  //    for (const auto &pix : cont) {
  //      debug_conts.at<uchar>(pix.x, pix.y) = static_cast<uchar>(255);
  //    }
  //  }

  //  // add to image patches
  //  debug_images->addImagePatch(1, debug_conts);
}

HierarchicalContours PedestrianClassifierDebug::filterContours(const Contours &contours,
                                                               const Hierarchy &hierarchy,
                                                               const ROI &roi_) const {
  const auto ret = PedestrianClassifier::filterContours(contours, hierarchy, roi_);

  const auto children = common::make_vector(ret | child_contours);
  const auto fathers = common::make_vector(ret | father_contours);

  const auto roi_img = *debug_images->getImagePatch(0);
  cv::Mat out_childs(roi_img.size(), CV_8U, static_cast<uchar>(0));
  //  cv::Mat out_fathers(roi_img.size(), CV_8U, static_cast<uchar>(0));
  cv::drawContours(out_childs, children, -1, cv::Scalar(255, 255, 255), 3);
  cv::drawContours(out_childs, fathers, -1, cv::Scalar(255, 255, 255), 3);

  debug_images->addImagePatch(3, out_childs);
  //  debug_images->addImagePatch(2, out_fathers);

  nr_boxes = children.empty() ? 1 : children.size() + children.size() % 2;

  ROS_DEBUG("nr of boxes is %zu", nr_boxes);

  plot_box = cv::Rect(0,
                      0,
                      2 * debug_images->getCameraImage()->size().width / nr_boxes,
                      debug_images->getCameraImage()->size().height / 2);

  ROS_DEBUG("plot box has size (%d,%d)", plot_box.width, plot_box.height);

  return ret;
}

BoundingBox PedestrianClassifierDebug::refineBoundingBox(const VehiclePose &pose,
                                                         const ImagePatch &img_patch,
                                                         const Contour &cont,
                                                         const double half_box_height,
                                                         const double half_box_width) const {
  const auto ret = PedestrianClassifier::refineBoundingBox(
      pose, img_patch, cont, half_box_height, half_box_width);

  //  const std::vector<BoundingBox> bb = {ret};
  //  cv::drawContours(*debug_images->getImagePatch(3), bb, -1, cv::Scalar(255,
  //  255, 255), 3);

  return ret;
}

BoundingBox PedestrianClassifierDebug::extractBoundingBox(const VehiclePose &pose,
                                                          const ImagePatch &roi_patch,
                                                          const double half_bb_height,
                                                          const double half_bb_width) const {
  //  const auto center_in_img =
  //      camera_transformation_->transformVehicleToImage(VehiclePoint{pose.translation()});

  //  const auto center_on_ground = camera_transformation_->transformVehicleToImage(
  //      VehiclePoint{pose.translation().x(), pose.translation().y(), 0});

  //  cv::circle(*debug_images->getCameraImage(), toCV(center_on_ground), 5,
  //             cv::Scalar(0, 255, 255), -1);

  return PedestrianClassifier::extractBoundingBox(pose, roi_patch, half_bb_height, half_bb_width);
}

void PedestrianClassifierDebug::plotPolygon(const Polygon &polygon,
                                            cv::Mat &img,
                                            const cv::Scalar &color) const {

  //  const auto img_temp = *debug_images->getCameraImage();

  const auto target_height = static_cast<float>(img.size().height);

  // convert template to cv::point and visualize
  Contour2f template_as_cv;
  template_as_cv.reserve(polygon.outer().size());

  const auto toCVPoint = [&template_as_cv](const auto &p) {
    template_as_cv.emplace_back(p.x(), p.y());
  };

  for_each_point(polygon, toCVPoint);

  const auto toY = [](const auto &p) { return p.y; };
  const auto minmax_y = common::minmax_element(
      template_as_cv | boost::adaptors::transformed(std::cref(toY)));
  const auto temp_height = *minmax_y.second - *minmax_y.first;

  const float scale = target_height / temp_height;

  // draw contours
  const auto scaleContour = [&scale](const auto &p) {
    return static_cast<cv::Point>(p * scale);
  };

  const cv::Point img_center(img.size().width / 2, img.size().height / 2);

  Contours dbg_contours;
  dbg_contours.push_back(
      common::make_vector(template_as_cv | boost::adaptors::transformed(scaleContour) |
                          boost::adaptors::transformed(shift(-img_center))));

  cv::drawContours(img, dbg_contours, -1, color, 3);
}

double PedestrianClassifierDebug::computeScore(Contour2f contour) const {
  offset_in_debug_img = cv::Point((cont_count % (nr_boxes / 2)) * plot_box.width,
                                  (cont_count % 2) * plot_box.height);

  cont_count++;

  cv::Rect debug_rect =
      cv::Rect(offset_in_debug_img, plot_box.size()) &
      cv::Rect(cv::Point(0, 0), debug_images->getCameraImage()->size());

  cv::Mat current_img(plot_box.size(), CV_8UC3, static_cast<uchar>(0));

  // plot template
  plotPolygon(template_polygon_, current_img, cv::Scalar(255, 0, 0));

  // transform contour as in computeScore and plot
  // compute center of gravity
  const cv::Moments template_moments = cv::moments(contour);
  const cv::Point2f cog(
      static_cast<float>(template_moments.m10 / template_moments.m00),
      static_cast<float>(template_moments.m01 / template_moments.m00));
  //  const auto shift = [&boundary](const auto &p) { return p - boundary.tl();
  //  };

  // shift polygon in order to meet coordinate frame requirements
  //  boost::transform(contour, contour.begin(), shift2f(mean));
  const auto toY = [](const auto &p) { return p.y; };
  const auto minmax_y =
      common::minmax_element(contour | boost::adaptors::transformed(std::cref(toY)));

  const auto height = *minmax_y.second - *minmax_y.first;
  const auto contour_centered =
      common::make_vector(contour | boost::adaptors::transformed(shift2f(cog)) |
                          boost::adaptors::transformed(scale(height)));

  const auto conversion = [](const auto &p) {
    return cvPoint2fToImagePointExact(p);
  };

  const auto difference_angle = common::getAngle(
      common::getPrincipalComponent(common::make_eigen_vector(
          contour_centered | boost::adaptors::transformed(conversion))),
      common::getPrincipalComponent(common::make_eigen_vector(
          template_contour_ | boost::adaptors::transformed(conversion))));

  cv::Mat trafo = cv::getRotationMatrix2D(
      cv::Point2f{0, 0}, common::toDegree(difference_angle), 1);

  cv::transform(contour_centered, contour_centered, trafo);

  // construct polygon (scaled to one)
  Polygon polygon;
  assign_points(polygon, contourToPolygonPoints(contour_centered));

  plotPolygon(polygon, current_img, cv::Scalar(0, 0, 255));

  if (debug_rect.size().width > 1 && debug_rect.size().height > 1) {
    current_img.copyTo((*debug_images->getImagePatch(4))(debug_rect));
  }

  return PedestrianClassifier::computeScore(contour);
}

Contour2f PedestrianClassifierDebug::transformContourToObjectPlane(const Contour &contour_in_image,
                                                                   const BoundingBox &bounding_box,
                                                                   const double bb_height,
                                                                   const double bb_width) const {
  std::stringstream id;
  id << "id " << cont_count;

  const cv::Point center = boost::accumulate(contour_in_image, cv::Point(0, 0)) /
                           static_cast<int>(contour_in_image.size());

  cv::putText(*debug_images->getImagePatch(3), id.str(), center, 2, 1, cv::Vec3b(0, 0, 255));

  return PedestrianClassifier::transformContourToObjectPlane(
      contour_in_image, bounding_box, bb_height, bb_width);
}

}  // namespace road_object_detection
