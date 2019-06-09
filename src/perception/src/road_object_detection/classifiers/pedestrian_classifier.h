#ifndef PEDESTRIAN_CLASSIFIER_H
#define PEDESTRIAN_CLASSIFIER_H

#include "../classifier.h"

THIRD_PARTY_HEADERS_BEGIN
#include <opencv2/core.hpp>

#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/polygon.hpp>
THIRD_PARTY_HEADERS_END
namespace road_object_detection {

using namespace boost::geometry;
using PolygonPoint = model::d2::point_xy<float>;
using PolygonPoints = std::vector<PolygonPoint>;
using Polygon = model::polygon<PolygonPoint, false>;

using Contour = std::vector<cv::Point>;
using Contours = std::vector<Contour>;

using Contour2f = std::vector<cv::Point2f>;
using Contours2f = std::vector<Contour2f>;

using BoundingBox = std::vector<cv::Point>;
using BoundingBoxes = std::vector<BoundingBox>;

using Hierarchy = std::vector<cv::Vec4i>;

using ContoursWithHierarchy = std::vector<std::pair<Contour, cv::Vec4i>>;
using HierarchicalContour = std::pair<Contour, BoundingBox>;
using HierarchicalContours = std::vector<HierarchicalContour>;

class PedestrianClassifier : public virtual Classifier {
 public:
  PedestrianClassifier(const common::CameraTransformation *const camera_transformation,
                       ParameterInterface *const parameter_interface);

  PedestrianClassifier() = delete;

  virtual ~PedestrianClassifier() override = default;

  virtual RoadObjects classify(const Features &features) override;

  virtual size_t getClassifierId() const final override;

  static const std::string NAMESPACE;

 protected:
  virtual cv::Mat binarizeImage(const cv::Mat &img_gray) const;

  virtual void extractContours(const cv::Mat &img_bin, Contours &contours, Hierarchy &hierarchy) const;

  virtual HierarchicalContours filterContours(const Contours &contours,
                                              const Hierarchy &hierarchy,
                                              const ROI &roi_) const;

  void readInTemplate();

  virtual BoundingBox extractBoundingBox(const VehiclePose &pose,
                                         const ImagePatch &roi_patch,
                                         const double half_bb_height,
                                         const double half_bb_width) const;

  virtual BoundingBox refineBoundingBox(const VehiclePose &pose,
                                        const ImagePatch &img_patch,
                                        const Contour &cont,
                                        const double half_box_height,
                                        const double half_box_width) const;

  virtual Contour2f transformContourToObjectPlane(const Contour &contour_in_image,
                                                  const BoundingBox &bounding_box,
                                                  const double bb_height,
                                                  const double bb_width) const;

  virtual double computeScore(Contour2f contour) const;

  VehiclePoints computeHullPolygon(const BoundingBox &bounding_box,
                                   const cv::Point &roi_tl) const;

  VehiclePoint getContourCenter(const Contour &cont,
                                const ImagePatch &img_patch,
                                const double half_box_height) const;

  const common::CameraTransformation *const camera_transformation_;
  const ParameterInterface *const parameters_ptr_;

  static const ParameterString<double> C_BINARIZATION;
  static const ParameterString<double> SIGMA_BINARIZATION;
  static const ParameterString<int> BLOCK_SIZE_BINARIZATION;
  static const ParameterString<int> MIN_CONTOUR_SIZE;

  static const ParameterString<double> SIGMA_GAUSS;
  static const ParameterString<int> BLOCK_SIZE_GAUSS;

  static const ParameterString<double> MIN_SCORE;
  static const ParameterString<double> BOX_DEPTH;

  static const ParameterString<double> BB_WIDTH;
  static const ParameterString<double> BB_HEIGHT;

  Polygon template_polygon_;
  Contour2f template_contour_;
};

}  // namespace road_object_detection

#endif  // PEDESTRIAN_CLASSIFIER_H
