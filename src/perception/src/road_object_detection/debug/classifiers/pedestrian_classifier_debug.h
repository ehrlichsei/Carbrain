#ifndef PEDESTRIAN_CLASSIFIER_DEBUG_H
#define PEDESTRIAN_CLASSIFIER_DEBUG_H

#include "../../classifiers/pedestrian_classifier.h"
#include "../classifier_debug.h"

THIRD_PARTY_HEADERS_BEGIN

THIRD_PARTY_HEADERS_END

namespace road_object_detection {

class PedestrianClassifierDebug : public virtual PedestrianClassifier,
                                  public virtual ClassifierDebug {
 public:
  PedestrianClassifierDebug(const common::CameraTransformation *const camera_transformation,
                            common::ParameterInterface *const parameter_interface,
                            DebugImages *debug_images);

  virtual ~PedestrianClassifierDebug() override = default;

  virtual RoadObjects classify(const Features &features) override;

 private:
  virtual cv::Mat binarizeImage(const cv::Mat &img_gray) const override;

  virtual void extractContours(const cv::Mat &img_bin,
                               Contours &contours,
                               Hierarchy &hierarchy) const override;

  virtual HierarchicalContours filterContours(const Contours &contours,
                                              const Hierarchy &hierarchy,
                                              const ROI &roi_) const override;

  virtual BoundingBox refineBoundingBox(const VehiclePose &pose,
                                        const ImagePatch &img_patch,
                                        const Contour &cont,
                                        const double half_box_height,
                                        const double half_box_width) const override;

  virtual BoundingBox extractBoundingBox(const VehiclePose &pose,
                                         const ImagePatch &roi_patch,
                                         const double half_bb_height,
                                         const double half_bb_width) const override;

  void plotPolygon(const Polygon &polygon, cv::Mat &img, const cv::Scalar &color) const;

  virtual double computeScore(Contour2f contour) const override;

  virtual Contour2f transformContourToObjectPlane(const Contour &contour_in_image,
                                                  const BoundingBox &bounding_box,
                                                  const double bb_height,
                                                  const double bb_width) const override;

  mutable cv::Rect plot_box;
  mutable cv::Point offset_in_debug_img = cv::Point(0, 0);
  mutable std::size_t cont_count = 0;
  mutable std::size_t nr_boxes = 0;
};

}  // namespace road_object_detection

#endif  // PEDESTRIAN_CLASSIFIER_DEBUG_H
