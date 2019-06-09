#ifndef CONTOUR_CLASSIFIER_DEBUG_H
#define CONTOUR_CLASSIFIER_DEBUG_H

#include "../../classifiers/contour_classifier.h"
#include "../classifier_debug.h"

namespace road_object_detection {

namespace contour_classifier {

class ContourClassifierDebug : public virtual ContourClassifier, public virtual ClassifierDebug {
 public:
  ContourClassifierDebug(ParameterInterface *parameter_interface,
                         const common::CameraTransformation *camera_transformation,
                         DebugImages *debug_images);
  virtual ~ContourClassifierDebug() override = default;

  // Classifier interface
  RoadObjects classify(const Features &features) override;

  // ContourClassifier interface
 protected:
  RoadObjects classify(const Features &features, const ContourTrees &contour_trees) const override;

 private:
  //! teach in stuff
  mutable bool tmp_folder_created_;
  mutable bool tmp_folder_creation_failed_;
  mutable std::string tmp_folder_;
  private:
  cv::Mat drawFrame(const cv::Mat &image_grey);
//  cv::Mat drawContour(const cv::Mat &image_grey) const;
//  CvContours2D contours_2d;

  //!
  //! \brief createTemplateTmpDir tries to create a unique folder in /tmp
  //! \return true if tmp folder is valid
  //!
  bool createTemplateTmpDir() const;



  static const ParameterString<bool> TEACH_IN;
};


}  // namespace contour_classifier

} //namespace road_object_detection

#endif  // CONTOUR_CLASSIFIER_DEBUG_H
