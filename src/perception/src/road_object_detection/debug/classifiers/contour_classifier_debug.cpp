#include "contour_classifier_debug.h"

#include "../../classifiers/contour_classifier/unclassified_contour_template.h"

namespace road_object_detection {


namespace contour_classifier {


const ParameterString<bool> ContourClassifierDebug::TEACH_IN(
    "contour_classifier/teach_in");

ContourClassifierDebug::ContourClassifierDebug(ParameterInterface *parameter_interface,
                                               const common::CameraTransformation *camera_transformation,
                                               DebugImages *debug_images)
    : ContourClassifier(parameter_interface, camera_transformation),
      ClassifierDebug(debug_images),
      tmp_folder_created_(false),
      tmp_folder_creation_failed_(false) {

  parameter_interface->registerParam(TEACH_IN);

  if (parameter_interface->getParam(TEACH_IN)) {
    createTemplateTmpDir();
  }
}

RoadObjects ContourClassifierDebug::classify(const Features &features) {
  RoadObjects road_objects = ContourClassifier::classify(features);


    debug_images->addImagePatch(
    getClassifierId(), drawFrame(features.image_patch.image));
//    debug_images->addImagePatch(
//    getClassifierId(), drawContour(features.image_patch.image));
  return road_objects;
}

cv::Mat ContourClassifierDebug::drawFrame(const cv::Mat &image_grey) {
  cv::Mat binarized = binarizeAdaptiveThreshold(image_grey);
  cv::rectangle(binarized,
                cv::Point2i(0, 0),
                cv::Point2i(image_grey.size().width - 1, image_grey.size().height - 1),
                CV_RGB(150, 150, 150),
                1,
                8,
                0);
  return binarized;
}
//cv::Mat ContourClassifierDebug::drawContour(const cv::Mat &image_grey) const{
//  CvContours contours;
//  CvHierarchy hierarchy;
//  cv::Mat binarized = binarizeAdaptiveThreshold(image_grey);
//  cv::findContours(binarized, contours, hierarchy,cv::RETR_TREE, cv::CHAIN_APPROX_TC89_KCOS);
//  for (uint i = 0; i < contours.size(); i++) {
//      for (uint j = 0;  j< contours.at(i).size(); j++) {
//      cv::circle(binarized,
//                static_cast<cv::Point2i>(contours.at(i).at(j)),
//                 3,
//                 cv::Scalar(255, 0, 0),
//                 2);
//      }
//    }
//  return binarized;

//}

RoadObjects ContourClassifierDebug::classify(const Features &features,
                                             const ContourTrees &contour_trees) const {
  auto road_objects = ContourClassifier::classify(features, contour_trees);
  //  debug_images->addImagePatch(getClassifierId(),
  //  drawContour(contour_trees));

  //  // check if teach in is activated
  if (parameter_interface_->getParam(TEACH_IN) && !tmp_folder_creation_failed_) {
    if (createTemplateTmpDir()) {  // if
      static int i = 0;

      UnclassifiedContourTemplate tmpl_debug(contour_trees);
      tmpl_debug.write(tmp_folder_ + "/" + std::to_string(++i) + ".yaml");
    }
  }

  return road_objects;
}



bool ContourClassifierDebug::createTemplateTmpDir() const {
  // only try to create tmp folder once
  if (tmp_folder_creation_failed_) {
    return false;
  }

  if (tmp_folder_created_) {
    return true;
  }

  char tmppath[] = "/tmp/contour_templates.XXXXXX";

  char *tmpdir = mkdtemp(tmppath);
  if (tmpdir == nullptr) {
    tmp_folder_creation_failed_ = true;

    ROS_ERROR("Could not create temporary directory!");
    return false;
  }

  tmp_folder_created_ = true;
  tmp_folder_ = std::string(tmpdir);

  ROS_INFO_STREAM("Storing detected contours in " << tmp_folder_);
  return true;
}

}  // namespace contour_classifier


} //namespace road_object_detection
