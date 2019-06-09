#ifndef KITCAR_BRAIN_SRC_PERCEPTION_SRC_ROAD_WATCHER_CLASSIFIERS_CONTOUR_CLASSIFIER_H_
#define KITCAR_BRAIN_SRC_PERCEPTION_SRC_ROAD_WATCHER_CLASSIFIERS_CONTOUR_CLASSIFIER_H_
#include <common/macros.h>

THIRD_PARTY_HEADERS_BEGIN
#include <Eigen/Core>

#include <string>
#include <vector>

#include <opencv2/core.hpp>
#include <opencv2/shape/shape_distance.hpp>
THIRD_PARTY_HEADERS_END

#include "../classifier.h"
#include "common/camera_transformation.h"
#include "contour_classifier/contour_template.h"
#include "perception_types.h"

namespace road_object_detection {

namespace contour_classifier {

//!
//! \brief The ContourClassifier class classifies unknown object based on their
//! contours by comparing them with templates.
//!
//! This works for all kind of markings printed on the road e.g.
//!   - speed limit ( + relieved)
//!   - arrows
//!   - parking
//!
//! To teach in new contours set the flag [TODO] to <i> true </i> which will
//! dump out
//! all detections to a /tmp subfolder [FIXME implement!]
//!
//! Algorithm:
//!  - Extract contours in image patch
//!  - Create contour trees to represent hierarchies
//!  - Project contours onto ground plane
//!  - Discard contours that do not satisfy basic geometric constraints and
//! position
//!  - Classify each contour based on provided templates
//!
class ContourClassifier : public virtual Classifier {
 public:
  explicit ContourClassifier(ParameterInterface* parameter_interface,
                             const common::CameraTransformation* camera_transformation);

  virtual ~ContourClassifier() override = default;

  // Classifier interface
  RoadObjects classify(const Features& features) override;
  size_t getClassifierId() const final override;

  static ContourTemplates loadTemplates(const std::string& path);

 protected:
  struct Cluster {
    //! mean position of cluster elements in vehicle frame
    //! (dropped perpendicular foot point)
    double pos_along_middle_lane = 0.0;
    ContourTrees contour_trees;
  };

  using Clusters = std::vector<Cluster>;

  /**
   * Extracts hierarchical contour from image_patch.
   * \param image_patch the image patch.
   * \param contours [in|out] fount contours
   * \param hierarchy [in|out] corresponding hierarchy
   */
  void extractContours(const ImagePatch& image_patch,
                       CvContours* contours,
                       CvHierarchy* hierarchy) const;
  const common::CameraTransformation* get_camera_transformation();
  /**
   * Projects contours (image points) onto ground plane.
   * \param  features Containing birdsview_patch member that provides transform
   * \param  contours Contours in image space (pixels)
   * \return          projected contour (metric)
   */
  Contours3D projectContours(const Features& features, const CvContours& contours) const;

  RoadObjects classify(const Features& features,
                       const Contours3D& contours_3d,
                       const Contours3D& removed_contours_3d,
                       const CvHierarchy& hierarchy) const;

  virtual RoadObjects classify(const Features& features, const ContourTrees& contour_trees) const;

  //!
  //! \brief loadTemplates helper function to load templates from defined path
  //! \return false if templates could not be loaded, otherwise true
  //!
  bool loadTemplates();


  //!
  //! \brief filterContours removes all contours and entries in hierarchy, that
  //! touch features.image_patch borders
  //! \param features contains image_patch that provides border (size)
  //! \param contours [in|out] Contours that might touch the features.image_patch
  //! borders. These will be removed.
  //! \param hierarchy [in|out| Hierarchy corresponding to contours. This vector
  //! will be filtered according to contours.
  //!
  CvContours filterContoursTouchingImageBorders(const Features& features,
                                                CvContours* contours,
                                                CvHierarchy* hierarchy) const;

  //!
  //! \brief filterContours removes contours from contour_trees that do not
  //! satisfy criterea like
  //!   - center position within right lane
  //! filterContours operates in place
  //! \param contour_trees [in|out]
  //!
  // void filterContoursTouchingImageBorders(ContourTrees* contour_trees) const;

  ContourClassifier::Clusters clusterContourTrees(ContourTrees contours,
                                                  const common::DynamicPolynomial& middle_lane_polynomial) const;



  //! binarization of image patch
  cv::Mat binarizeAdaptiveThreshold(const cv::Mat& image_grey) const;

  void removeClustersContainingRemovedContours(ContourClassifier::Clusters& clusters,
                                               const Contours3D& removed_contours_3d,
                                               const common::DynamicPolynomial& middle_lane_polynomial,
                                               const double cluster_height) const;

  bool isTooSmall(const ContourTree& contour_tree, const double min_area) const;

  const ParameterInterface* const parameter_interface_;

  //! to project contour points from image to ground plane
  const common::CameraTransformation* const camera_transformation_;

  //! container for all known templates
  ContourTemplates contour_templates_;

 private:
  //! ROS param stuff
  static const ParameterString<double> LANE_WIDTH;
  static const ParameterString<double> MIN_SCORE_TH;
  static const ParameterString<double> MIN_CENTER_DISTANCE_TO_BOUNDARY;
  static const ParameterString<double> BINARIZATION_C_CONTOUR;
  static const ParameterString<double> BINARIZATION_SIGMA_CONTOUR;
  static const ParameterString<int> BINARIZATION_BLOCK_SIZE_CONTOUR;
  static const ParameterString<double> MAX_ANGLE_TO_MIDDLE_LANE;
  static const ParameterString<double> MIN_ZERO_MATCH_SCORE;

  static const ParameterString<double> MIN_CONTOUR_AREA;

  static const ParameterString<double> SAMPLING_RESOLUTION;
  static const ParameterString<double> CLUSTER_RANGE;

  static const ParameterString<bool> DUMP_POLYGONS;
  static const ParameterString<bool> TEACH_IN;
};

}  // namespace contour_classifier

}  // namespace road_object_detection

#endif  // KITCAR_BRAIN_SRC_PERCEPTION_SRC_ROAD_WATCHER_CLASSIFIERS_CONTOUR_CLASSIFIER_H_
