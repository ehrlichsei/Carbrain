#ifndef CONTOUR_TEMPLATE_H
#define CONTOUR_TEMPLATE_H
#include <common/macros.h>

THIRD_PARTY_HEADERS_BEGIN
#include <Eigen/Core>

#include <common/polynomial.h>
#include <perception_types.h>

#include <limits>
#include <string>
#include <vector>

#include <opencv2/core.hpp>
#include <opencv2/shape/shape_distance.hpp>
THIRD_PARTY_HEADERS_END

#include "../../road_objects/road_object.h"

namespace road_object_detection {

namespace contour_classifier {

using CvHierarchy = std::vector<cv::Vec4i>;

//! contour in image frame (pixels)
using CvContour = std::vector<cv::Point>;
using CvContours = std::vector<CvContour>;

//! contour in vehicle frame with z=0
// do not change this -> cv::moments requires float
using CvContour2D = std::vector<cv::Point2f>;
using CvContours2D = std::vector<CvContour2D>;

//! contour in vehicle frame
using Contour3D = WorldPoints;
using Contours3D = std::vector<Contour3D>;

struct ContourNode;
using ContourNodes = std::vector<ContourNode>;

class ContourTree;
using ContourTrees = std::vector<ContourTree>;

enum class ContourTemplateType : int {
  UNCATEGORIZED = 0,
  ARROW = 1,
  SPEED_LIMIT = 2,
};

// Node to construct a tree that can be traversed from root to leafs
struct ContourNode {
  //! OpenCV YAML read and write functions
  void write(cv::FileStorage *fs) const;
  void read(const cv::FileNode &node);

  CvContour2D contour;
  //! the position of each contour relative to the parent
  Eigen::Vector2d relative_position;
};

//!
//! \brief The ContourTree struct is a ContourNode with additional fields to
//! serve as the root of the contour tree
//!
class ContourTree {
 public:
  double match(const ContourTree &other) const;

  void write(cv::FileStorage *fs) const;
  void read(const cv::FileNode &node);

  Eigen::Affine3d vehicle_T_contour;
  double distance_to_middle_lane_polynomial;
  ContourNodes children;
  CvContour2D contour;

  void centerAndAlign();

  CvContour2D sampledOuterContour() const;

  void transform(const Eigen::Affine2d &transformation);

  void normalizeContour();
  double scaling = 1;

  static double sampling_resolution;

  double angle_to_middle_lane = 0.0;

  static bool dump_polygons;
};

class ContourTemplate {
 public:
  explicit ContourTemplate(const CvContours2D &contours,
                           const CvHierarchy &hierarchy,
                           const common::DynamicPolynomial &middle_lane_polynomial);

  explicit ContourTemplate(const cv::FileStorage &fs);

  explicit ContourTemplate(const ContourTrees &contour_trees);

  ContourTemplate() = delete;

  virtual ~ContourTemplate() = default;

  //!
  //! \brief match Calculates the similarity between own and given contour tree
  //!
  //! The contours are represented as polygons (with holes) and
  //! intersection over union (IOU) is calculated.
  //!
  //! If more than one contour tree is available, only the first will be
  //! considered as the rules mention speed limits that could have multiple
  //! outer contours. Consequently, the second will always be the zero digit.
  //!
  //! \param  other_trees the other contour tree to match.
  //! \return similarity [0,1.0] where 1.0 is a perfect match
  //!
  double match(ContourTrees other_trees, const double min_zero_match_score = 0.0) const;

  //!
  //! \brief asRoadObject Populates a road object (derived class) with contour
  //! template data.
  //!
  virtual RoadObjectPtr asRoadObject(const ros::Time &timestamp,
                                     double score,
                                     const ContourTrees &contour_trees) const = 0;

  static ContourTrees cvContoursToContourTrees(const CvContours2D &contours,
                                               const CvHierarchy &hierarchy,
                                               const common::DynamicPolynomial &middle_lane_polynomial);

  //!
  //! \brief write writes the ContourTemplate to file
  //! \param filename the file to write to.
  //!
  void write(const std::string &filename);
  void write(cv::FileStorage *fs) const;
  static ContourTrees read(const cv::FileStorage &fs);

  const ContourTrees &getContourTrees() const { return contour_trees_; }

  void normalizeContours();

 protected:
  //!
  //! \brief writeAdditionalFields function to be overriden in derived class
  //! to save specific data
  //! \param file_storage the storage to write to.
  //!
  virtual void writeAdditionalFields(cv::FileStorage *file_storage) const = 0;

  virtual int getTemplateId() const = 0;

  VehiclePoints getConvexHull(const ContourTrees &contour_trees) const;

  VehiclePoints getEnclosingRectangle(const ContourTrees &contour_trees) const;

 private:
  ContourTrees contour_trees_;
  void sanitizeContours();
};

using ContourTemplatePtr = std::unique_ptr<ContourTemplate>;
using ContourTemplates = std::vector<ContourTemplatePtr>;

//!
//! \brief The ContourTemplateFactory class creates ContourTemplates from YAML
// files.
//!
class ContourTemplateFactory {
 public:
  static ContourTemplatePtr createFromFile(const std::string &filename);
};

Eigen::Vector2d moments_to_center_point(const cv::Moments &m);

}  // namespace contour_classifier

}  // namespace road_object_detection

#endif  // CONTOUR_TEMPLATE_H
