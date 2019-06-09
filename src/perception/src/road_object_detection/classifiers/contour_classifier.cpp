#include "contour_classifier.h"

THIRD_PARTY_HEADERS_BEGIN
#include <ros/console.h>
#include <ros/package.h>
#include <algorithm>
#include <boost/algorithm/cxx11/any_of.hpp>
#include <boost/filesystem.hpp>
#include <boost/range/adaptors.hpp>
#include <boost/range/algorithm.hpp>
#include <boost/range/algorithm_ext.hpp>
#include <iterator>
#include <numeric>
#include <opencv2/imgproc.hpp>
THIRD_PARTY_HEADERS_END

#include "../../utils/foot_finder.h"
#include "common/best_score.h"
#include "common/console_colors.h"
#include "common/contains.h"
#include "common/eigen_utils.h"
#include "common/indirect_sort.h"
#include "common/lift.h"
#include "common/move_range.h"
#include "common/polynomial_utils.h"
#include "opencv_eigen_conversions.h"

namespace fs = ::boost::filesystem;

namespace road_object_detection {

namespace contour_classifier {

const ParameterString<double> ContourClassifier::LANE_WIDTH("lane_width");
const ParameterString<double> ContourClassifier::MIN_SCORE_TH(
    "contour_classifier/min_score_th");
const ParameterString<double> ContourClassifier::MIN_CENTER_DISTANCE_TO_BOUNDARY(
    "contour_classifier/min_center_distance_to_boundary");
const ParameterString<double> ContourClassifier::BINARIZATION_C_CONTOUR(
    "contour_classifier/binarization_c_contour");
const ParameterString<double> ContourClassifier::BINARIZATION_SIGMA_CONTOUR(
    "contour_classifier/binarization_sigma_contour");
const ParameterString<int> ContourClassifier::BINARIZATION_BLOCK_SIZE_CONTOUR(
    "contour_classifier/binarization_block_size_contour");
const ParameterString<double> ContourClassifier::SAMPLING_RESOLUTION(
    "contour_classifier/sampling_resolution");
const ParameterString<double> ContourClassifier::MAX_ANGLE_TO_MIDDLE_LANE(
    "contour_classifier/max_angle_to_middle_lane");
const ParameterString<bool> ContourClassifier::DUMP_POLYGONS(
    "contour_classifier/dump_polygons");
const ParameterString<double> ContourClassifier::MIN_ZERO_MATCH_SCORE(
    "contour_classifier/min_zero_match_score");
const ParameterString<double> ContourClassifier::CLUSTER_RANGE(
    "contour_classifier/cluster_range");
const ParameterString<double> ContourClassifier::MIN_CONTOUR_AREA(
    "contour_classifier/min_contour_area");
const ParameterString<bool> ContourClassifier::TEACH_IN(
    "contour_classifier/teach_in");

ContourClassifier::ContourClassifier(ParameterInterface *parameter_interface,
                                     const common::CameraTransformation *camera_transformation)
    : parameter_interface_(parameter_interface),
      camera_transformation_(camera_transformation) {
  parameter_interface->registerParam(LANE_WIDTH);
  parameter_interface->registerParam(MIN_SCORE_TH);
  parameter_interface->registerParam(MIN_CENTER_DISTANCE_TO_BOUNDARY);
  parameter_interface->registerParam(BINARIZATION_C_CONTOUR);
  parameter_interface->registerParam(BINARIZATION_SIGMA_CONTOUR);
  parameter_interface->registerParam(BINARIZATION_BLOCK_SIZE_CONTOUR);

  parameter_interface->registerParam(DUMP_POLYGONS);
  parameter_interface->registerParam(CLUSTER_RANGE);

  parameter_interface->registerParam(MAX_ANGLE_TO_MIDDLE_LANE);
  parameter_interface->registerParam(MIN_ZERO_MATCH_SCORE);

  parameter_interface->registerParam(MIN_CONTOUR_AREA);
  parameter_interface->registerParam(TEACH_IN);

  parameter_interface->registerParam(SAMPLING_RESOLUTION);
  ContourTree::sampling_resolution = parameter_interface->getParam(SAMPLING_RESOLUTION);

  // load templates from predefined storage location
  if (!loadTemplates()) {
    ROS_ERROR("Could not load templates!");
  }
}

inline bool isInsideRightLane(double y_position, double lane_width, double padding) {
  assert(padding >= 0.0);
  assert(lane_width > 0.0);
  const bool is_inside =
      ((y_position > padding) && (y_position < (lane_width - padding)));

  return is_inside;
}

CvContour2D to2D(const Contour3D &contour_3d) {
  CvContour2D contour_2d;
  contour_2d.reserve(contour_3d.size());

  // 3d -> 2d by discarding z value
  boost::transform(
      contour_3d, std::back_inserter(contour_2d), [](const auto &point_3d) -> cv::Point2d {
        return {point_3d.x(), point_3d.y()};
      });

  return contour_2d;
}

CvContours2D to2D(const Contours3D &contours_3d) {
  CvContours2D contours_2d;
  contours_2d.reserve(contours_3d.size());
  boost::transform(contours_3d, std::back_inserter(contours_2d), LIFT(to2D));
  return contours_2d;
}


inline bool isInvalidContour(const Contour3D &contour_3d) {
  auto contour_2d = to2D(contour_3d);
  auto m = cv::moments(contour_2d);

  return m.m00 == 0.0;
}


inline auto centerPoint(const Contour3D &contour_3d) {
  auto contour_2d = to2D(contour_3d);
  auto m = cv::moments(contour_2d);

  assert(m.m00 != 0.0);

  auto c = moments_to_center_point(m);

  return c;
}


inline bool isContourOutsideRightLane(const Contour3D &contour_3d,
                                      const Features &features,
                                      const double lane_width_fallback,
                                      const double min_center_distance_to_boundary) {
  const auto c = centerPoint(contour_3d);

  const Eigen::Vector2d vehicle_P_lf =
      utils::findLotfusspunkt(features.middle_lane_polynomial, c);

  // rotation in local coordinate frame  // rotation in local coordinate frame
  const auto yaw =
      common::tangentAngle(features.middle_lane_polynomial, vehicle_P_lf.x());

  Eigen::Affine2d vehicle_T_contour_lf;
  vehicle_T_contour_lf.translation() = vehicle_P_lf;
  vehicle_T_contour_lf.linear() = Eigen::Rotation2Dd(yaw).toRotationMatrix();

  Eigen::Vector2d contour_lf_P_contour = c - vehicle_P_lf;
  contour_lf_P_contour = vehicle_T_contour_lf.linear().inverse() * contour_lf_P_contour;

  double lane_width;
  if (features.points_right.empty()) {
    // fallback solution
    lane_width = lane_width_fallback;
  } else {

    Eigen::Affine3d contour_lf_T_vehicle = to3D(vehicle_T_contour_lf.inverse());

    auto &points_right = features.points_right;

    auto point_idx = std::distance(
        points_right.begin(),
        common::min_score(points_right, [&contour_lf_T_vehicle](const VehiclePoint &vehicle_point) -> double {
          return std::abs((contour_lf_T_vehicle * vehicle_point)[0]);
        }));

    lane_width = std::abs((contour_lf_T_vehicle * features.points_right[point_idx])[1]);

    ROS_DEBUG("current lane width: %fm", lane_width);
  }

  ROS_DEBUG("Distance to middle lane: %f", -contour_lf_P_contour[1]);

  return !isInsideRightLane(-contour_lf_P_contour[1], lane_width, min_center_distance_to_boundary);
}

bool ContourClassifier::isTooSmall(const ContourTree &contour_tree, const double min_area) const {
  double area = cv::contourArea(contour_tree.contour);
  return area < min_area;
}

RoadObjects ContourClassifier::classify(const Features &features) {
  CvContours contours;
  CvHierarchy hierarchy;
  extractContours(features.image_patch, &contours, &hierarchy);

  auto removed_outer_contours =
      filterContoursTouchingImageBorders(features, &contours, &hierarchy);

  if (contours.empty()) {
    return {};
  }

  Contours3D removed_outer_contours_3d = projectContours(features, removed_outer_contours);

  // update params
  ContourTree::dump_polygons = parameter_interface_->getParam(DUMP_POLYGONS);

  const double min_center_distance_to_boundary =
      parameter_interface_->getParam(MIN_CENTER_DISTANCE_TO_BOUNDARY);
  const double lane_width = parameter_interface_->getParam(LANE_WIDTH);

  // Some contours in removed_outer_contours might be valid objects.
  // It might happen, that e.g. the zero of 50 is touching the image border, so
  // it would be filtered out. In this case, it does not make sense to match the
  // rest.

  // Therefore, remove all contours from removed_outer_contours, that are
  // definitely outliers
  boost::remove_erase_if(removed_outer_contours_3d, isInvalidContour);

  boost::remove_erase_if(
      removed_outer_contours_3d,
      boost::bind(isContourOutsideRightLane, _1, boost::cref(features), lane_width, min_center_distance_to_boundary));

  ROS_DEBUG_COND(!removed_outer_contours_3d.empty(),
                 "Found %lu valid removed contours",
                 removed_outer_contours_3d.size());

  // the removed_outer_contours_3d that are left are valid contours


  ROS_DEBUG_COND(!contours.empty(), "Found %lu contours.", contours.size());

  Contours3D contours_3d = projectContours(features, contours);

  return classify(features, contours_3d, removed_outer_contours_3d, hierarchy);
}

size_t ContourClassifier::getClassifierId() const {
  return typeid(ContourClassifier).hash_code();
}

void ContourClassifier::extractContours(const ImagePatch &image_patch,
                                        CvContours *contours,
                                        CvHierarchy *hierarchy) const {
  cv::Mat binarized = binarizeAdaptiveThreshold(image_patch.image);
  cv::findContours(binarized, *contours, *hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_TC89_KCOS);

  //  for (auto &contour : *contours) {
  //    cv::approxPolyDP(contour, contour, 0.1, true);
  //  }

  for (CvContour &contour : *contours) {
    contour.push_back(contour.front());
  }
}

cv::Mat ContourClassifier::binarizeAdaptiveThreshold(const cv::Mat &image_grey) const {

  int blocksize = parameter_interface_->getParam(BINARIZATION_BLOCK_SIZE_CONTOUR);
  double sigma = parameter_interface_->getParam(BINARIZATION_SIGMA_CONTOUR);
  double c = parameter_interface_->getParam(BINARIZATION_C_CONTOUR);
  double stddev = 0.0;
  cv::Mat image_binary;

  if (sigma != 0.0) {
    cv::Scalar mean_scalar, stddev_scalar;
    cv::meanStdDev(image_grey, mean_scalar, stddev_scalar);
    stddev = stddev_scalar.val[0];
  }
  cv::adaptiveThreshold(image_grey,
                        image_binary,
                        255,
                        cv::ADAPTIVE_THRESH_MEAN_C,
                        cv::THRESH_BINARY,
                        blocksize,
                        -sigma * stddev - c);
  return image_binary;
}

Contours3D ContourClassifier::projectContours(const Features &features,
                                              const CvContours &contours) const {
  Contours3D contours_3d;
  contours_3d.reserve(contours.size());

  for (const CvContour &contour : contours) {
    Contour3D contour_3d;
    contour_3d.reserve(contour.size());

    for (const cv::Point &contour_point : contour) {
      auto ground_point = camera_transformation_->transformImageToGround(
          toEigen(contour_point) + toEigen(features.image_patch.position));

      contour_3d.push_back(ground_point);
    }

    contours_3d.push_back(contour_3d);
  }

  return contours_3d;
}


/*
void ContourClassifier::filterContoursTouchingImageBorders(ContourTrees *contour_trees) const {
  // remove all contour nodes that appear on the reverse lane
  const double lane_width = parameter_interface_->getParam(LANE_WIDTH);
  const double padding = parameter_interface_->getParam(MIN_CENTER_DISTANCE_TO_BOUNDARY);
  const double max_angle_to_middle_lane =
      parameter_interface_->getParam(MAX_ANGLE_TO_MIDDLE_LANE);

  // FIXME calculate lane width based on features.right_points

  auto n_trees_in = contour_trees->size();

  boost::remove_erase_if(
      *contour_trees,
      [&lane_width, &padding](const ContourTree &contour_tree) {
        return !isInsideRightLane(
                   contour_tree.distance_to_middle_lane_polynomial, lane_width, padding);
      });

  ROS_DEBUG_STREAM("Filtered "
                   << n_trees_in - contour_trees->size()
                   << " contour trees that as they are not inside right lane.");

  n_trees_in = contour_trees->size();

  for (const auto &contour_tree : *contour_trees) {
    ROS_DEBUG_STREAM("angle to middle lane: " << contour_tree.angle_to_middle_lane);
  }

  boost::remove_erase_if(*contour_trees,
                         [&max_angle_to_middle_lane](const ContourTree &contour_tree) {
                           return std::abs(contour_tree.angle_to_middle_lane) >
                                  max_angle_to_middle_lane;
                         });


  ROS_DEBUG_STREAM(
      "Filtered "
      << n_trees_in - contour_trees->size()
      << " contour trees as they are not pointing into lane direction.");
}
*/


inline double projectTreeToMiddleLane(const ContourTree &contour,
                                      const common::DynamicPolynomial &middle_lane_polynomial) {
  const auto &position = contour.vehicle_T_contour.translation();
  return utils::findLotfusspunktX(middle_lane_polynomial, ::to2D(position));
}

inline double projectContourToMiddleLane(const Contour3D &contour,
                                         const common::DynamicPolynomial &middle_lane_polynomial) {
  return utils::findLotfusspunktX(middle_lane_polynomial, centerPoint(contour));
}


ContourClassifier::Clusters ContourClassifier::clusterContourTrees(
    ContourTrees contours, const common::DynamicPolynomial &middle_lane_polynomial) const {
  Clusters clusters;

  std::vector<double> pos_along_polynomial;

  std::vector<size_t> sorted_indices = common::indirect_sort(
      contours,
      boost::bind(projectTreeToMiddleLane, _1, boost::cref(middle_lane_polynomial)),
      pos_along_polynomial);

  const double cluster_range = parameter_interface_->getParam(CLUSTER_RANGE);

  for (auto idx_it = sorted_indices.begin(); idx_it < sorted_indices.end(); ++idx_it) {
    const auto isOutsideClusterRange =
        [&idx_it, &pos_along_polynomial, cluster_range](const auto &pos_idx) {
          return std::abs(pos_along_polynomial[*idx_it] -
                          pos_along_polynomial[pos_idx]) > cluster_range;
        };
    // build a cluster
    auto last_it = std::find_if(idx_it, sorted_indices.end(), isOutsideClusterRange)--;

    {
      Cluster cluster;
      for (auto cluster_idx_it = idx_it;
           cluster_idx_it < last_it && cluster_idx_it < sorted_indices.end();
           ++cluster_idx_it) {
        cluster.pos_along_middle_lane += pos_along_polynomial.at(*cluster_idx_it);
        cluster.contour_trees.push_back(std::move(contours.at(*cluster_idx_it)));
      }
      cluster.pos_along_middle_lane /= cluster.contour_trees.size();
      clusters.push_back(std::move(cluster));
    }

    idx_it = last_it;
  }

  ROS_DEBUG_STREAM(COLOR_LIGHT_GREY << "Created " << clusters.size()
                                    << " clusters." << COLOR_DEBUG);

  return clusters;
}



RoadObjects ContourClassifier::classify(const Features &features,
                                        const ContourTrees &contour_trees) const {
  RoadObjects road_objects;

  const double min_score_th = parameter_interface_->getParam(MIN_SCORE_TH);
  const double min_zero_match_score = parameter_interface_->getParam(MIN_ZERO_MATCH_SCORE);

  for (const ContourTemplatePtr &tmpl : contour_templates_) {
    double score = tmpl->match(contour_trees, min_zero_match_score);

    if (score > min_score_th) {

      road_objects.push_back(tmpl->asRoadObject(features.timestamp, score, contour_trees));
    }
  }
  return road_objects;
}



void ContourClassifier::removeClustersContainingRemovedContours(
    ContourClassifier::Clusters &clusters,
    const Contours3D &removed_contours_3d,
    const common::DynamicPolynomial &middle_lane_polynomial,
    const double cluster_height) const {
  std::vector<double> pos_along_middle_lane;
  pos_along_middle_lane.reserve(clusters.size());

  boost::transform(
      removed_contours_3d,
      std::back_inserter(pos_along_middle_lane),
      boost::bind(projectContourToMiddleLane, _1, boost::cref(middle_lane_polynomial)));

  auto containsRemovedContour = [&](const ContourClassifier::Cluster &cluster) -> bool {
    return boost::algorithm::any_of(
        pos_along_middle_lane, [&cluster_height, &cluster](double x) -> bool {
          return std::abs(x - cluster.pos_along_middle_lane) < cluster_height;
        });
  };

  size_t n = clusters.size();
  boost::remove_erase_if(clusters, containsRemovedContour);

  size_t n_rem = n - clusters.size();
  ROS_DEBUG_COND(
      n_rem, "Removed %lu clusters as they contains removed contours!", n_rem);
}

RoadObjects ContourClassifier::classify(const Features &features,
                                        const Contours3D &contours_3d,
                                        const Contours3D &removed_contours_3d,
                                        const CvHierarchy &hierarchy) const {
  Clusters clusters;

  {
    CvContours2D contours_2d = to2D(contours_3d);
    ContourTrees contour_trees = ContourTemplate::cvContoursToContourTrees(
        contours_2d, hierarchy, features.middle_lane_polynomial);

    const double min_contour_area = parameter_interface_->getParam(MIN_CONTOUR_AREA);
    size_t n = contour_trees.size();

    boost::remove_erase_if(
        contour_trees,
        boost::bind(&ContourClassifier::isTooSmall, this, _1, min_contour_area));

    if (!parameter_interface_->getParam(TEACH_IN)) {
      for (ContourTree &contour_tree : contour_trees) {
        contour_tree.normalizeContour();
      }
    }

    const double lane_width = parameter_interface_->getParam(LANE_WIDTH);
    const double min_center_distance_to_boundary =
        parameter_interface_->getParam(MIN_CENTER_DISTANCE_TO_BOUNDARY);

    boost::remove_erase_if(contour_trees, [&](const ContourTree &contour_tree) {
      return !isInsideRightLane(contour_tree.distance_to_middle_lane_polynomial,
                                lane_width,
                                min_center_distance_to_boundary);
    });

    size_t m = n - contour_trees.size();
    ROS_DEBUG_COND(m, "Removed %lu too small contours.", m);

    ROS_DEBUG_STREAM("Individual contours: " << contour_trees.size());

    clusters = clusterContourTrees(std::move(contour_trees), features.middle_lane_polynomial);

    const double cluster_height = parameter_interface_->getParam(CLUSTER_RANGE);
    removeClustersContainingRemovedContours(
        clusters, removed_contours_3d, features.middle_lane_polynomial, cluster_height);
  }

  RoadObjects road_objects;

  for (const auto &cluster : clusters) {
    boost::push_back(road_objects,
                     common::move_range(classify(features, cluster.contour_trees)));
  }

  return road_objects;
}

ContourTemplates ContourClassifier::loadTemplates(const std::string &path) {
  fs::path package_path(path);
  if (!fs::exists(package_path)) {
    ROS_WARN_STREAM("Path " << package_path.string() << " does not exist!");
    return {};
  }

  ContourTemplates contour_templates;
  for (fs::directory_iterator di(package_path); di != fs::directory_iterator(); di++) {
    // search directory for the highest used file index
    if (fs::exists(*di) && fs::is_regular_file(*di)) {
      ROS_DEBUG_STREAM("Reading template file " << di->path().string());
      auto contour_template =
          ContourTemplateFactory::createFromFile(di->path().string());
      if (contour_template) {
        contour_templates.push_back(std::move(contour_template));
      }
    }
  }
  return contour_templates;
}

bool ContourClassifier::loadTemplates() {
  const auto path =
      ros::package::getPath("perception") + "/data/contour_templates/";

  contour_templates_ = loadTemplates(path);

  ROS_WARN_COND(contour_templates_.empty(), "No templates loaded!");
  ROS_INFO("Loaded %i templates", static_cast<int>(contour_templates_.size()));

  return !contour_templates_.empty();
}

bool contourTouchesBorderMin(const CvContour &contour) {
  return std::any_of(contour.begin(), contour.end(), [](const auto &pnt) {
    return (pnt.x == 0 || pnt.y == 0);
  });
}


bool contourTouchesBorderMax(const CvContour &contour, const cv::Size border) {
  const int X_MAX = border.width - 1;
  const int Y_MAX = border.height - 1;

  return std::any_of(contour.begin(), contour.end(), [&X_MAX, &Y_MAX](const auto &pnt) {
    return (pnt.x >= X_MAX || pnt.y >= Y_MAX);
  });
}

bool contourTouchesBorder(const CvContour &contour, const cv::Size &border) {
  return contourTouchesBorderMin(contour) || contourTouchesBorderMax(contour, border);
}

bool contourTouchesBothSides(const CvContour &contour, const cv::Size &border) {
  return contourTouchesBorderMin(contour) && contourTouchesBorderMax(contour, border);
}

CvContours ContourClassifier::filterContoursTouchingImageBorders(
    const Features &features, CvContours *contours, CvHierarchy *hierarchy) const {
  assert(contours->size() == hierarchy->size());

  const int INVALID = -1;

  std::vector<unsigned int> contours_to_remove;
  const auto contours_in_size = contours->size();

  CvContours removed_valid_outer_contours;

  // find root contours that touch border
  for (unsigned int i = 0; i < contours->size(); ++i) {
    auto &h = (*hierarchy)[i];
    auto &contour = (*contours)[i];
    if (h[3] == INVALID) {  // root node
      // check if one of the contour points touches a border
      if (contourTouchesBorder(contour, features.image_patch.image.size())) {
        contours_to_remove.push_back(i);

        // fix for junctions

        if (!contourTouchesBothSides(contour, features.image_patch.image.size())) {
          removed_valid_outer_contours.push_back(contour);
        }

        // remove children as well
        for (unsigned int j = 0; j < contours->size(); j++) {
          const auto &h_inner = hierarchy->at(j);

          if (h_inner[3] == static_cast<int>(i))  // check parent
            contours_to_remove.push_back(j);
        }
      }
    }
  }

  // make sure that erase below receives sorted vector
  contours_to_remove = boost::sort(contours_to_remove);

  // remove contours and hierarchies
  for (int idx : boost::range::reverse(contours_to_remove)) {
    contours->erase(contours->begin() + idx);
    hierarchy->erase(hierarchy->begin() + idx);
  }

  // create LUT (mapping 'old index' -> 'new index') for hierarchy indices
  std::vector<unsigned int> lut;
  lut.resize(contours_in_size);

  std::iota(lut.begin(), lut.end(), 0);  // 0, 1, 2, 3, ...

  for (unsigned int idx : contours_to_remove) {
    for (unsigned int i = idx; i < lut.size(); ++i) {
      lut[i]--;
    }
  }

  // update hierarchy indices
  for (cv::Vec4i &h : *hierarchy) {
    for (unsigned int j = 0; j < 4; ++j) {
      if (h[j] == INVALID) {
        continue;  // nothing to do...
      } else if (common::contains(contours_to_remove, h[j])) {
        h[j] = INVALID;
      } else {
        h[j] = lut[h[j]];
      }
    }
  }

  ROS_DEBUG_STREAM("Filtered " << contours_in_size - contours->size() << " contours.");

  return removed_valid_outer_contours;
}

const common::CameraTransformation *ContourClassifier::get_camera_transformation() {
  return this->camera_transformation_;
}


}  // namespace contour_classifier

}  // namespace road_object_detection
