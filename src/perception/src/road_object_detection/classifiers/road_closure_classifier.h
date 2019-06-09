#ifndef ROADCLOSURECLASSIFIER_H
#define ROADCLOSURECLASSIFIER_H

#include "../classifier.h"
#include "common/parameter_interface.h"

THIRD_PARTY_HEADERS_BEGIN
#include <boost/optional.hpp>

#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/polygon.hpp>
THIRD_PARTY_HEADERS_END

#include "../../utils/dbscan_clusterer.h"
#include "vehicle_scan_line.h"

namespace road_object_detection {

using namespace boost::geometry;
using PolygonPoint = model::d2::point_xy<float>;
using PolygonPoints = std::vector<PolygonPoint>;
using Polygon = model::polygon<PolygonPoint, false>;

using LineEquation = Eigen::ParametrizedLine<double, 2>;
using IntersectionLine = Eigen::Hyperplane<double, 2>;

struct RCLine {

  LineEquation line_;

  mutable Eigen::Vector2d start_;

  mutable Eigen::Vector2d end_;

  void align(const Eigen::Vector2d &dir) const {
    const auto swapped = start_.dot(dir) > end_.dot(dir);
    if (swapped) {
      const Eigen::Vector2d tmp = start_;
      start_ = end_;
      end_ = tmp;
    }
  }
};

using RCLines = std::vector<RCLine>;

class RoadClosureClassifier : public virtual Classifier {
 public:
  RoadClosureClassifier(const common::CameraTransformation *cam_transform,
                        ParameterInterface *parameter_interface);

  virtual ~RoadClosureClassifier() override = default;

  virtual RoadObjects classify(const Features &features) override;
  virtual size_t getClassifierId() const final override;

  static const std::string NAMESPACE;

 protected:
  virtual std::pair<VehiclePoints, VehiclePoints> splitCluster(const FeaturePointCluster &refined_cluster) const;

  virtual RCLines fitLinesR(const VehiclePoints &points,
                            const Eigen::Vector2d &prior_direction,
                            const double min_dist,
                            const double angle_eps = M_PI,
                            const std::size_t min_set_size = 6,
                            const std::size_t nr_lines = 1) const;

  virtual boost::optional<RCLine> searchBoundary(const VehiclePoints &road_closure_boundary,
                                                 const Features &features) const;

  virtual RCLines extractInnerLines(const std::vector<FeaturePointCluster> &inner_clusters,
                                    const RCLine &bounding_line) const;

  VehiclePoints localizeRoadClosure(const Features &features,
                                    const boost::optional<RCLine> &inner_boundary) const;

  RoadObjects returnRoadClosure(const Features &features,
                                const boost::optional<RCLine> &inner_boundary,
                                const double score) const;

  //    double checkBaseAreaConsistency(const Features& features, const
  //    VehiclePoints& base_hull)const;
  virtual VehiclePoints searchAdditionalFeaturePoints(const FeaturePointCluster &dominant_cluster,
                                                      const Features &features) const;

  virtual ScanLines createAdditionalScanlines(const FeaturePointCluster &dominant_cluster,
                                              const Features &features) const;
  VehiclePoints removeMiddleLanePointsInCluster(const Features &features) const;

  VehiclePoints removeClosePoints(const VehiclePoints &ref_points,
                                  const VehiclePoints &cluster_points,
                                  const double thresh) const;

  virtual VehiclePoints removeClosePointsPolynomial(const VehiclePoints &ref_points,
                                                    const VehiclePoints &cluster_points,
                                                    const double thresh,
                                                    const common::PolynomialDegree poly_deg) const;

  bool isRoadClosurePlausible(const VehiclePoints &complete_points,
                              const Features &features) const;

  bool enoughPointsOutside(const VehiclePoints &targets, const VehiclePoints &reference) const;

  double computeArcLength(const Features &features) const;

  //  VehiclePoints removePointsLeftOfCenter(const VehiclePoints& points, const Features& features) const;

  const common::CameraTransformation *camera_transform;

  const ParameterInterface *const parameter_interface_;

  DBScanClusterer line_clusterer_, refinement_clusterer_;

  static const ParameterString<double> FILTER_DISTANCE_MIDDLE_LANE;
  static const ParameterString<double> FILTER_DISTANCE_RIGHT_LANE;
  static const ParameterString<double> HORIZON;
  static const ParameterString<int> MIN_SIZE_ADDITIONAL_OUTSIDE;

  static const ParameterString<double> FEATURE_POINT_DETECTION_THLD;
  static const ParameterString<int> STEP_DETECTION_REF_FUNC_LENGTH;

  static const ParameterString<double> ADDITIONAL_SCAN_LINE_WINDOW_WIDTH;
  static const ParameterString<double> ADDITIONAL_SCAN_LINE_STEP_Y;

  static const ParameterString<double> RANSAC_MIN_DIST_BOUNDARY;
  static const ParameterString<double> RANSAC_ALLOWED_ANGLE_DEVIATION;
  static const ParameterString<double> RANSAC_MAX_DIST_INNER;
  static const ParameterString<int> RANSAC_NR_LINES;
  static const ParameterString<int> RANSAC_MIN_SET_SIZE;

  static const ParameterString<double> FRONTLINE_ANGLE;
  static const ParameterString<double> INNER_LINE_ANGLE;
  static const ParameterString<double> MINIMAL_AREA_SIZE_X;
  static const ParameterString<int> MIN_SIZE_LINES;
  static const ParameterString<double> MIN_LATERAL_DISTANCE_TO_MP;
};

}  // namespace road_object_detection

#endif  // ROADCLOSURECLASSIFIER_H
