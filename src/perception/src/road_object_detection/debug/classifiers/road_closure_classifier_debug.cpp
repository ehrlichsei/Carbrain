#include "road_closure_classifier_debug.h"
#include "opencv_eigen_conversions.h"

#include "common/basic_statistics_eigen.h"
#include "common/discretisize.h"
#include "common/eigen_utils.h"
#include "common/minmax_element.h"
#include "common/polynomialfit.h"
#include "distinct_colors.h"
#include "vehicle_scan_line.h"
// THIRD_PARTY_HEADERS_BEGIN
//#include <boost/range/adaptor/transformed.hpp>
// THIRD_PARTY_HEADERS_END

namespace road_object_detection {

RoadClosureClassifierDebug::RoadClosureClassifierDebug(DebugImages *debug_images,
                                                       const common::CameraTransformation *cam_transform,
                                                       ParameterInterface *parameter_interface)
    : ClassifierDebug(debug_images),
      RoadClosureClassifier(cam_transform, parameter_interface) {}

RoadObjects RoadClosureClassifierDebug::classify(const Features &features) {
  RoadObjects return_objects = RoadClosureClassifier::classify(features);

  return return_objects;
}

std::pair<VehiclePoints, VehiclePoints> RoadClosureClassifierDebug::splitCluster(
    const FeaturePointCluster &refined_cluster) const {
  const auto ret = RoadClosureClassifier::splitCluster(refined_cluster);
  //  for (const auto &p : ret.first) {
  //    cv::circle(*debug_images->getCameraImage(),
  //               toCV(camera_transform->transformGroundToImage(p)),
  //               5,
  //               cv::Scalar(255, 255, 0));
  //  }
  return ret;
}

boost::optional<RCLine> RoadClosureClassifierDebug::searchBoundary(
    const VehiclePoints &road_closure_boundary, const Features &features) const {
  const auto line = RoadClosureClassifier::searchBoundary(road_closure_boundary, features);

  //  cv::circle(*debug_images->getCameraImage(),
  //             toCV(camera_transform->transformGroundToImage(common::mean(road_closure_boundary))),
  //             5,
  //             cv::Scalar(255, 0, 0),
  //             -1);

//  if (line) {
//    cv::line(*debug_images->getCameraImage(),
//             toCV(camera_transform->transformGroundToImage(to3D(line->start_))),
//             toCV(camera_transform->transformGroundToImage(to3D(line->end_))),
//             cv::Scalar(0, 255, 255),
//             5);
//  }
  return line;
}

void RoadClosureClassifierDebug::visualize(const ImagePoints &feature_points,
                                           const cv::Scalar &color) const {
  for (const ImagePoint &feature_point : feature_points) {
    cv::circle(*debug_images->getCameraImage(), toCV(feature_point), 3, color, 1);
  }
}

RCLines RoadClosureClassifierDebug::extractInnerLines(const std::vector<FeaturePointCluster> &inner_clusters,
                                                      const RCLine &bounding_line) const {
  // visualize clusters
  //  const auto colors = perception::nDistinctColors(inner_clusters.size());
  //  for (unsigned int i = 0; i < inner_clusters.size(); i++) {
  //    visualize(inner_clusters[i].feature_points_img, colors[i]);
  //  }

  const auto lines_out =
      RoadClosureClassifier::extractInnerLines(inner_clusters, bounding_line);

//  for (const auto &line : lines_out) {

//    ScanLine spot_line = transformGroundToImage(
//        camera_transform, VehicleScanLine(to3D(line.start_), to3D(line.end_)));
//    ScanLine::clip(debug_images->getCameraImage()->size(), spot_line);
//    cv::line(*debug_images->getCameraImage(),
//             imagePointToCvPoint(spot_line.start),
//             imagePointToCvPoint(spot_line.end),
//             cv::Scalar(0, 0, 255),
//             2);
//  }

  return lines_out;
}

VehiclePoints RoadClosureClassifierDebug::removeClosePointsPolynomial(
    const VehiclePoints &ref_points,
    const VehiclePoints &cluster_points,
    const double thresh,
    const common::PolynomialDegree poly_deg) const {
  const auto poly = common::fitToPoints(ref_points, poly_deg);

//  const auto minmax =
//      common::minmax_element(common::join(ref_points, cluster_points) | common::x_values);

//  VehiclePoints pp;
//  const common::DiscretizationParams params{*minmax.first, *minmax.second, 0.02};
//  common::discretisize(poly, params, &pp);

//  for (const auto &p : pp) {
//    cv::circle(*debug_images->getCameraImage(),
//               toCV(camera_transform->transformGroundToImage(p)),
//               1,
//               cv::Scalar(0, 0, 255),
//               -1);
//  }

  return RoadClosureClassifier::removeClosePointsPolynomial(
      ref_points, cluster_points, thresh, poly_deg);
}

VehiclePoints RoadClosureClassifierDebug::searchAdditionalFeaturePoints(
    const FeaturePointCluster &dominant_cluster, const Features &features) const {
  const auto ret =
      RoadClosureClassifier::searchAdditionalFeaturePoints(dominant_cluster, features);

  //  for (const auto &p : ret) {
  //    cv::circle(*debug_images->getCameraImage(),
  //               toCV(camera_transform->transformGroundToImage(p)),
  //               5,
  //               cv::Scalar(0, 255, 255),
  //               -1);
  //  }

  return ret;
}

ScanLines RoadClosureClassifierDebug::createAdditionalScanlines(
    const FeaturePointCluster &dominant_cluster, const Features &features) const {
  const auto ret =
      RoadClosureClassifier::createAdditionalScanlines(dominant_cluster, features);

//  for (const auto &l : ret) {
//    cv::line(
//        *debug_images->getCameraImage(), toCV(l.start), toCV(l.end), cv::Scalar(255, 0, 255), 1);
//  }

  return ret;
}

}  // namespace road_object_detection
